/*
  Hardware:
  ATmega, NRF24L01 (CE=9, CSN=10), 6 analog IR receivers (A0-A5),
  1 IR transmitter on pin 7.
 */

#include <SPI.h>
#include <RF24.h>
#include <math.h>

/* ── config ──────────────────────────────────────────────────────── */
#define MAX_BOTS     6        
#define MY_ID        0        // ← Change per bot (0 to MAX_BOTS-1)
#define SLOT_MS      100      // IR slot duration per bot
#define PHASE_MS     4000     // Duration of each phase

/* ── NRF ─────────────────────────────────────────────────────────── */
RF24 radio(9, 10);
const uint64_t PIPE = 0xE8E8F0F0E1LL;

/* ── message types ───────────────────────────────────────────────── */
#define MSG_PING   0x01
#define MSG_ANGLE  0x02
#define MSG_ASSIGN 0x03

struct Message {
    uint8_t type;
    uint8_t sender;
    uint8_t data[6];
};

/* ── IR ──────────────────────────────────────────────────────────── */
const float   IR_ANGLES[6] = {0, 60, 120, 180, 240, 300};
const uint8_t IR_PINS[6]   = {A0, A1, A2, A3, A4, A5};
#define IR_THRESHOLD  50
#define IR_TX_PIN     7

/* ── state ───────────────────────────────────────────────────────── */
float   bearings[MAX_BOTS];
bool    bearing_known[MAX_BOTS];

float   angle_table[MAX_BOTS][MAX_BOTS];   // Signed angle from i to j, seen from me
bool    angle_known[MAX_BOTS][MAX_BOTS];

float   remote_angles[MAX_BOTS][MAX_BOTS][MAX_BOTS];  // [observer][i][j]
bool    remote_known[MAX_BOTS][MAX_BOTS][MAX_BOTS];

uint8_t g_vertex     = 0;
uint8_t g_n_bots     = 0;
uint8_t vertex_order[MAX_BOTS];

/* ══════════════════════════════════════════════════════════════════
 * MATH HELPERS
 * ══════════════════════════════════════════════════════════════════ */

// Keep angle in (-180, 180]
float normalize(float a) {
    while (a <= -180.0) a += 360.0;
    while (a >   180.0) a -= 360.0;
    return a;
}

// Weighted circular mean of 6 IR receivers → bearing in degrees (0-360)
float readBearing() {
    float sin_sum = 0, cos_sum = 0;
    bool  any     = false;
    for (uint8_t i = 0; i < 6; i++) {
        int val = analogRead(IR_PINS[i]);
        if (val < IR_THRESHOLD) continue;
        float rad = IR_ANGLES[i] * PI / 180.0;
        sin_sum += val * sin(rad);
        cos_sum += val * cos(rad);
        any = true;
    }
    if (!any) return -1;
    float b = atan2(sin_sum, cos_sum) * 180.0 / PI;
    if (b < 0) b += 360;
    return b;
}

// Pack signed float into 2 bytes using bitwise manipulation
void packFloat(float val, uint8_t &b1, uint8_t &b2) {
    int16_t scaled = (int16_t)(val * 100.0);
    b1 = (uint8_t)(scaled >> 8);
    b2 = (uint8_t)(scaled & 0xFF);
}

// Fixed: Explicitly handles uint8_t components to safely restore signed values
float unpackFloat(uint8_t b1, uint8_t b2) {
    uint16_t raw = ((uint16_t)b1 << 8) | b2;
    int16_t scaled = (int16_t)raw;
    return (float)scaled / 100.0;
}

/* ══════════════════════════════════════════════════════════════════
 * NRF HELPERS
 * ══════════════════════════════════════════════════════════════════ */
void sendMsg(Message &msg) {
    radio.stopListening();
    radio.write(&msg, sizeof(msg));
    radio.startListening();
}

bool recvMsg(Message &msg) {
    if (!radio.available()) return false;
    radio.read(&msg, sizeof(msg));
    return true;
}

/* ══════════════════════════════════════════════════════════════════
 * PHASE 1 — SLOTTED IR DISCOVERY
 * ══════════════════════════════════════════════════════════════════ */
void runDiscovery() {
    memset(bearing_known, false, sizeof(bearing_known));
    memset(angle_known,   false, sizeof(angle_known));
    digitalWrite(IR_TX_PIN, LOW);

    bool    ping_sent = false;
    uint8_t last_slot = 0xFF;

    uint32_t start = millis();
    while (millis() - start < PHASE_MS) {
        uint32_t now      = millis() - start;
        uint8_t  active   = (now % ((uint32_t)MAX_BOTS * SLOT_MS)) / SLOT_MS;
        uint32_t elapsed = now % SLOT_MS;

        if (active != last_slot) {
            ping_sent = false;
            last_slot = active;
        }

        if (active == MY_ID) {
            digitalWrite(IR_TX_PIN, HIGH);
            if (elapsed >= SLOT_MS / 2 && !ping_sent) {
                Message ping = {MSG_PING, MY_ID, {}};
                sendMsg(ping);
                ping_sent = true;
            }
        } else {
            digitalWrite(IR_TX_PIN, LOW);
            Message msg;
            if (recvMsg(msg) && msg.type == MSG_PING && msg.sender == active) {
                float b = readBearing();
                if (b >= 0) {
                    bearings[msg.sender]      = b;
                    bearing_known[msg.sender] = true;
                    if ((uint8_t)(msg.sender + 1) > g_n_bots)
                        g_n_bots = msg.sender + 1;
                }
            }
        }
    }
    digitalWrite(IR_TX_PIN, LOW);

    for (uint8_t i = 0; i < MAX_BOTS; i++) {
        if (!bearing_known[i]) continue;
        for (uint8_t j = 0; j < MAX_BOTS; j++) {
            if (i == j || !bearing_known[j]) continue;
            angle_table[i][j] = normalize(bearings[j] - bearings[i]);
            angle_known[i][j] = true;
        }
    }
}

/* ══════════════════════════════════════════════════════════════════
 * PHASE 2 — SHARE ANGLE TABLES
 * ══════════════════════════════════════════════════════════════════ */
void shareAngleTables() {
    memset(remote_known, false, sizeof(remote_known));

    for (uint8_t i = 0; i < MAX_BOTS; i++) {
        for (uint8_t j = 0; j < MAX_BOTS; j++) {
            if (!angle_known[i][j]) continue;
            Message msg;
            msg.type    = MSG_ANGLE;
            msg.sender  = MY_ID;
            msg.data[0] = i;
            msg.data[1] = j;
            packFloat(angle_table[i][j], msg.data[2], msg.data[3]);
            sendMsg(msg);
            delay(5);
        }
    }

    uint32_t start = millis();
    while (millis() - start < PHASE_MS) {
        Message msg;
        if (!recvMsg(msg) || msg.type != MSG_ANGLE) continue;
        uint8_t from = msg.sender;
        uint8_t i    = msg.data[0];
        uint8_t j    = msg.data[1];
        if (from < MAX_BOTS && i < MAX_BOTS && j < MAX_BOTS) {
            remote_angles[from][i][j] = unpackFloat(msg.data[2], msg.data[3]);
            remote_known[from][i][j]  = true;
        }
    }
}

/* ══════════════════════════════════════════════════════════════════
 * PHASE 3 — COMPUTE CYCLIC ORDER (Coordinator only)
 * ══════════════════════════════════════════════════════════════════ */
float getAngle(uint8_t observer, uint8_t a, uint8_t b) {
    if (observer == MY_ID && angle_known[a][b]) return angle_table[a][b];
    if (observer < MAX_BOTS && remote_known[observer][a][b])
        return remote_angles[observer][a][b];
    return -999;
}

bool alreadyAssigned(uint8_t id, uint8_t n) {
    for (uint8_t i = 0; i < n; i++)
        if (vertex_order[i] == id) return true;
    return false;
}

void computeCyclicOrder() {
    memset(vertex_order, 0xFF, sizeof(vertex_order));

    // Step 1: Find an exterior corner by minimizing the maximum arc span
    uint8_t start  = 0;
    float   min_span = 360.0;
    for (uint8_t obs = 0; obs < g_n_bots; obs++) {
        float max_angle_found = 0;
        for (uint8_t a = 0; a < g_n_bots; a++) {
            for (uint8_t b = 0; b < g_n_bots; b++) {
                if (a == b || a == obs || b == obs) continue;
                float ang = abs(getAngle(obs, a, b));
                float raw = getAngle(obs, a, b);
                if (raw == -999) continue;
                float ang = abs(raw);
                if (ang != -999 && ang > max_angle_found) {
                    max_angle_found = ang;
                }
            }
        }
        if (max_angle_found < min_span && max_angle_found > 0) {
            min_span = max_angle_found;
            start  = obs;
        }
    }
    vertex_order[0] = start;

    // Step 2: Fix direction profile — smallest positive signed angle
    uint8_t best    = 0xFF;
    float   bestAng = 999;
    for (uint8_t cand = 0; cand < g_n_bots; cand++) {
        if (cand == start) continue;
        for (uint8_t ref = 0; ref < g_n_bots; ref++) {
            if (ref == start || ref == cand) continue;
            float ang = getAngle(start, cand, ref);
            if (ang == -999) continue;
            if (ang > 0 && ang < bestAng) {
                bestAng = ang;
                best    = cand;
            }
        }
    }
    if (best == 0xFF) return;
    vertex_order[1] = best;

    // Step 3: Walk the boundary perimeter using turning angles
    for (uint8_t v = 2; v < g_n_bots; v++) {
        uint8_t cur  = vertex_order[v-1];
        uint8_t prev = vertex_order[v-2];
        uint8_t next = 0xFF;
        float   minTurn = 999;

        for (uint8_t cand = 0; cand < g_n_bots; cand++) {
            if (cand == prev || alreadyAssigned(cand, v)) continue;
            float ang = getAngle(cur, prev, cand);
            if (ang == -999) continue;
            float turn = normalize(180.0 - ang);
            if (turn < minTurn) {
                minTurn = turn;
                next    = cand;
            }
        }
        if (next == 0xFF) break;
        vertex_order[v] = next;
    }
}

/* ══════════════════════════════════════════════════════════════════
 * PHASE 4 — BROADCAST ASSIGNMENTS
 * ══════════════════════════════════════════════════════════════════ */
void broadcastAssignments() {
    uint32_t start = millis();
    while (millis() - start < PHASE_MS) {

        if (MY_ID == 0) {
            for (uint8_t v = 0; v < g_n_bots; v++) {
                if (vertex_order[v] == 0xFF) continue;
                Message msg;
                msg.type    = MSG_ASSIGN;
                msg.sender  = MY_ID;
                msg.data[0] = vertex_order[v];
                msg.data[1] = v + 1;
                sendMsg(msg);
                delay(5);
                if (vertex_order[v] == MY_ID) g_vertex = v + 1;
            }
        }

        Message msg;
        if (recvMsg(msg) && msg.type == MSG_ASSIGN && msg.data[0] == MY_ID)
            g_vertex = msg.data[1];
    }
}

/* ── Entry Points ────────────────────────────────────────────────── */
void setup() {
    Serial.begin(9600);

    pinMode(IR_TX_PIN, OUTPUT);
    digitalWrite(IR_TX_PIN, LOW);

    radio.begin();
    radio.openWritingPipe(PIPE);
    radio.openReadingPipe(1, PIPE);
    radio.setPALevel(RF24_PA_LOW);
    radio.startListening();

    runDiscovery();
    shareAngleTables();

    if (MY_ID == 0) {
        computeCyclicOrder();
    }

    broadcastAssignments();

    Serial.print("My vertex: ");
    Serial.println(g_vertex);
}

void loop() {}
