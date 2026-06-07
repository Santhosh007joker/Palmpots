#include <SPI.h>
#include <RF24.h>


const uint8_t CE_PIN =  ;
const uint8_t CSN_PIN = ;

RF24 radio(CE_PIN, CSN_PIN);
const byte BROADCAST_ADDR[6] = "SWARM";

struct Command {
    //command packet (to be sent)
};

void setup() {
    Serial.begin(9600);         // talk to laptop
    
    radio.begin();
    radio.setChannel(108);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_LOW);
    radio.setPayloadSize(sizeof(Command));
    radio.setCRCLength(RF24_CRC_16);
    radio.setAutoAck(false);
    
    radio.openWritingPipe(BROADCAST_ADDR);
    radio.stopListening();      // base station only transmits
    
    Serial.println("Base station ready.");
}

void loop() {
    if (Serial.available()) {
        // Read shape command from laptop
        
        String input = Serial.readStringUntil('\n');
        
        //interpret and transmit command to bots using radio.write()
    }
}

