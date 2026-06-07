//setup code for MCU on bots

#include <EEPROM.h>
#include <SPI.h>
#include <RF24.h>

const uint8_t CE_PIN =  ;
const uint8_t CSN_PIN = ;

const byte BROADCAST_ADDR[6] = "SWARM";
const byte BOT_ADDR[6][6] = {
    "BOT_1","BOT_2","BOT_3","BOT_4","BOT_5","BOT_6"
};
int BOT_ID = ;

struct Command{
   // command packet (to be recieved)
};

RF 24 radio(CE_PIN, CSN_PIN);

void setup(){
    //pinModes and other setup
    radio.begin();
    radio.setChannel(108);              // MUST match base station
    radio.setDataRate(RF24_250KBPS);    // MUST match base station
    radio.setPALevel(RF24_PA_LOW);
    radio.setPayloadSize(sizeof(Command));
    radio.setCRCLength(RF24_CRC_16);
    radio.setAutoAck(false);
    
    // Listen on broadcast address (pipe 1)
    radio.openReadingPipe(1, BROADCAST_ADDR);
    // Listen on this bot's private address (pipe 2)
    radio.openReadingPipe(2, BOT_ADDR[BOT_ID - 1]);
    
    radio.startListening(); //bots recieve


}
void loop(){
    if (radio.available()){
        Command cmd;
        radio.read(&cmd, sizeof(cmd));
        // process command and locomotion
        

    }
}