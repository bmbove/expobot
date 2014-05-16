// xbox remote controller for expobot
#include <RF24.h>
#include <SPI.h>

// Select and enable pins for RF
#define CSN 10
#define CE 9

// set pins for controller in
int lr_stick = A0;
int ud_stick = A1;

// controller set values
int lr_set = 0;
int ud_set = 0;

// RF init
#define RF_SETUP 0x17
RF24 radio(CE, CSN);
// buffer for sending
float send_buffer = 1000;
// pipes for sending and receiving
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0x7365727631LL };



void setup(){
    
    pinMode(lr_stick, INPUT);
    pinMode(ud_stick, INPUT);

   // RF setup
    radio.begin();

    // Enabling this seems to work better
    radio.enableDynamicPayloads();

    // set rate to 1mbps
    radio.setDataRate(RF24_1MBPS);
    radio.setPALevel(RF24_PA_MAX);
    // set channel to 70
    radio.setChannel(70);
    radio.setRetries(15,15);

    radio.openWritingPipe(pipes[0]); 
    radio.openReadingPipe(1,pipes[1]); 
    // let's give it a sec to settle down
    delay(200);
    // ignore listening mode by commenting (send only)
    //radio.startListening();
}

void loop(){

    int lr_value = analogRead(lr_stick);
    int ud_value = analogRead(ud_stick);

    // this is really really ugly
    if (lr_value >= 600){
        lr_set = 1;
    }
    else if (lr_value <= 400){
        lr_set = -1;
    }
    else{
        lr_set = 0;
    }

    if (ud_value >= 600){
        ud_set = 1;
    }
    else if (ud_value <= 400){
        ud_set = -1;
    }
    else{
        ud_set = 0;
    }

    // and... it gets worse
    if (ud_set == 1 && lr_set == 0){
        send_buffer = 1001;
    }
    else if (ud_set == -1 && lr_set == 0){
        send_buffer = 1002;
    }
    else if (ud_set == 1 && lr_set == -1){
        send_buffer = 1005;
    }
    else if (ud_set == -1 && lr_set == -1){
        send_buffer = 1006;
    }
    else if (ud_set == 1 && lr_set == 1){
        send_buffer = 1003;
    }
    else if (ud_set == -1 && lr_set == 1){
        send_buffer = 1005;
    }
    else if (ud_set == 0 && lr_set == -1){
        send_buffer = 1007;
    }
    else if (ud_set == 0 && lr_set == 1){
        send_buffer = 1008;
    }
    else if (ud_set == 0 && lr_set == 0){
        send_buffer = 1000;
    }
    else{
        send_buffer = 1000;
    }

    nRF_send();

    delay(50);
}

void nRF_send(){
    // need to turn into some kind of queue
    // so that stuff can be added without taking
    // extra time and then sent all at once in
    // bursts

    char temp[8];

    // Clear the outBuffer before every loop
    char outBuffer[32]="";

        
    dtostrf(send_buffer, 8, 2, temp);
    strcat(outBuffer,temp);
    send_buffer = 1000;
    
    // Stop listening and write to radio 
    radio.stopListening();
    
    //// Send to hub
    radio.write( outBuffer, strlen(outBuffer));

    radio.startListening();

}
