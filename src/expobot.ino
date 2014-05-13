#include <RF24.h>
#include <Kalman.h>

#define PI 3.14159265359
#define RAD_TO_DEG 57.2957795
// ADC reference voltage
#define AREF 3.41
// fixed loop time of 10ms
#define STD_LOOP_TIME 10000

// Select and enable pins for RF
#define CSN 10
#define CE 9


// Motor Controller
#define A_PWM 6
#define A1 7
#define A2 8
#define B_PWM 5
#define B1 4
#define B2 3

// RF init
#define RF_SETUP 0x17
RF24 radio(CE, CSN);
// buffer for sending
float send_buffer[3] = { 0 };
// pipes for sending and receiving
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0x7365727631LL };

// init kalman filter
Kalman kalman;


// sensor inputs
int AccX = A0;
int AccY = A1;
int AccZ = A2;
int GyroX = A3;

// sensor conversions
// gyro: 2mV per degree/s
// acc: 800mv per g
float gyro_conv = (.002/AREF) * 1024;
float acc_conv = (.800/AREF) * 1024;


// loop timing
unsigned long last_useful_time = STD_LOOP_TIME;
unsigned long loop_start_time;
unsigned long timer;
int s_count;


// sensor calibration
float zeroValues[4] = { 0 };

// current pitch
float pitch;



void setup(){
    // use AREF voltage for ADC
    analogReference(EXTERNAL);

    // set sensor pins to input
    pinMode(AccX, INPUT);
    pinMode(AccY, INPUT);
    pinMode(AccZ, INPUT);
    pinMode(GyroX, INPUT);

    pinMode(CSN, OUTPUT);
    pinMode(CE, OUTPUT);

    pinMode(A_PWM, OUTPUT);
    pinMode(A1, OUTPUT);
    pinMode(A2, OUTPUT);

    pinMode(B_PWM, OUTPUT);
    pinMode(B1, OUTPUT);
    pinMode(B2, OUTPUT);

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
    delay(1000);

    // ignore listening mode by commenting (send only)
    radio.startListening();

    calibrateSensors();

    loop_start_time = micros();
    timer = loop_start_time;
    s_count = 0;

}

void loop(){

    float x_angle = get_acc_angle();
    float gyro_rate = get_gyro();

    pitch = kalman.getAngle(x_angle, gyro_rate, (micros()-timer)/100000.0);
    timer = micros();

    send_buffer[0] = pitch;
    send_buffer[1] = x_angle;
    send_buffer[2] = gyro_rate;

    last_useful_time = micros() - loop_start_time;
    while( (micros() - loop_start_time) < STD_LOOP_TIME){
        true; // dat
    }

    loop_start_time = micros();
}

void calibrateSensors(){
    // take average of 100 readings
    for (uint8_t i = 0; i < 100; i++){
        zeroValues[0] += analogRead(GyroX);
        zeroValues[1] += analogRead(AccX);
        zeroValues[2] += analogRead(AccY);
        zeroValues[3] += analogRead(AccZ);
        delay(10);
    }

    zeroValues[0] /= 100; // Gyro X-axis
    zeroValues[1] /= 100; // Accelerometer X-axis
    zeroValues[2] /= 100; // Accelerometer Y-axis
    zeroValues[3] /= 100; // Accelerometer Z-axis  

    if (zeroValues[3] > 650){ 
        // +1g when lying on one side
        zeroValues[3] -= acc_conv; 
        pitch = 0;
        kalman.setAngle(0);
    } 
    else {
        // -1g when lying on the other side
        zeroValues[3] += acc_conv; 
        pitch = 180;
        kalman.setAngle(180);
    }
}

float get_acc_angle() {

    float accXval = ((float)analogRead(AccX) - zeroValues[1]);
    float accZval = ((float)analogRead(AccZ) - zeroValues[3]);

    // Convert to 360 degrees resolution
    // atan2 outputs the value of -π to π (radians)
    // Convert it to 0 to 2π and then from radians to degrees
    float x_angle = (atan2(-accXval,-accZval)+PI)*RAD_TO_DEG;

    // get rid of noise around 0
    if (x_angle > 300 ){
        x_angle = x_angle - 360;
    }

    return x_angle; 
}

float get_gyro() {

    float gyro_read = (float)analogRead(GyroX);
    float gyro_rate = gyro_read - zeroValues[0]; 
    gyro_rate = gyro_rate * gyro_conv; 

    // negative to make up for orientation
    return -gyro_rate;
}

void nRF_send()
{
    // need to turn into some kind of queue
    // so that stuff can be added without taking
    // extra time and then sent all at once in
    // bursts

    char temp[8];

    // Clear the outBuffer before every loop
    char outBuffer[32]="";
    unsigned long send_time, rtt = 0;
        
    dtostrf(send_buffer[0], 8, 2, temp);
    strcat(outBuffer,temp);
    strcat(outBuffer,",");
    dtostrf(send_buffer[1], 8, 2, temp);
    strcat(outBuffer,temp);
    strcat(outBuffer,",");
    dtostrf(send_buffer[2], 8, 2, temp);
    strcat(outBuffer,temp);
    strcat(outBuffer,",");

    for (int i=0; i < 3; i++){
        send_buffer[i] = 0;
    }
    
    send_time = millis();
    
    // Stop listening and write to radio 
    radio.stopListening();
    
    // Send to hub
    radio.write( outBuffer, strlen(outBuffer));

    radio.startListening();
    /*delay(100);  */

}

