// for flashing:
// avrdude -v -v -v -patmega328p -cstk500v1 -P/dev/ttyACM0 -b19200 -Uflash:w:./firmware.hex:i
#include <RF24.h>
#include <SPI.h>
#include <PID_v1.h>


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
#define A_1 7
#define A_2 8
#define B_PWM 5
#define B_1 4
#define B_2 3

// RF init
#define RF_SETUP 0x17
RF24 radio(CE, CSN);
// buffer for sending
float send_buffer[3] = { 0.0 };
// pipes for sending and receiving
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0x7365727631LL };

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
unsigned long RF_timer;
unsigned long pitch_timer;
int s_count;


// sensor calibration
float zeroValues[4] = { 0.0 };

// current pitch
float pitch;

// Motor setup
float m_speed = 0;
float m_dir = 0;

// PID vars
float set_point = 93.7;
float Kp = 25;
float Ki = 5;
float Kd = 0;

PID a_pid(&pitch, &m_speed, &set_point, Kp, Ki, Kd, DIRECT);



void setup(){
    // use AREF voltage for ADC
    analogReference(EXTERNAL);
    
    // init PID
    a_pid.SetMode(AUTOMATIC);
    a_pid.SetOutputLimits(-255, 255);
    a_pid.SetSampleTime(10);

    // set sensor pins to input
    pinMode(AccX, INPUT);
    pinMode(AccY, INPUT);
    pinMode(AccZ, INPUT);
    pinMode(GyroX, INPUT);

    pinMode(CSN, OUTPUT);
    pinMode(CE, OUTPUT);

    pinMode(A_PWM, OUTPUT);
    pinMode(A_1, OUTPUT);
    pinMode(A_2, OUTPUT);

    pinMode(B_PWM, OUTPUT);
    pinMode(B_1, OUTPUT);
    pinMode(B_2, OUTPUT);

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
    // ignore listening mode by commenting (send only)
    radio.startListening();
    delay(500);

    calibrateSensors();

    pitch_timer = micros(); 
    RF_timer= micros(); 
    s_count = 0;

}

void loop(){

    float x_angle = get_acc_angle();
    float gyro_rate = get_gyro();

    float gyro_angle = ((micros()-pitch_timer)/1000000.0) * gyro_rate;
    pitch_timer = micros();
    gyro_angle += pitch;

    // complementary filter
    pitch = (.95 * gyro_angle) + (.05 * x_angle);


    if (pitch > 70 && pitch < 115){
        a_pid.Compute();
    }
    else{
        m_speed=0;
    }
    set_speed(m_speed);



    if (((micros() - RF_timer)/1000000.0) > .25){
        send_buffer[0] = x_angle;
        send_buffer[1] = pitch;
        send_buffer[2] = m_speed;
        // receive before sending
        // doesn't work the other way around
        nRF_rec();
        nRF_send();
        a_pid.SetTunings(Kp, Ki, Kd);
        RF_timer = micros();
    }

    // get to a fixed loop time of 10ms
    //last_useful_time = micros() - loop_start_time;
    //while( (micros() - loop_start_time) < STD_LOOP_TIME){
        //true; // dat
    //}

    //loop_start_time = micros();
}

void set_speed(float speed){

    speed = constrain(speed, -255, 255);
    int w_speed = abs(floor(speed));
    if (w_speed < 20){
        w_speed = 0;
    }
    m_speed = speed;


    //analogWrite(A_PWM, 0);
    //analogWrite(B_PWM, 0);

    if (speed < 0){
        digitalWrite(A_1, LOW);
        digitalWrite(A_2, HIGH);

        digitalWrite(B_1, LOW);
        digitalWrite(B_2, HIGH);
    }
    else if (speed > 0 ){
        digitalWrite(A_2, LOW);
        digitalWrite(A_1, HIGH);

        digitalWrite(B_2, LOW);
        digitalWrite(B_1, HIGH);
    }
    else{
        digitalWrite(A_2, LOW);
        digitalWrite(A_1, LOW);

        digitalWrite(B_2, LOW);
        digitalWrite(B_1, LOW);

        analogWrite(A_PWM, 0);
        analogWrite(B_PWM, 0);
    }

    analogWrite(A_PWM, w_speed);
    analogWrite(B_PWM, w_speed);
    
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

    if (zeroValues[3] > 450){ 
        // +1g when lying on one side
        zeroValues[3] -= acc_conv; 
        pitch = 0;
    } 
    else {
        // -1g when lying on the other side
        zeroValues[3] += acc_conv; 
        pitch = 180;
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

void nRF_send(){
    // need to turn into some kind of queue
    // so that stuff can be added without taking
    // extra time and then sent all at once in
    // bursts

    char temp[8];

    // Clear the outBuffer before every loop
    char outBuffer[32]="";

        
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
        //send_buffer[i] = 0;
    }
    
    // Stop listening and write to radio 
    radio.stopListening();
    
    //// Send to hub
    radio.write( outBuffer, strlen(outBuffer));

    radio.startListening();

}

void nRF_rec(){
   
    char receivePayload[31];
    uint8_t len = 0;
    uint8_t pipe = 0;
    char* reply;
        
        
    // Loop thru the pipes 0 to 5 and check for payloads    
    if ( radio.available( &pipe ) ) {
      bool done = false;
      while (!done)
      {
        len = radio.getDynamicPayloadSize();  
        done = radio.read( &receivePayload,len );
        
        // Sending back reply to sender using the same pipe
        radio.stopListening();
        //radio.openWritingPipe(pipes[pipe]);
        radio.write(receivePayload,len);
        radio.startListening();
        float recv = atof(receivePayload);
        if (recv < 100){
            Kp = recv;
        }
        if (recv >= 100 && recv < 200){
            Ki = recv - 100;
        }
        if (recv >= 200 && recv < 300){
            Kd = recv - 200;
        }
        if (recv >= 300 && recv < 400){
            set_point = recv - 300;
        }


        if (recv == 999){
            calibrateSensors();
            send_buffer[0] = 999;
            send_buffer[1] = 999;
            send_buffer[2] = 999;
        }
        // Format string for printing ending with 0
        /*receivePayload[len] = 0;*/
        
        // Increase pipe and reset to 0 if more than 5
        pipe++;
        if ( pipe > 1 ) pipe = 0;
      }

    }

}


int freeRam () {
    extern int __heap_start, *__brkval; 
    int v; 
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
