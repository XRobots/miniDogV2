#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <PID_v1.h>  //PID loop from http://playground.arduino.cc/Code/PIDLibrary

double Pk1 = 2;  
double Ik1 = 0;
double Dk1 = 0;

double Setpoint1, Input1, Output1;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup

double Pk2 = 2;  
double Ik2 = 3;
double Dk2 = 0;

double Setpoint2, Input2, Output2;    // PID variables
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);    // PID Setup

double Pk3 = 2;  
double Ik3 = 3;
double Dk3 = 0;

double Setpoint3, Input3, Output3;    // PID variables
PID PID3(&Input3, &Output3, &Setpoint3, Pk3, Ik3 , Dk3, DIRECT);    // PID Setup

RF24 radio(27, 10); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

//IMU stuff
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
int requested_state;   

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float pitch;
float roll;
float rollFiltered;

#define PI 3.1415926535897932384626433832795

int IMUdataReady = 0;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

Servo servo5;
Servo servo6;
Servo servo7;
Servo servo8;

Servo servo9;
Servo servo10;
Servo servo11;
Servo servo12;

//**************remote control****************
struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO 

    int16_t menuDown;      
    int16_t Select; 
    int16_t menuUp;  
    int16_t toggleBottom;  
    int16_t toggleTop; 
    int16_t toggle1; 
    int16_t toggle2; 
    int16_t mode;  
    int16_t RLR;
    int16_t RFB;
    int16_t RT;
    int16_t LLR;
    int16_t LFB;
    int16_t LT;
    int16_t checkit;
    int16_t checkit2;
};

RECEIVE_DATA_STRUCTURE mydata_remote;

int RLR;
int RFB;
int RT;
int LLR;
int LFB;
int LT;
int toggleTop;
int toggleBottom;
int toggle1;
int toggle2;
int Select;

int x;
int y;
int z;

int yaw;
int r;
int p;


// timers

unsigned long currentMillis;
unsigned long previousMillis;
unsigned long previousWalkMillis;
unsigned long previousSafetyMillis;

// There is a first order filter running on all motions, allbeit turned down a lot. It's also used for the reactivity of the compliance

float multiplierKneesRight = 2;
float multiplierShouldersRight = 2;
float multiplierHipsRight = 0;

float multiplierKneesLeft = 2;
float multiplierShouldersLeft = 2;
float multiplierHipsLeft = 0;

// filters values

int filterKneesRight = 15;
int filterShouldersRight = 15;
int filterHipsRight = 25;

int filterKneesLeft = 15;
int filterShouldersLeft = 15;
int filterHipsLeft = 25;

// global joint threshold value for hall effects
int threshholdGlobal =  5;

// values to write out to servos

float servo1Pos;    // initial value
float servo2Pos;
float servo3Pos;
float servo4Pos;
float servo5Pos;
float servo6Pos;
float servo7Pos;
float servo8Pos;
float servo9Pos;
float servo10Pos;
float servo11Pos;
float servo12Pos;

float servo1PosTrack;   // ongoing tracking value
float servo2PosTrack;
float servo3PosTrack;
float servo4PosTrack;
float servo5PosTrack;
float servo6PosTrack;
float servo7PosTrack;
float servo8PosTrack;
float servo9PosTrack;
float servo10PosTrack;
float servo11PosTrack;
float servo12PosTrack;

float servo1PosFiltered;  // filtered values
float servo2PosFiltered;
float servo3PosFiltered;
float servo4PosFiltered;
float servo5PosFiltered;
float servo6PosFiltered;
float servo7PosFiltered;
float servo8PosFiltered;
float servo9PosFiltered;
float servo10PosFiltered;
float servo11PosFiltered;
float servo12PosFiltered;

// *******************************************************************************
// servo offsets for default position - knees at 45', hips at 0', shoulders at 45'
// *******************************************************************************

int servo1Offset = 1480;      // these are really important to get as accurate as possible
int servo2Offset = 1570;      // they are used as home positions of all servos
int servo3Offset = 1500;
int servo4Offset = 1600;

int servo5Offset = 1500;
int servo6Offset = 1340;
int servo7Offset = 1600;
int servo8Offset = 1570;

int servo9Offset = 1630;
int servo10Offset = 1550;
int servo11Offset = 1550;
int servo12Offset = 1400;

// *******************************************************************************

// hall effect sensors - not used in miniDog V2 but the coe is still included

float hall1;
float hall2;
float hall3;
float hall4;
float hall5;
float hall6;
float hall7;
float hall8;
float hall9;
float hall10;
float hall11;
float hall12;

// offset values for auto-calibration at startup

int hall1Offset;
int hall2Offset;
int hall3Offset;
int hall4Offset;
int hall5Offset;
int hall6Offset;
int hall7Offset;
int hall8Offset;
int hall9Offset;
int hall10Offset;
int hall11Offset;
int hall12Offset;

// modes

int mode = 0;

// walk test positions

int initStart;
int state;
float rate;

int targetLeg1x;
int targetLeg1z;
int prevLeg1x;
int prevLeg1z;
float currentLeg1x;
float currentLeg1z;
float stepDiffLeg1x;
float stepDiffLeg1z;

int targetLeg2x;
int targetLeg2z;
int prevLeg2x;
int prevLeg2z;
float currentLeg2x;
float currentLeg2z;
float stepDiffLeg2x;
float stepDiffLeg2z;

int targetLeg3x;
int targetLeg3z;
int prevLeg3x;
int prevLeg3z;
float currentLeg3x;
float currentLeg3z;
float stepDiffLeg3x;
float stepDiffLeg3z;

int targetLeg4x;
int targetLeg4z;
int prevLeg4x;
int prevLeg4z;
float currentLeg4x;
float currentLeg4z;
float stepDiffLeg4x;
float stepDiffLeg4z;

int targetLeg1y;
int prevLeg1y;
float currentLeg1y;
float stepDiffLeg1y;

int walkXPos1;
int walkXPos2;
int walkXPos3; 
int walkXPos4; 
int walkXPos5;
int walkXPos6; 
int walkXPos7; 
int walkXPos8; 

int walkYPos1;
int walkYPos2;
int walkYPos3;
int walkYPos4;
int walkYPos5;
int walkYPos6;
int walkYPos7;
int walkYPos8;

int walkZPos1;
int walkZPos2;
int walkZPos3;
int walkZPos4;
int walkZPos5;
int walkZPos6;
int walkZPos7;
int walkZPos8;

void setup() {

    // PID stuff

  PID1.SetMode(AUTOMATIC);              
  PID1.SetOutputLimits(-50, 50);
  PID1.SetSampleTime(10);

  PID2.SetMode(AUTOMATIC);              
  PID2.SetOutputLimits(-50, 50);
  PID2.SetSampleTime(10);

  PID3.SetMode(AUTOMATIC);              
  PID3.SetOutputLimits(-50, 50);
  PID3.SetSampleTime(10);

  // radio stuff

  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00002
  radio.openReadingPipe(1, addresses[1]); // 00001
  radio.setPALevel(RF24_PA_MIN);

  radio.startListening();

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  // initialize device
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(53);
  mpu.setYGyroOffset(-5);
  mpu.setZGyroOffset(51);
  mpu.setXAccelOffset(-2230);
  mpu.setYAccelOffset(-698);
  mpu.setZAccelOffset(2035);  

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      attachInterrupt(26, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));

   
  }

  Serial.begin(115200);

  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  pinMode(A8, INPUT);

  pinMode(A9, INPUT);
  pinMode(A14, INPUT);
  pinMode(A15, INPUT);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  
  pinMode(A3, INPUT);
  pinMode(A12, INPUT);
  pinMode(A13, INPUT);

  // write default positions with delays
  
  servo1.attach(24);      // hips
  servo2.attach(25);
  servo3.attach(5);
  servo4.attach(2);

  servo1.writeMicroseconds(servo1Offset);     // back left hip
  servo2.writeMicroseconds(servo2Offset);     // back right hip
  servo3.writeMicroseconds(servo3Offset);     // front left hip
  servo4.writeMicroseconds(servo4Offset);     // front right hip

  delay(1000);

  servo5.attach(9);       // shoulders
  servo6.attach(29);  
  servo7.attach(6);  
  servo8.attach(1);

  servo5.writeMicroseconds(servo5Offset);     // back left shoulder
  servo6.writeMicroseconds(servo6Offset);     // back right shoulder
  servo7.writeMicroseconds(servo7Offset);     // front left shoulder
  servo8.writeMicroseconds(servo8Offset);     // front right shoulder

  delay(1000);

  servo9.attach(8);       // knees
  servo10.attach(30);
  servo11.attach(7);
  servo12.attach(0);

  servo9.writeMicroseconds(servo9Offset);     // back left knee
  servo10.writeMicroseconds(servo10Offset);    // back right knee
  servo11.writeMicroseconds(servo11Offset);    // front left knee
  servo12.writeMicroseconds(servo12Offset);    // front right knee

  // read hall effects on start up

  hall2Offset = analogRead(A7);     // front left hip         |   lower number is more force
  hall3Offset = analogRead(A8);     // front left shoulder    |   higher number is more force.
  hall4Offset = analogRead(A9);     // front left knee        |   lower number is more force 
   
  hall1Offset = analogRead(A6);     // front right hip        |   higher number is more foce
  hall6Offset = analogRead(A15);    // front right shoulder   |   lower nuber is more force  
  hall5Offset = analogRead(A14);    // front right knee       |   higher number is more force
  
  // back
  // forces are from the ground up
  
  hall7Offset = analogRead(A0);     // back left hip          |   higher number is more force
  hall12Offset = analogRead(A13);   // back left shoulder     |   lower number is more force
  hall11Offset = analogRead(A12);   // back left knee         |   higher number is more force
  
  hall8Offset = analogRead(A1);     // back right hip         |   lower number is more force
  hall9Offset = analogRead(A2);     // back right shoulder    |   higher number is more force
  hall10Offset = analogRead(A3);    // back right knee        |   lower number is more force

}

void loop() {

  currentMillis = millis();
  if (currentMillis - previousMillis >= 10) {  // start timed event
          
        previousMillis = currentMillis;

        // receive radio data
        
        if (radio.available()) {
                    radio.read(&mydata_remote, sizeof(RECEIVE_DATA_STRUCTURE));
                    previousSafetyMillis = currentMillis;
        }

        // check if remote has become disconnected

        if (currentMillis - previousSafetyMillis > 500) {         
            Serial.println("*no remote data* ");
            RLR = 512;
            RFB = 512;
            RT = 512;
            LLR = 512;
            LFB = 512;
            LT = 512;
            toggleTop = 0;
            toggleBottom = 0;
            toggle1 = 0;
            toggle2 = 0;
            Select = 0;
        } 

        // use values if the remote is connected ok
        
        else {
          RLR = mydata_remote.RLR;
          RFB = mydata_remote.RFB;
          RT = mydata_remote.RT;
          LLR = mydata_remote.LLR;
          LFB = mydata_remote.LFB;
          LT = mydata_remote.LT;
          toggleTop = mydata_remote.toggleTop;
          toggleBottom = mydata_remote.toggleBottom;
          toggle1 = mydata_remote.toggle1;
          toggle2 = mydata_remote.toggle2;
          Select = mydata_remote.Select;
        }

        // Zero values to swing around +/- 0

        RLR = RLR - 512;      // get to +/- zero value
        RFB = RFB - 512;
        RT = RT - 512;
        LLR = LLR - 512;
        LFB = LFB - 512;
        LT = LT - 512;

        // Threshold remote data for slop in sticks

        if (RLR > 50) {
          RLR = RLR -50;
        }
        else if (RLR < -50) {
          RLR = RLR +50;
        }
        else {
          RLR = 0;
        }
        //*******
        if (RFB > 50) {
          RFB = RFB -50;
        }
        else if (RFB < -50) {
          RFB = RFB +50;
        }
        else {
          RFB = 0;
        }
        //******
        if (RT > 50) {
          RT = RT -50;
        }
        else if (RT < -50) {
          RT = RT +50;
        }
        else {
          RT = 0;
        }
        //******
        if (LLR > 50) {
          LLR = LLR -50;
        }
        else if (LLR < -50) {
          LLR = LLR +50;
        }
        else {
          LLR = 0;
        }
        //*******
        if (LFB > 50) {
          LFB = LFB -50;
        }
        else if (LFB < -50) {
          LFB = LFB +50;
        }
        else {
          LFB = 0;
        }
        //******
        if (LT > 50) {
          LT = LT -50;
        }
        else if (LT < -50) {
          LT = LT +50;
        }
        else {
          LT = 0;
        }      
       
        // check for IMU inpterrupt

        if (IMUdataReady == 1) {
          readAngles();
        }

        roll = (ypr[1] * 180/M_PI) - 1.5;
        pitch = (ypr[2] * 180/M_PI) + 0.7;

        // modes - use serial 
        
        if (Serial.available()) {       // check for serial data
            char c = Serial.read();
  
            if (c == 'a') {             // no compliance
              mode = 0;
            }
            else if (c == 'b') {        // compliance on
              mode = 1;
            }
            else if (c == 'z') {        // calibrate hall sensors
              mode = 99;
            }
        }

        // mode - use remote

        if (toggleTop == 0) {           // no compliance
          mode = 0;
        }

        else if (toggleTop == 1) {      // compliance on
          mode = 1;
        }

        if (Select == 1) {              // calibrate hall sensors
          mode = 99;
        }        
  

        // manual offset calibration of hall effect sensors (when dog is on the ground)- re-reads offsets

        if (mode == 99) {
            hall2Offset = analogRead(A7);     // front left hip         |   lower number is more force
            hall3Offset = analogRead(A8);     // front left shoulder    |   higher number is more force.
            hall4Offset = analogRead(A9);     // front left knee        |   lower number is more force 
             
            hall1Offset = analogRead(A6);     // front right hip        |   higher number is more foce
            hall6Offset = analogRead(A15);    // front right shoulder   |   lower nuber is more force  
            hall5Offset = analogRead(A14);    // front right knee       |   higher number is more force
            
            // back
            // forces are from the ground up
            
            hall7Offset = analogRead(A0);     // back left hip          |   higher number is more force
            hall12Offset = analogRead(A13);   // back left shoulder     |   lower number is more force
            hall11Offset = analogRead(A12);   // back left knee         |   higher number is more force
            
            hall8Offset = analogRead(A1);     // back right hip         |   lower number is more force
            hall9Offset = analogRead(A2);     // back right shoulder    |   higher number is more force
            hall10Offset = analogRead(A3);    // back right knee        |   lower number is more force
        }

        
        // *** read hall effect sensors on every loop ***
        
        // front
        // forces are from the ground up
        
        hall2 = analogRead(A7) - hall2Offset;     // front left hip         |   lower number is more force
        hall3 = analogRead(A8) - hall3Offset;     // front left shoulder    |   higher number is more force.
        hall4 = analogRead(A9) - hall4Offset;     // front left knee        |   lower number is more force 
         
        hall1 = analogRead(A6) - hall1Offset;     // front right hip        |   higher number is more foce
        hall6 = analogRead(A15) - hall6Offset;    // front right shoulder   |   lower nuber is more force  
        hall5 = analogRead(A14) - hall5Offset;    // front right knee       |   higher number is more force
        
        // back
        // forces are from the ground up
        
        hall7 = analogRead(A0) - hall7Offset;     // back left hip          |   higher number is more force
        hall12 = analogRead(A13) - hall12Offset;   // back left shoulder     |   lower number is more force
        hall11 = analogRead(A12) - hall11Offset;   // back left knee         |   higher number is more force
        
        hall8 = analogRead(A1) - hall8Offset;     // back right hip         |   lower number is more force
        hall9 = analogRead(A2) - hall9Offset;     // back right shoulder    |   higher number is more force
        hall10 = analogRead(A3) - hall10Offset;    // back right knee        |   lower number is more force

        
        // **************** Kinematic Model DEMO ****************************

        if (toggleBottom == 0) {      

            // convert sticks to measurements in mm or degrees
    
            z = map(RT, -460,460,80,274);    // overall height of the robot | Higher number makes the leg longer
            z = constrain(z,130,216);
    
            x = map(RFB, -460,460,-60,60);   // front/back                  | Higher number moves the foot forward
            x = constrain(x,-60,60);
    
            y = map(RLR, -460,460,-60,60);   // side/side                   | Higher number moves the foot left
            y = constrain(y,-60,60);
    
            r = map(LLR, -460, 460, -30, 30);   // ROLL - covert to degrees of rotation
            r = constrain(r,-20,20);
    
            p = (map(LFB, -460, 460, 30, -30))-1;   // PITCH - covert to degrees of rotation  (there is a weird offset on my joystick hence -1)
            p = constrain(p,-20,20);
    
            yaw = map(LT, -460, 460, -30, 30);   // YAW - covert to degrees of rotation (there is a weird offset on my joystick hence -1)
            yaw = constrain(yaw,-20,20);
            
            // print control data for debug
                    
            Serial.print(" MODE: ");
            Serial.print(mode);
            Serial.print(" z: ");
            Serial.print(z);        
            Serial.print(" x: ");
            Serial.print(x);
            Serial.print(" y: ");
            Serial.print(y);
            Serial.print(" r: ");
            Serial.print(r);
            Serial.print(" p: ");
            Serial.print(p);
            Serial.print(" yaw: ");
            Serial.print(yaw);
            Serial.println();

            // send data to kinematic model function, compliance engine, and eventually write out to servos
        
            kinematics (1, mode, x,y-20,z,r,p,yaw);   // front left leg
            kinematics (2, mode, x,y+20,z,r,p,yaw);   // front right leg
            kinematics (3, mode, x,y-20,z,r,p,yaw);   // back left leg
            kinematics (4, mode, x,y+20,z,r,p,yaw);   // back right leg

            initStart = 0;      // set the flag so we know what to do at the start of the walk gait below
            state = 20;          // make sure the walk always starts at the beginning

        }

        // ****************** Start of test walking mode *****************************
        // ********************   DYNAMICALLY STABLE *********************************

        else if (toggleBottom == 1) {     // position legs for walking and define the positions for this gait
                         
                  // define walking positions

                    walkXPos1 = -60;
                    walkXPos2 = 40;  // was 0 in the 'old triangle'
                    walkXPos3 = 60; 
                    walkXPos4 = 40; 
                    walkXPos5 = 20;
                    walkXPos6 = 0; 
                    walkXPos7 = -20; 
                    walkXPos8 = -40;
                  
                    walkZPos1 = 215;    // leg down
                    walkZPos2 = 150;    // leg up
                    walkZPos3 = 215;    // leg down
                    walkZPos4 = 215;    // leg down
                    walkZPos5 = 215;    // leg down
                    walkZPos6 = 215;    // leg down
                    walkZPos7 = 215;    // leg down
                    walkZPos8 = 215;    // leg down

                    // the first time we go to this mode we position the feet to start walking

                    if (initStart == 0) {         // first state of prepairing to walk
                        targetLeg1x = walkXPos6;
                        targetLeg1z = walkZPos6; 
                        targetLeg2x = walkXPos6;   
                        targetLeg2z = walkZPos6;
                        targetLeg3x = walkXPos6;
                        targetLeg3z = walkZPos6;
                        targetLeg4x = walkXPos6;
                        targetLeg4z = walkZPos6; 

                        currentLeg1x = targetLeg1x;
                        currentLeg1z = targetLeg1z;
                        currentLeg2x = targetLeg2x;
                        currentLeg2z = targetLeg2z;
                        currentLeg3x = targetLeg3x;
                        currentLeg3z = targetLeg3z;
                        currentLeg4x = targetLeg4x;
                        currentLeg4z = targetLeg4z;

                        initStart = 1; 
                        state = 20;  

                    }

                 if (toggle1 == 1) {           // start state machine for walking 

                /*
                
                //scale the rate so it ranges from 1 to 9 and it's a float  - test manual rate control
                rate = (float) (RFB*-1)/1000; 
                rate = 4-rate; 

                
                //  IMU rate control - not used in the end
                Setpoint1 = 0;
                Input1 = (pitch*-1)-1;
                PID1.Compute();
                rate = constrain(Output1,3.5,4);
                */

                rate = 2.9;   // hardcode rate    

                // state machine is below for the steps throughout the leg motions

                if (state == 20) {                                                            // *********** THE TWO FIRST STATES ARE ONLY USED ONCE TO GET INTO THE RIGHT POSITION
                      targetLeg1x = walkXPos7;
                      targetLeg1z = walkZPos1;    // legs are down
                      targetLeg2x = walkXPos7;    // legs are back
                      targetLeg2z = walkZPos1;
                      targetLeg3x = walkXPos7;
                      targetLeg3z = walkZPos1;
                      targetLeg4x = walkXPos7;
                      targetLeg4z = walkZPos1;                    
                    if (currentLeg1x >= targetLeg1x) {
                      //state = 21;
                      prevLeg1x = targetLeg1x;
                      prevLeg1z = targetLeg1z;
                      prevLeg2x = targetLeg2x;
                      prevLeg2z = targetLeg2z;
                      prevLeg3x = targetLeg3x;
                      prevLeg3z = targetLeg3z;
                      prevLeg4x = targetLeg4x;
                      prevLeg4z = targetLeg4z;
                      // check we actually get there due to dividing errors
                      currentLeg1x = targetLeg1x;
                      currentLeg1z = targetLeg1z;
                      currentLeg2x = targetLeg2x;
                      currentLeg2z = targetLeg2z;
                      currentLeg3x = targetLeg3x;
                      currentLeg3z = targetLeg3z;
                      currentLeg4x = targetLeg4x;
                      currentLeg4z = targetLeg4z;
                      if (RFB < -50) {             // check we are pushing the hoystick forward before starting to walk
                        state = 21;
                      }
                    }
                }
  
                else if (state == 21) {                                                         // *********** THE TWO FIRST STATES ARE ONLY USED ONCE TO GET INTO THE RIGHT POSITION ***************
                      targetLeg1x = walkXPos4;
                      targetLeg1z = walkZPos2;
                      targetLeg2x = walkXPos1;
                      targetLeg2z = walkZPos6;
                      targetLeg3x = walkXPos1;
                      targetLeg3z = walkZPos6;
                      targetLeg4x = walkXPos4;
                      targetLeg4z = walkZPos2;                    
                    if (currentLeg1x >= targetLeg1x) {
                      state = 0;
                      prevLeg1x = targetLeg1x;
                      prevLeg1z = targetLeg1z;
                      prevLeg2x = targetLeg2x;
                      prevLeg2z = targetLeg2z;
                      prevLeg3x = targetLeg3x;
                      prevLeg3z = targetLeg3z;
                      prevLeg4x = targetLeg4x;
                      prevLeg4z = targetLeg4z;
                      // check we actually get there due to dividing errors
                      currentLeg1x = targetLeg1x;
                      currentLeg1z = targetLeg1z;
                      currentLeg2x = targetLeg2x;
                      currentLeg2z = targetLeg2z;
                      currentLeg3x = targetLeg3x;
                      currentLeg3z = targetLeg3z;
                      currentLeg4x = targetLeg4x;
                      currentLeg4z = targetLeg4z;
                    }
                }
                    
                else if (state == 0) {                                                          // ************ FIRST STATE THAT LOOPS FOR ACTUAL WALKING *************
                      targetLeg1x = walkXPos3;
                      targetLeg1z = walkZPos3;
                      targetLeg2x = walkXPos7;
                      targetLeg2z = walkZPos7;
                      targetLeg3x = walkXPos7;
                      targetLeg3z = walkZPos7;
                      targetLeg4x = walkXPos3;
                      targetLeg4z = walkZPos3;                   
                    if (currentLeg1x >= targetLeg1x) {
                      state = 1;
                      prevLeg1x = targetLeg1x;
                      prevLeg1z = targetLeg1z;
                      prevLeg2x = targetLeg2x;
                      prevLeg2z = targetLeg2z;
                      prevLeg3x = targetLeg3x;
                      prevLeg3z = targetLeg3z;
                      prevLeg4x = targetLeg4x;
                      prevLeg4z = targetLeg4z;
                      // check we actually get there due to dividing errors
                      currentLeg1x = targetLeg1x;
                      currentLeg1z = targetLeg1z;
                      currentLeg2x = targetLeg2x;
                      currentLeg2z = targetLeg2z;
                      currentLeg3x = targetLeg3x;
                      currentLeg3z = targetLeg3z;
                      currentLeg4x = targetLeg4x;
                      currentLeg4z = targetLeg4z;
                    }
                }
    
                else if (state == 1) {
                    targetLeg1x = walkXPos4;
                    targetLeg1z = walkZPos4;
                    targetLeg2x = walkXPos8;
                    targetLeg2z = walkZPos8;
                    targetLeg3x = walkXPos8;
                    targetLeg3z = walkZPos8;
                    targetLeg4x = walkXPos4;
                    targetLeg4z = walkZPos4;
                  if (currentLeg1x <= targetLeg1x) {
                    state = 2;
                    prevLeg1x = targetLeg1x;
                    prevLeg1z = targetLeg1z;
                    prevLeg2x = targetLeg2x;
                    prevLeg2z = targetLeg2z;
                    prevLeg3x = targetLeg3x;
                    prevLeg3z = targetLeg3z;
                    prevLeg4x = targetLeg4x;
                    prevLeg4z = targetLeg4z;
                    // check we actually get there due to dividing errors
                    currentLeg1x = targetLeg1x;
                    currentLeg1z = targetLeg1z;
                    currentLeg2x = targetLeg2x;
                    currentLeg2z = targetLeg2z;
                    currentLeg3x = targetLeg3x;
                    currentLeg3z = targetLeg3z;
                    currentLeg4x = targetLeg4x;
                    currentLeg4z = targetLeg4z;
                   }
                }
    
                else if (state == 2) {
                    targetLeg1x = walkXPos5;
                    targetLeg1z = walkZPos5;
                    targetLeg2x = walkXPos1;
                    targetLeg2z = walkZPos1;
                    targetLeg3x = walkXPos1;
                    targetLeg3z = walkZPos1;
                    targetLeg4x = walkXPos5;
                    targetLeg4z = walkZPos5;
                  if (currentLeg1x <= targetLeg1x) {
                    state = 3;
                    prevLeg1x = targetLeg1x;
                    prevLeg1z = targetLeg1z;
                    prevLeg2x = targetLeg2x;
                    prevLeg2z = targetLeg2z;
                    prevLeg3x = targetLeg3x;
                    prevLeg3z = targetLeg3z;
                    prevLeg4x = targetLeg4x;
                    prevLeg4z = targetLeg4z;
                    // check we actually get there due to dividing errors
                    currentLeg1x = targetLeg1x;
                    currentLeg1z = targetLeg1z;
                    currentLeg2x = targetLeg2x;
                    currentLeg2z = targetLeg2z;
                    currentLeg3x = targetLeg3x;
                    currentLeg3z = targetLeg3z;
                    currentLeg4x = targetLeg4x;
                    currentLeg4z = targetLeg4z;
                  }
                }
    
                else if (state == 3) {
                    targetLeg1x = walkXPos6;
                    targetLeg1z = walkZPos6;
                    targetLeg2x = walkXPos2;
                    targetLeg2z = walkZPos2;
                    targetLeg3x = walkXPos2;
                    targetLeg3z = walkZPos2;
                    targetLeg4x = walkXPos6;
                    targetLeg4z = walkZPos6;
                  if (currentLeg1x <= targetLeg1x) {
                    state = 4;
                    prevLeg1x = targetLeg1x;
                    prevLeg1z = targetLeg1z;
                    prevLeg2x = targetLeg2x;
                    prevLeg2z = targetLeg2z;
                    prevLeg3x = targetLeg3x;
                    prevLeg3z = targetLeg3z;
                    prevLeg4x = targetLeg4x;
                    prevLeg4z = targetLeg4z;
                    // check we actually get there due to dividing errors
                    currentLeg1x = targetLeg1x;
                    currentLeg1z = targetLeg1z;
                    currentLeg2x = targetLeg2x;
                    currentLeg2z = targetLeg2z;
                    currentLeg3x = targetLeg3x;
                    currentLeg3z = targetLeg3z;
                    currentLeg4x = targetLeg4x;
                    currentLeg4z = targetLeg4z;
                  }
                }
  
                else if (state == 4) {
                    targetLeg1x = walkXPos7;
                    targetLeg1z = walkZPos7;
                    targetLeg2x = walkXPos3;
                    targetLeg2z = walkZPos3;
                    targetLeg3x = walkXPos3;
                    targetLeg3z = walkZPos3;
                    targetLeg4x = walkXPos7;
                    targetLeg4z = walkZPos7;
                  if (currentLeg1x <= targetLeg1x) {
                    state = 5;
                    prevLeg1x = targetLeg1x;
                    prevLeg1z = targetLeg1z;
                    prevLeg2x = targetLeg2x;
                    prevLeg2z = targetLeg2z;
                    prevLeg3x = targetLeg3x;
                    prevLeg3z = targetLeg3z;
                    prevLeg4x = targetLeg4x;
                    prevLeg4z = targetLeg4z;
                    // check we actually get there due to dividing errors
                    currentLeg1x = targetLeg1x;
                    currentLeg1z = targetLeg1z;
                    currentLeg2x = targetLeg2x;
                    currentLeg2z = targetLeg2z;
                    currentLeg3x = targetLeg3x;
                    currentLeg3z = targetLeg3z;
                    currentLeg4x = targetLeg4x;
                    currentLeg4z = targetLeg4z;
                  }
                }
  
                else if (state == 5) {
                    targetLeg1x = walkXPos8;
                    targetLeg1z = walkZPos8;
                    targetLeg2x = walkXPos4;
                    targetLeg2z = walkZPos4;
                    targetLeg3x = walkXPos4;
                    targetLeg3z = walkZPos4;
                    targetLeg4x = walkXPos8;
                    targetLeg4z = walkZPos8;
                  if (currentLeg1x <= targetLeg1x) {
                    state = 6;
                    prevLeg1x = targetLeg1x;
                    prevLeg1z = targetLeg1z;
                    prevLeg2x = targetLeg2x;
                    prevLeg2z = targetLeg2z;
                    prevLeg3x = targetLeg3x;
                    prevLeg3z = targetLeg3z;
                    prevLeg4x = targetLeg4x;
                    prevLeg4z = targetLeg4z;
                    // check we actually get there due to dividing errors
                    currentLeg1x = targetLeg1x;
                    currentLeg1z = targetLeg1z;
                    currentLeg2x = targetLeg2x;
                    currentLeg2z = targetLeg2z;
                    currentLeg3x = targetLeg3x;
                    currentLeg3z = targetLeg3z;
                    currentLeg4x = targetLeg4x;
                    currentLeg4z = targetLeg4z;
                  }
                }
  
                else if (state == 6) {
                    targetLeg1x = walkXPos1;
                    targetLeg1z = walkZPos1;
                    targetLeg2x = walkXPos5;
                    targetLeg2z = walkZPos5;
                    targetLeg3x = walkXPos5;
                    targetLeg3z = walkZPos5;
                    targetLeg4x = walkXPos1;
                    targetLeg4z = walkZPos1;
                  if (currentLeg1x <= targetLeg1x) {
                    state = 7;
                    prevLeg1x = targetLeg1x;
                    prevLeg1z = targetLeg1z;
                    prevLeg2x = targetLeg2x;
                    prevLeg2z = targetLeg2z;
                    prevLeg3x = targetLeg3x;
                    prevLeg3z = targetLeg3z;
                    prevLeg4x = targetLeg4x;
                    prevLeg4z = targetLeg4z;
                    // check we actually get there due to dividing errors
                    currentLeg1x = targetLeg1x;
                    currentLeg1z = targetLeg1z;
                    currentLeg2x = targetLeg2x;
                    currentLeg2z = targetLeg2z;
                    currentLeg3x = targetLeg3x;
                    currentLeg3z = targetLeg3z;
                    currentLeg4x = targetLeg4x;
                    currentLeg4z = targetLeg4z;
                  }
                }
  
                else if (state == 7) {
                    targetLeg1x = walkXPos2;
                    targetLeg1z = walkZPos2;
                    targetLeg2x = walkXPos6;
                    targetLeg2z = walkZPos6;
                    targetLeg3x = walkXPos6;
                    targetLeg3z = walkZPos6;
                    targetLeg4x = walkXPos2;
                    targetLeg4z = walkZPos2;
                  if (currentLeg1x >= targetLeg1x) {
                    //state = 0;
                    prevLeg1x = targetLeg1x;
                    prevLeg1z = targetLeg1z;
                    prevLeg2x = targetLeg2x;
                    prevLeg2z = targetLeg2z;
                    prevLeg3x = targetLeg3x;
                    prevLeg3z = targetLeg3z;
                    prevLeg4x = targetLeg4x;
                    prevLeg4z = targetLeg4z;
                    // check we actually get there due to dividing errors
                    currentLeg1x = targetLeg1x;
                    currentLeg1z = targetLeg1z;
                    currentLeg2x = targetLeg2x;
                    currentLeg2z = targetLeg2z;
                    currentLeg3x = targetLeg3x;
                    currentLeg3z = targetLeg3z;
                    currentLeg4x = targetLeg4x;
                    currentLeg4z = targetLeg4z;

                if (RFB < -50) {    // carry on with the loop
                  state = 0;
                }
                else { 
                  state = 30;                                 
                }

                if (state == 30) {                                              // ********* FINAL STATE(S) IN WALKING SO THE LEGS COME BACK TOGETHER AGAIN
                    targetLeg1x = walkXPos6;
                    targetLeg1z = walkZPos6; 
                    targetLeg2x = walkXPos6;   
                    targetLeg2z = walkZPos6;
                    targetLeg3x = walkXPos6;
                    targetLeg3z = walkZPos6;
                    targetLeg4x = walkXPos6;
                    targetLeg4z = walkZPos6; 
                  if (currentLeg1x >= targetLeg1x) {
                    state = 20;                             // go back to the start until we push the joystick again
                    prevLeg1x = targetLeg1x;
                    prevLeg1z = targetLeg1z;
                    prevLeg2x = targetLeg2x;
                    prevLeg2z = targetLeg2z;
                    prevLeg3x = targetLeg3x;
                    prevLeg3z = targetLeg3z;
                    prevLeg4x = targetLeg4x;
                    prevLeg4z = targetLeg4z;
                    // check we actually get there due to dividing errors
                    currentLeg1x = targetLeg1x;
                    currentLeg1z = targetLeg1z;
                    currentLeg2x = targetLeg2x;
                    currentLeg2z = targetLeg2z;
                    currentLeg3x = targetLeg3x;
                    currentLeg3z = targetLeg3z;
                    currentLeg4x = targetLeg4x;
                    currentLeg4z = targetLeg4z;                      
                    }
                }
  
                  }   
                } // end of current state
                
                // *** interpolation divison based on rate ***
    
                stepDiffLeg1x = (targetLeg1x - prevLeg1x)/(5*rate);
                stepDiffLeg1z = (targetLeg1z - prevLeg1z)/(5*rate);
                currentLeg1x = currentLeg1x + stepDiffLeg1x;
                currentLeg1z = currentLeg1z + stepDiffLeg1z;

                stepDiffLeg2x = (targetLeg2x - prevLeg2x)/(5*rate);
                stepDiffLeg2z = (targetLeg2z - prevLeg2z)/(5*rate);
                currentLeg2x = currentLeg2x + stepDiffLeg2x;
                currentLeg2z = currentLeg2z + stepDiffLeg2z;

                stepDiffLeg3x = (targetLeg3x - prevLeg3x)/(5*rate);
                stepDiffLeg3z = (targetLeg3z - prevLeg3z)/(5*rate);
                currentLeg3x = currentLeg3x + stepDiffLeg3x;
                currentLeg3z = currentLeg3z + stepDiffLeg3z;

                stepDiffLeg4x = (targetLeg4x - prevLeg4x)/(5*rate);
                stepDiffLeg4z = (targetLeg4z - prevLeg4z)/(5*rate);
                currentLeg4x = currentLeg4x + stepDiffLeg4x;
                currentLeg4z = currentLeg4z + stepDiffLeg4z;

            
            }   // end of state machine for walk test
  
            // offsets to balance centre of gravity statically
            int offsetX = (offsetX - 5);          // offset legs back/forward to hard coded balance
            int offsetY = offsetY + 20;           // move the legs closer together for better stability

            // offsets based on IMU below
                  
            // vary X translation based on pitch
            Setpoint2 = 0;
            Input2 = pitch;
            PID2.Compute();
            Output2 = constrain(Output2,-20,20);
            offsetX = offsetX + Output2;

            /*
            // vary X translation based on roll - not initially used.
            Setpoint3 = 0;
            Input3 = (roll*-1);
            PID3.Compute();
            y = Output3;
            y = constrain(y, -10, 10);         
            */ 

            y = -3;      // manual bodge to keep it on centre - there's quite a lot of issues getting the home positions of the servos aligned, so use this to bias it
            
            Serial.print(state);
            Serial.print(" , ");
            Serial.print(initStart);
            Serial.print(" , ");
            Serial.print(RFB);
            
            
            Serial.println(); 

            kinematics (1, mode, currentLeg1x+offsetX, 0-offsetY+y, currentLeg1z, r, p, yaw);   // front left leg  
            kinematics (2, mode, currentLeg2x+offsetX, 0+offsetY+y, currentLeg2z, r, p, yaw);   // front right leg
            kinematics (3, mode, currentLeg3x+offsetX, 0-offsetY+y, currentLeg3z, r, p, yaw);   // back left leg  
            kinematics (4, mode, currentLeg4x+offsetX, 0+offsetY+y, currentLeg4z, r, p, yaw);   // back right leg                
        }        

  }   // end of timed loop

} //  end of main loop




