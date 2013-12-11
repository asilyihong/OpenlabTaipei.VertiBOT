/********************************************************
// Based on:
// VARESANO's Free IMU library. The hard work is done. Fabio, We won't forget you! http://www.varesano.net/projects/hardware/FreeIMU
// Brett Beauregard. Introducing the Pid.Good PID library and good guide http://goo.gl/QKANE
// Patrick Olsson.X-firm System Projects. Best, Balancing guide. http://www.x-firm.com/
// Jason Dorweiler http://www.jddorweiler.appspot.com/electronics.html
// Much of the code is adapted from http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1284738418/all
// and http://www.kerrywong.com/2012/03/08/a-self-balancing-robot-i
// Bildr 6Dof IMu notes and library.http://bildr.org/2012/03/stable-orientation-digital-imu-6dof-arduino/
// C. J.Fisher. Using and accelerometer for inclination sensing. Analog Device. AN-1057
// Shane Colton. A Simple solution for Balance Filter. MIT. June 2007
// J.A. Shaw. PID Algorithm & Tuning Methods. Rochester,NY.

// Code,parts, video, and diagrams available in: http://madebyfrutos.wordpress.com/2013/05/02/vertibot/

// Made(by)Frutos http://madebyfrutos.wordpress.com/
// Ocero El Bierzo, Mayo'13

 ********************************************************/

//#include <PID_v1.h>

#include <SoftwareSerial.h>
#include <PololuQik.h>    
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <Wire.h>

#define BUTTON_INPUT  2
#define MOTOR_CLOCKW  7
#define MOTOR_CCLOCKW 8
#define SKIP_INTERVAL 0

const int Motor_E1 = 5; // digital pin 5 of Arduino (PWM)
const int Motor_E2 = 6; // digital pin 6 of Arduino (PWM)

#define BUTTON_INPUT 2
int Motor_M1;
int Motor_M2;
// 此兩個變數用來定義馬達正反轉的腳位，要做動態調整所以不定義初始值

int motorSpeed = 0;
// 這一個變數用來控制馬達轉速(pwm電壓)

// Define Variables we'll be connecting to
double Setpoint, Input, Output;
// Aggressive
double aggK=1, aggKp=50, aggKi=1.5, aggKd=-8; 
// Conservative
double consK=1, consKp=15, consKi=0.005, consKd=3;
// Specify the links and initial tuning parameters
// PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
PololuQik2s9v1 qik(2, 3, 4);
unsigned long serialTime=0; //this will help us know when to talk with processing
float angles[3]; // yaw pitch roll
// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();
float error = 0;
int STD_LOOP_TIME = 9;   
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0;
int a = 1;
int buttonState = 0;

void setup() {
    Serial.begin(9600);
    Setpoint = 97;
    qik.init();
    Wire.begin();  
    delay(1000);
    sixDOF.init(); //begin the IMU 
    
    Serial.println("setup 1");
    pinMode(BUTTON_INPUT, INPUT);
    int btnState = LOW;
    
    while((btnState = digitalRead(BUTTON_INPUT)) == LOW) {
        sixDOF.getEuler(angles);
        Setpoint=abs(angles[2]);
        Serial.print("Setpoint: ");
        Serial.println(Setpoint);
    }
}

void loop() {
    sixDOF.getEuler(angles);
    Input = abs(angles[2]);
    /* Change the tuning parameters: use Conservative Tuning Parameters when we're near
       setpoint and more agressive Tuning Parameters when we're farther away */ 
    double gap = abs(Setpoint - Input); // distance away from setpoint
    if (gap < 10) { 
        // we're close to setpoint, use conservative tuning parameters
        // myPID.SetTunings(consKp, consKi, consKd);
        Output = updatePid(Setpoint, Input, consK, consKp, consKi, consKd);
    } else {
        // we're far from setpoint, use aggressive tuning parameters
        // myPID.SetTunings(aggKp, aggKi, aggKd);
        Output = updatePid(Setpoint, Input, aggK, aggKp, aggKi, aggKd);
    }

    if (Input < 15) {    
        Output = 0; 
    }

    if (Output > SKIP_INTERVAL) {
        motorSpeed = map(Output, 20, 127, 200, 255);    // 則motorSpeed(馬達轉速) 以map函數以0~400的區域轉換成255~110
        Motor_M1 = MOTOR_CLOCKW;                        // 這兩行代表的是一個旋轉方向，數字對調就是另一個
        Motor_M2 = MOTOR_CCLOCKW;
    } else if (Output < -SKIP_INTERVAL) {
        motorSpeed = map(Output, -127, -20, 200, 255);
        Motor_M1 = MOTOR_CCLOCKW;                       // 這兩行代表的是一個旋轉方向，數字對調就是另一個
        Motor_M2 = MOTOR_CLOCKW;
    } else {
        motorSpeed = 0;
    }

    qik.setSpeeds(Output, Output);
    // asil add
    digitalWrite(Motor_M1, HIGH);       // 數位寫入馬達正反轉
    analogWrite(Motor_E1, motorSpeed);  // 類比寫入馬達轉速
    digitalWrite(Motor_M2, LOW);        // 數位寫入馬達正反轉
    analogWrite(Motor_E2, motorSpeed);  // 類比寫入馬達轉速

    if(millis() > serialTime) {
        SerialSend();
        serialTime += 500;
    }

    //*********************** loop timing control **************************
    lastLoopUsefulTime = millis() - loopStartTime;
    if(lastLoopUsefulTime < STD_LOOP_TIME)
        delay(STD_LOOP_TIME - lastLoopUsefulTime);
    lastLoopTime = millis() - loopStartTime;
    loopStartTime = millis();
} //END LOOP
