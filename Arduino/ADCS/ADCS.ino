// Libraries
#include <math.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <MPU6050.h>
#include "ConfigurationHandler.h"
#include "BTS7960.h"

// Macros Definitions
#define BT_Rx 11
#define BT_Tx 10
#define Baud_rate 115200
#define pi 3.14



#define ENA 9
#define IN1 6
#define IN2 7


//define software serial: RX, TX
//SoftwareSerial BTSerial(BT_Rx,BT_Tx);



/**
 * Variables Definition
 */
MPU6050 mpu;  /** MPU Reading */
ConfigurationHandler config_handler; /** Handling Configration & Settings */
BTS7960 bts7960; /** Handling Configration & Settings */
Packet packet; /** Packet That Hold All Data */
Packet_Send packet_send; /** Package That Hold Data To Be Sent */


/**
 * General Control Variables
 */
uint8_t sendTelemetry = 0;
/**
 * Control Variables Definition
 */
// kp, kd, ki
//K SpeedControlGains = {0.025, 0.02, 0.005};
K SpeedControlGains = {0.03, 0.008, 0.006};
uint16_t desiredSpeed = 10;
uint8_t controlSpeedFlag = 0;
float SpeedError, SpeedTotalError, SpeedPreviousError, motorPMW = 0;

K AngleControlGains = {0.0, 0.0, 0.0};
uint16_t desiredAngle = 0;
uint8_t controlAngleFlag = 0;
float AngleError, AngleTotalError, AnglePreviousError, angleRes = 0;


// General vars
unsigned long timer = 0;
// Timers | PID
float currentTime = 0;
float previousTime = 0;
float elapsedTime = 0;

uint8_t bitsAvailable = 0;
uint8_t bitsPrev = 0;
uint8_t deltaBit = 0;


float p, q, r = 0;


void setup() {
//  Serial.begin(Baud_rate);
  Serial1.begin(Baud_rate);
  initMpu();
  bts7960.init(LPWM, RPWM); /** Motor Control Class */
  config_handler.init(&Serial1, &sendTelemetry, &desiredSpeed, &AngleControlGains, &bts7960, &controlSpeedFlag, &controlAngleFlag);
  while (Serial1.available() && Serial1.read()); // empty buffer
}

void loop() {
  timer = millis();
  
  currentTime = (float)millis();
  
  elapsedTime = (currentTime - previousTime)/(float)1000;
  // Read normalized values
  // But first Delay for 10ms MPU6050 does not like to be hit much
  // so give it time to make his mind
  delay(20);
  Vector norm = mpu.readNormalizeGyro();

  float toRads = PI / 180;
  p = norm.XAxis + norm.YAxis*(sin(packet.data.pitch*toRads)*tan(packet.data.roll*toRads)) + norm.ZAxis*(cos(packet.data.pitch*toRads)*tan(packet.data.roll*toRads));
  q = norm.YAxis*cos(packet.data.pitch*toRads) - norm.ZAxis*sin(packet.data.roll*toRads);
  r = norm.YAxis*(sin(packet.data.pitch*toRads)/cos(packet.data.roll*toRads)) + norm.ZAxis*(cos(packet.data.pitch*toRads)/cos(packet.data.roll*toRads));

  
  // store angular velocity
  packet.data.omegaX = p;
  packet.data.omegaY = q;
  packet.data.omegaZ = r;
  
  packet.data.pitch = packet.data.pitch + p * elapsedTime;
  packet.data.roll = packet.data.roll + q * elapsedTime;
  packet.data.yaw = packet.data.yaw + r * elapsedTime;
  packet.data.timing = micros()/(float)1000000;

  packet_send.data.timing = packet.data.timing;
  packet_send.data.yaw = getLimitedAngleReading(packet.data.yaw);
  packet_send.data.omegaZ = packet.data.omegaZ;
//  packet_send.data.omegaZ = getLimitedAngleReading(packet.data.yaw);
//  packet_send.data.SpeedControlResponse = packet_send.data.yaw;

   
  // Track the Configuration and update
  if(Serial1.available()){
    delay(20);
    bitsAvailable = Serial1.available();
    
    deltaBit = abs(bitsAvailable - bitsPrev);
    if(deltaBit == 0){
      config_handler.monitor();
      bitsPrev = 0;
      bitsAvailable = 0;
    }
    bitsPrev = bitsAvailable;
    
  }

  // Log To Serial if needed
  printOutPutToSerial();

  
  /**
   * Speed PID
   */
  if(controlSpeedFlag){
    SpeedPidController();
  }else{
    SpeedPidRest();
  }


  if(controlAngleFlag){
    AngleError = desiredAngle - packet.data.yaw;
    
    angleRes = AngleControlGains.k * (AngleError);
    angleRes += AngleControlGains.ki * (AngleTotalError);
    angleRes += AngleControlGains.kd * (AngleError - AnglePreviousError);
    
    AnglePreviousError = AngleError;
    AngleTotalError += AngleError;

    /**
     * Somehow must be mapped to rotational speed value
     */
    angleRes = angleRes *255/(float)5;

    if(abs(angleRes) >= 15){
      angleRes = 15* (angleRes/abs(angleRes));
    }

    /**
     * Then Control the speed
     */
    desiredSpeed = angleRes;
    SpeedPidController();
  }else{
    AnglePidRest();
    SpeedPidRest();
  }
  


  // not uploaded ..
  if(sendTelemetry){
      Serial1.write(packet_send.dataStream, dataSizeSend) ;
      sendTelemetry = 0;
//      Serial.println("\tSent Data !");
  }else{
//    Serial.println("no Send");
  }

  previousTime = currentTime;
  //delay((timeStep*1000) - (millis() - timer));
}


void SpeedPidRest(){
  SpeedError = 0;
  SpeedTotalError = 0;
  SpeedPreviousError = 0;
  motorPMW = 0;
  bts7960.MotorStop();
}

void AnglePidRest(){
  AngleError = 0;
  AngleTotalError = 0;
  AnglePreviousError = 0;
  motorPMW = 0;
  bts7960.MotorStop();
}

void initMpu(){
  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_500DPS, MPU6050_RANGE_2G))
  {
//    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro(200);

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(0);
}


void printOutPutToSerial(){
//  Serial.println(packet_send.data.omegaZ);
}

void SpeedPidController(){
  SpeedError = desiredSpeed - packet.data.omegaZ;
  
  motorPMW = SpeedControlGains.k * (SpeedError);
  motorPMW += SpeedControlGains.ki * (SpeedTotalError);
  motorPMW += SpeedControlGains.kd * (SpeedError - SpeedPreviousError);
  
  SpeedPreviousError = SpeedError;
  SpeedTotalError += SpeedError;
//    if (SpeedTotalError >8000) SpeedTotalError = 8000;
//    if (SpeedTotalError <-8000) SpeedTotalError = -8000;
  
  motorPMW = motorPMW *255/(float)5;
  if(abs(motorPMW) >= 120){
    motorPMW = 120* (motorPMW/abs(motorPMW));
  }
  
  bts7960.SetMotorSpeed(abs(motorPMW), motorPMW/abs(motorPMW));  
  packet_send.data.SpeedControlResponse = motorPMW;
  
}


//void PID_angle_control(){
//  e = atan2(sin(( (float)desiredAngle - packet.data.yaw), cos( (float)desiredAngle - packet.data.yaw));
//  E = E + e;
//  E_dot = e - e_prev;
//  set_speed(Kp * e + Ki * E + Kd * E_dot);
//  e_prev = e;
//}

float getLimitedAngleReading(float angle){
  return atan2( sin(angle*pi/180), cos(angle*pi/180) )*180/pi;
}
