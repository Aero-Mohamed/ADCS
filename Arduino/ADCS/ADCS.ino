// Libraries
#include <math.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <MPU6050.h>
#include "ConfigurationHandler.h"

// Macros Definitions
#define BT_Rx 10
#define BT_Tx 11
#define Baud_rate 115200
#define pi 3.14
#define dataSize 28


//define software serial: RX, TX
SoftwareSerial BTSerial(BT_Rx,BT_Tx);

/**
 * Types Definition
 */
typedef union {
  /**
   * The order of the vars inside 
   * struct will be same as the order of
   * recived data in matlab application
   */
   struct{
    float timing;
    // Pitch, Roll and Yaw values
    float pitch;
    float roll;
    float yaw;
    // Angular Velocities
    float omegaX;
    float omegaY;
    float omegaZ;
   }data;
   /**
    * Store Byte Representation
    * in dataStream variable
    */
   byte dataStream[dataSize];
} Packet;

/**
 * Enum Definition
 */
enum ControllerStatus {
  SPEED_CONTROL,
  MOTOR_RUN,
  MOTOR_STOP,
};


/**
 * Variables Definition
 */
MPU6050 mpu;
ConfigurationHandler config_handler;
Packet packet;
ControllerStatus status_;

// Timers | angles
unsigned long timer = 0;

// Timers | PID
float currentTime = 0;
float previousTime = 0;
float elapsedTime = 0;


byte startCommFlag = 0;

// System Parameters
uint8_t sendTelemetry = 0;
uint16_t desiredSpeed = 100; 
float K, Ki, Kd = 0;
float p, q, r = 0;

void setup() {
  Serial.begin(Baud_rate) ;
  BTSerial.begin(Baud_rate) ;
  initMpu();
  config_handler.init(&BTSerial, &sendTelemetry, &desiredSpeed);
}

void loop() {
  timer = millis();

  currentTime = (float)millis();
  
  elapsedTime = (currentTime - previousTime)/(float)1000;
  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();

  // store angular velocity
  packet.data.omegaX = norm.XAxis;
  packet.data.omegaY = norm.YAxis;
  packet.data.omegaZ = norm.ZAxis;
  
  //Serial.print("Elapsed Time = ");
  //Serial.println(elapsedTime,5);
  // Integrate Pitch, Roll and Yaw
//  p = norm.XAxis;
//  q = norm.YAxis;
//  r = norm.ZAxis;
  p = norm.XAxis + norm.YAxis*(sin(packet.data.pitch)*tan(packet.data.roll)) + norm.ZAxis*(cos(packet.data.pitch)*tan(packet.data.roll));
  q = norm.YAxis*cos(packet.data.pitch) - norm.ZAxis*sin(packet.data.roll);
  r = norm.YAxis*(sin(packet.data.pitch)/cos(packet.data.roll)) + norm.ZAxis*(cos(packet.data.pitch)/cos(packet.data.roll));

  
  
  packet.data.pitch = packet.data.pitch + p * elapsedTime;
  packet.data.roll = packet.data.roll + q * elapsedTime;
  packet.data.yaw = packet.data.yaw + r * elapsedTime;
  packet.data.timing = micros()/(float)1000000;

  // Log To Serial if needed
  printOutPutToSerial();

  // Track the Configuration and update
  if(BTSerial.available()){
    config_handler.monitor();
  }


  // not uploaded ..
  
  
  

  if(sendTelemetry){
    // Send DataStream, 3 float numbers converted to equavilent bytes representation
    BTSerial.write(packet.dataStream, dataSize) ;
  }

  previousTime = currentTime;
  // Wait to full timeStep period
//  delay((timeStep*1000) - (millis() - timer));
}


void initMpu(){
  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);
}


void printOutPutToSerial(){
//  if(status_ == MOTOR_RUN){
//    Serial.print("Desired Speed: ");
//    Serial.print(speed_desired);
//    Serial.print(" | Current: ");
//    Serial.print(packet.data.omegaZ);  
//    Serial.println(" deg/s");
//  }
  if(sendTelemetry == 1){
    Serial.print(" Pitch = ");
    Serial.print(packet.data.pitch);Serial.print(" ");
    Serial.print(" Roll = ");
    Serial.print(packet.data.roll); Serial.print(" "); 
    Serial.print(" Yaw = ");
    Serial.println(packet.data.yaw);

    Serial.print(" GyroX = ");
    Serial.print(packet.data.omegaX);  
    Serial.print(" GyroY = ");
    Serial.print(packet.data.omegaY);
    Serial.print(" GyroZ = ");
    Serial.println(packet.data.omegaZ); 
  }
}
