#include "I2Cdev.h"
#include "helper_3dmath.h"
#include "MPU6050.h"
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

MPU6050 mpu;  /** MPU Reading */
ConfigurationHandler config_handler; /** Handling Configration & Settings */
BTS7960 bts7960; /** Handling Configration & Settings */
Packet packet; /** Packet That Hold All Data */
Packet_Send packet_send; /** Package That Hold Data To Be Sent */

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 g;
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


/**
 * General Control Variables
 */
uint8_t sendTelemetry = 0;
/**
 * Control Variables Definition
 */
K SpeedControlGains = {0, 0, 0};
uint16_t desiredSpeed = 0; 
uint8_t controlSpeedFlag = 0;
float SpeedError, SpeedTotalError, SpeedPreviousError = 0;


// General vars
unsigned long timer = 0;
uint8_t bitsAvailable = 0;
uint8_t bitsPrev = 0;
uint8_t deltaBit = 0;

uint16_t desiredAngle = 0;


void setup() {
//    Serial.begin(Baud_rate);
    Serial1.begin(Baud_rate);
    mpu_init();
    bts7960.init(LPWM, RPWM); /** Motor Control Class */
    config_handler.init(&Serial1, &sendTelemetry, &desiredSpeed, &SpeedControlGains, &bts7960, &controlSpeedFlag);
}

void loop() {    
  // if programming failed, don't try to do anything
  if (!dmpReady) {
    Serial.println();
    Serial.println("Program Failed ! ... \t");
    Serial.print(dmpReady);
    return;
  }
  
  if (dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    // Then Read from MPU
    delay(20);
    GetMPUReadings();
  
    // store angular velocity
    packet.data.omegaX = g.x / 16.4* 180/M_PI;
    packet.data.omegaY = g.y / 16.4* 180/M_PI;
    packet.data.omegaZ = g.z / 16.4* 180/M_PI;
    
    packet.data.pitch = ypr[1] * 180/M_PI;
    packet.data.roll = ypr[2] * 180/M_PI;
    packet.data.yaw = ypr[0] * 180/M_PI;
    packet.data.timing = micros()/(float)1000000;
  
    packet_send.data.timing = packet.data.timing;
    packet_send.data.yaw = packet.data.yaw;
    packet_send.data.omegaZ = packet.data.omegaZ;
    packet_send.data.SpeedControlResponse = packet.data.yaw;
    // if needed..
    PrintReadingsToSerial();
  }

    // Track the Configuration and update
  if(Serial1.available()){
    delay(20);
    config_handler.monitor();
    
    
//    bitsAvailable = Serial1.available();
//    
//    deltaBit = abs(bitsAvailable - bitsPrev);
//    if(deltaBit == 0){
//      
//      bitsPrev = 0;
//      bitsAvailable = 0;
//    }
//    bitsPrev = bitsAvailable;
    
  }
  
  // Log To Serial if needed
  //printOutPutToSerial();
  /**
   * Speed PID
   */

  if(controlSpeedFlag){
    SpeedError = desiredSpeed - packet.data.omegaZ;
    SpeedTotalError += SpeedError;
    packet.data.SpeedControlResponse = SpeedControlGains.k * (SpeedError);
    packet.data.SpeedControlResponse += SpeedControlGains.ki * (SpeedTotalError);
    packet.data.SpeedControlResponse += SpeedControlGains.kp * (SpeedError - SpeedPreviousError);
    SpeedPreviousError = SpeedError;
    packet_send.data.SpeedControlResponse = packet.data.SpeedControlResponse*255/(float)5;
    if(abs(packet_send.data.SpeedControlResponse) >= 35){
      packet_send.data.SpeedControlResponse = 35*packet_send.data.SpeedControlResponse/abs(packet_send.data.SpeedControlResponse);
    }
  
    
    bts7960.SetMotorSpeed(abs(packet_send.data.SpeedControlResponse), packet_send.data.SpeedControlResponse/abs(packet_send.data.SpeedControlResponse));  
  }

  // not uploaded ..
  if(sendTelemetry){
      Serial1.write(packet_send.dataStream, dataSizeSend) ;
      sendTelemetry = 0;
//    sendTelemetry--;
//      Serial.println("\tSent Data !");
  }else{
    
  }
}

void PrintReadingsToSerial(){
//  Serial.println(packet.data.omegaZ);
//  Serial.print("Gx = ");Serial.print(g.x / 16.4);
//  Serial.print("\t Gy = ");Serial.print(g.y / 16.4);
//  Serial.print("\t Gz = ");Serial.println(g.z / 16.4);
//  Serial.print("Ax = ");Serial.print(aa.x / 8192.0 * 9.8);
//  Serial.print("\t Ay = ");Serial.print(aa.y / 8192.0 * 9.8);
//  Serial.print("\t Az = ");Serial.println(aa.z / 8192.0 * 9.8);
//  Serial.print("Pitch = ");Serial.print(ypr[1] * 180/M_PI);
//  Serial.print("\t Roll = ");Serial.print(ypr[2] * 180/M_PI);
//  Serial.print("\t Yaw = ");Serial.println(ypr[0] * 180/M_PI);
}

void GetMPUReadings(){
  mpu.dmpGetGyro(&g, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
}


void mpu_init(){
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    mpu.initialize();
    mpu.testConnection();
    devStatus = mpu.dmpInitialize();
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);
        dmpReady = true;
    }else{
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
//      Serial.print(F("DMP Initialization failed (code "));
//      Serial.print(devStatus);
//      Serial.println(F(")"));
    }
}
