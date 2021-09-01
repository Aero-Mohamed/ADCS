
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <math.h>
#include <SoftwareSerial.h>
#include "ConfigurationHandler.h"

void monitor();


bool ConfigurationHandler::init(SoftwareSerial *BlueTooth_Serial, uint8_t *send_Telemetry, uint16_t *set_speed){
    BT_Serial = BlueTooth_Serial;
    sendTelemetry = send_Telemetry;
    desiredSpeed = set_speed;
    return true;
}

void ConfigurationHandler::monitor(){
    uint16_t cmd[2];
    int y =  BT_Serial->available() ;
    
    for(int i = 0;i<2 ; i++){
        cmd[i] = readInt16();
//        Serial.print("Remenaing: ");
//        Serial.println(BT_Serial->available());
    }

    
    // Empty Buffer
    while(BT_Serial->available());
    BT_Serial->flush();
//    
    Serial.println(cmd[0]);
    Serial.println(cmd[1]);
    if(cmd[0] == 0x01){
      switch(cmd[1]){
        case 0x01:
          *sendTelemetry = 1;
          break;
        case 0x02:
        default:
          *sendTelemetry = 0;
          break;
      }
    }
    
    if(cmd[0] == 0x03){
      Serial.print("Desired Speed: ");
      Serial.println(*desiredSpeed);
    }

    if(cmd[0] == 0x02){
      *desiredSpeed = cmd[1];
    }

    if(cmd[0] == 0x04){
      switch(cmd[1]){
        case 0x01: 
          *sendTelemetry = 1;
          break;
        case 0x02:
        default:
          *sendTelemetry = 0;
          break;
      }
    }
    cmd[0] = 0;
    cmd[1] = 0;
}

uint16_t ConfigurationHandler::readInt16(){
  for(int i = 0; i<2; i++){
    CMD.value[i] = BT_Serial->read();
  }
  return (uint16_t)CMD.x;
}

uint8_t ConfigurationHandler::readInt8(){
  return uint8_t(BT_Serial->read());
}
