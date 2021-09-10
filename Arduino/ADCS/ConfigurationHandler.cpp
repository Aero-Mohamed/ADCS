
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <math.h>
#include <SoftwareSerial.h>
#include "ConfigurationHandler.h"
#include "BTS7960.h"

#define LogCommands 0


bool ConfigurationHandler::init(HardwareSerial *BlueTooth_Serial, uint8_t *send_Telemetry, uint16_t *set_speed, K *Speed_Control_Gains, BTS7960 *bts_7960, uint8_t *control_Speed_Flag){
    BT_Serial = BlueTooth_Serial;
    sendTelemetry = send_Telemetry;
    desiredSpeed = set_speed;
    SpeedControlGains = Speed_Control_Gains;
    bts7960 = bts_7960;
    controlSpeedFlag = control_Speed_Flag;
    return true;
}

void ConfigurationHandler::monitor(){
    Int16U cmd[3];
    int i, j;
    // Read Data
    for(i = 0; i<3 ; i++){
        cmd[i].value[0] = readInt8();
        cmd[i].value[1] = readInt8();
    }

    // Empty Buffer
//    while (BT_Serial->available() && BT_Serial->read()); // empty buffer
//    BT_Serial->flush();
    

    // Verify Data
    crc.reset();
    crc.setPolynome(0x1021);
    for(j = 0; j<3; j++){
      for(i = 0; i<2; i++){
        crc.add(cmd[j].value[i]);
      }
    }
    
    if(crc.getCRC() != 0){
      *sendTelemetry = 1;
      return; // Continue if CRC check pass ..
    }

    if(LogCommands){
      Serial.println();
      Serial.print("CRC: OK");
      //Serial.print(crc.getCRC());
      Serial.print(" Commands: ");
      Serial.print("\t"); Serial.print(cmd[0].x);
      Serial.print("\t"); Serial.print(cmd[1].x);
      Serial.print("\t"); Serial.println(cmd[2].x);
    }
    
    /**
     * Telemtery Sending Control Command    
     */

    if(cmd[0].x == 0xA1){
      switch(cmd[1].x){
        case 0x01: // Send Telemetry
          *sendTelemetry = 1;
          break;
        case 0x02: // Stop Telemetry
        default:
          *sendTelemetry = 0;
          break;
      }
    }else{
      *sendTelemetry = 1;
    }

    if(cmd[0].x == 0xA2){
      switch(cmd[1].x){
        case 0x01: // start Control action
          *controlSpeedFlag = 1;
          if(LogCommands){
            Serial.println("Control On");
          }
          break;
        case 0x02: // end control action
        default:
          *controlSpeedFlag = 0;
          if(LogCommands){
            Serial.println("Control Off");
          }
          break;
      }
    }

    /**
     * Control Speed Gains [K , Ki, Kp]
     */
    // K Value
    if(cmd[0].x == 0xB1){
      SpeedControlGains->k = cmd[1].x/(float)1000;
      if(LogCommands){
        Serial.print("K = "); Serial.println(SpeedControlGains->k);
      }
    }
    // Ki Value
    if(cmd[0].x == 0xB2){
      SpeedControlGains->ki = cmd[1].x/(float)1000;
      if(LogCommands){
        Serial.print("Ki = "); Serial.println(SpeedControlGains->ki);
      }
    }
    // Kp Value
    if(cmd[0].x == 0xB3){
      SpeedControlGains->kd = cmd[1].x/(float)1000;
      if(LogCommands){
        Serial.print("Kd = "); Serial.println(SpeedControlGains->kd);
      }
    }


    /**
     * Motor Direct Control
     */
     // Run Motor Test
    if(cmd[0].x == 0xC1){
      bts7960->runMotorTest();
    }
    // Motor Emergency Stop
    if(cmd[0].x == 0xC2){
      bts7960->MotorStop();
    }
    // Run Motor At Given PWM
    if(cmd[0].x == 0xC3){
      if(cmd[1].x > 255){
        bts7960->SetMotorSpeed(100, 1);
      }else{
        bts7960->SetMotorSpeed(cmd[1].x, 1);
      }     
    }
    
    
    /**
     * Set Refrance or Desired Speed
     */
    if(cmd[0].x == 0x02){
      *desiredSpeed = cmd[1].x;
      if(LogCommands){
        Serial.print("Desired Speed: ");Serial.println(*desiredSpeed);
      }
    }

    cmd[0].x = 0;
    cmd[1].x = 0;
    cmd[2].x = 0;

}

uint16_t ConfigurationHandler::readInt16(){
  CMD.value[0] =0;
  CMD.value[1] =0;
  for(int i = 0; i<2; i++){
    CMD.value[i] = BT_Serial->read();
  }
  return (uint16_t)CMD.x;
}

uint8_t ConfigurationHandler::readInt8(){
  return uint8_t(BT_Serial->read());
}
