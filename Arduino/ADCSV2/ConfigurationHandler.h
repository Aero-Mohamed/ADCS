/**

*/

#ifndef ConfigurationHandler_h
#define ConfigurationHandler_h
#include "CRC16.h"
#include "BTS7960.h"


#define dataSizeSend 16
#define dataSize 32
#define LPWM 8
#define RPWM 9

typedef  union {
  uint16_t  x;
  byte value[2];
} Int16U;

/**
 * Strut Definition
 * 
 */
struct K {
  float k;
  float kp;
  float ki;
};

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
    // Control Signal Response
    float SpeedControlResponse;
   }data;
   /**
    * Store Byte Representation
    * in dataStream variable
    */
   byte dataStream[dataSize];
} Packet;

typedef union {
  /**
   * The order of the vars inside 
   * struct will be same as the order of
   * recived data in matlab application
   */
   struct{
    float timing;
    // Pitch, Roll and Yaw values
    float yaw;
    // Angular Velocities
    float omegaZ;
    // Control Signal Response
    float SpeedControlResponse;
   }data;
   /**
    * Store Byte Representation
    * in dataStream variable
    */
   byte dataStream[dataSizeSend];
} Packet_Send;



/**
 * Enum Definition
 */
enum ControllerStatus {
  SPEED_CONTROL,
  MOTOR_RUN,
  MOTOR_STOP,
};



class ConfigurationHandler
{
    public:
        bool init(HardwareSerial *BlueTooth_Serial, uint8_t *send_Telemetry, uint16_t *set_speed, K *Speed_Control_Gains, BTS7960 *bts_7960, uint8_t *control_Speed_Flag);
        void monitor();
        uint16_t readInt16();
        uint8_t readInt8();
    private:
        Int16U CMD;
        HardwareSerial *BT_Serial;
        uint8_t *sendTelemetry;
        uint16_t *desiredSpeed;
        CRC16 crc;
        K *SpeedControlGains;
        BTS7960 *bts7960;
        uint8_t *controlSpeedFlag;
        
        

};

#endif
