/**

*/

#ifndef ConfigurationHandler_h
#define ConfigurationHandler_h

typedef  union {
  uint16_t  x;
  byte value[2];
} Int16U;


class ConfigurationHandler
{
    public:
        bool init(SoftwareSerial *BT_Serial, uint8_t *send_Telemetry, uint16_t *set_speed);
        void monitor();
        uint16_t readInt16();
        uint8_t readInt8();
        
    private:
        Int16U CMD;
        SoftwareSerial *BT_Serial;
        uint8_t *sendTelemetry;
        uint16_t *desiredSpeed;
        

};

#endif
