// Libraries
#include <math.h>
#include <SoftwareSerial.h>

// Macros Definitions
#define BT_Rx 10
#define BT_Tx 11
#define Baud_rate 115200
#define pi 3.14
#define dataSize 8


//define software serial: RX, TX
SoftwareSerial BTSerial(BT_Rx,BT_Tx);

// Types Definition
typedef union {
  /**
   * The order of the vars inside 
   * struct will be same as the order of
   * recived data in matlab application
   */
   struct{
    float timeValue;
    float y;
   }data;
   /**
    * Store Byte Representation
    * in dataStream variable
    */
   byte dataStream[dataSize];
} ourUnion;

// Variables Definition
ourUnion ourUnion1;
byte startCommFlag = 0;

void setup() {
  Serial.begin(Baud_rate) ;
  BTSerial.begin(Baud_rate) ;
}

void loop() {
  // if BlueTooth Serial Is Ready
  if(BTSerial.available()){
      if(BTSerial.read() == 1 && !startCommFlag)
      {
        Serial.println("Connection Established");
        startCommFlag = 1;
        BTSerial.flush();
      }
    }

    if(startCommFlag){
      // Build Up DataStream To Be Sent
      ourUnion1.data.timeValue = micros()/(float)1000000;
      ourUnion1.data.y = sin(2 * pi * ourUnion1.data.timeValue * 0.5);
      
      // Send DataStream, 2 float numbers converted to equavilent bytes representation
      BTSerial.write(ourUnion1.dataStream, dataSize) ;
    }
}
