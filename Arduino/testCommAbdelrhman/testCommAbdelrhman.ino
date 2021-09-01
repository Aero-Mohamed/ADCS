#include <SoftwareSerial.h>
SoftwareSerial BTSerial(10,11);
void setup() {
BTSerial.begin(115200) ;
Serial.begin(115200) ;
BTSerial.flush() ;
}

typedef union packetType{
  
   struct{
    unsigned short int x ;
    unsigned short int y ;
   }data;
   byte dataStream[4];
};
packetType packet ;

void loop() {
if (BTSerial.available()){
  for (int i = 0;i<4;i++){
    packet.dataStream[i] = BTSerial.read() ;
    Serial.print(BTSerial.available()) ;Serial.print(" ") ;
  }
  Serial.print(packet.data.x) ;Serial.print(" ") ;
  Serial.print(packet.data.y) ;Serial.println(" ") ;
}
}
