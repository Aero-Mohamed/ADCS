//
//    FILE: CRC16_test.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: demo
//    DATE: 2021-01-20
//    (c) : MIT

#include "CRC16.h"
#include "CRC.h"

typedef  union {
  uint16_t  x;
  byte value[2];
  uint8_t value_nbr[2];
} Int16U;

Int16U var[3];

CRC16 crc;

void setup()
{
  Serial.begin(115200);
  Serial.println(__FILE__);

  var[0].value[0] = 0x1;
  var[0].value[1] = 0x0;
  var[1].value[0] = 0x2;
  var[1].value[1] = 0x0;
  var[2].value[0] = 0xF0;
  var[2].value[1] = 0xB6;
  
  test();
}


void loop()
{
}

void test()
{
  Serial.print("Polynome: ");
  Serial.println(0x1021, HEX);
 
  crc.reset();
  crc.setPolynome(0x1021);
  for(int j=0; j < 2; j++){
    for (int i = 0; i < 2; i++)
    {
      crc.add(var[j].value[i]);
      Serial.print(j);
      Serial.print("|");
      Serial.print(i);
      Serial.print("\t");
      Serial.println(crc.getCRC(), HEX);
      
    }
    
  }
  Serial.println("Data: ");
  Serial.println(var[0].x);
  Serial.println(var[1].x);
  Serial.println(var[2].x);
    

//  crc.restart();
//  for (int i = 0; i < 9; i++)
//  {
//    crc.add(str[i]);
//  }
//  Serial.println(crc.getCRC(), HEX);
//  for (int i = 0; i < 9; i++)
//  {
//    crc.add(str[i]);
//  }
//  Serial.println(crc.getCRC(), HEX);
//  Serial.println(crc.count());
}
