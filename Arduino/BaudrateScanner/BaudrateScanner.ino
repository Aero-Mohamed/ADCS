#include <SoftwareSerial.h>

/**
 * Macros
 */
#define hc_Rx 10
#define hc_Tx 11

//define software serial: RX, TX
SoftwareSerial hc06(hc_Rx, hc_Tx);

void setup() {
  //Initialize Serial Monitor
  Serial.begin(9600);
  // scan
  scanBaudrate();
}
void loop(){}

/**
 * Send AT Command For multiple bauds To check
 */
void scanBaudrate() {
  unsigned long bauds[12] = {300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 74880, 115200, 230400, 250000};
  for (int i = 0; i < 12; i++) {
    hc06.begin(bauds[i]);
    delay(10);
    Serial.print(F("Baudrate "));
    Serial.println(bauds[i]);
    command("AT", 2000);
  }
}

/**
 * Send Command And Wait For Response
 */
String command(const char *toSend, unsigned long milliseconds) {
  String result;
  Serial.print("Sending: ");
  Serial.println(toSend);
  hc06.print(toSend);
  unsigned long startTime = millis();
  Serial.print(F("Received: "));
  while (millis() - startTime < milliseconds) {
    if (hc06.available()) {
      char c = hc06.read();
      Serial.write(c);
      result += c;
    }
  }
  Serial.println();
  return result;
}
