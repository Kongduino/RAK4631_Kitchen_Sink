#include <Arduino.h>
#include <bluefruit.h>
#include <SX126x-RAK4630.h> //http://librarymanager/All#SX126x
#include <SPI.h>
#include "Wire.h"
#include <U8g2lib.h>  // Click here to get the library: http://librarymanager/All#U8g2
#include "Utilities.h"

void setup() {
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, HIGH);
  delay(100);
  Serial.begin(115200);
  Wire.begin();
  strcpy(header, "Kitchen Sink");
  myBuff.push_back(header);
  myBuff.push_back(header);
  time_t timeout = millis();
  while (!Serial) {
    if ((millis() - timeout) < 5000) {
      delay(100);
    } else {
      break;
    }
  }
  delay(1000);
  Serial.println("=====================================");
  Serial.println("         Kitchen Sink");
  Serial.println("=====================================");
  Wire.begin();
  hasOLED = u8g2.begin();
  if (hasOLED) {
    u8g2.clearBuffer(); // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.setCursor(0, 0);
    u8g2.print(header);
    u8g2.sendBuffer(); // transfer internal memory to the display
  }
  sprintf(buff, "%d known devices.", numProducts);
  Serial.println(buff);
  memset(result, 0, 128);
  byte addr;
  uint8_t nDevices, ix = 0, success = 0, failure = 0;
  Serial.println("\nI2C scan in progress...");
  nDevices = 0;
  u8g2.setCursor(0, 30);
  for (addr = 1; addr < 128; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    // Wire.beginTransmission(addr);
    // error = Wire.endTransmission();
    if (error == 0) {
      result[addr] = addr;
      addresses[nDevices] = addr;
      nDevices++;
    }
  }
  Serial.printf("I2C devices found: %d\n", nDevices);
  sprintf(buff, "%d devices found", nDevices);
  addStringLine(buff);
  hexDump(result, 128, 32);
  u8g2.setCursor(0, 30);
  for (uint8_t i = 0; i < nDevices; i++) {
    addr = addresses[i];
    for (ix = 0; ix < numProducts; ix++) {
      if (knownProducts[ix].address == addr) {
        sprintf(buff, "Device at %02x is %s, ie %s", addr, knownProducts[ix].code, knownProducts[ix].name);
        Serial.println(buff);
        sprintf(buff, "%d: %s %s", i, knownProducts[ix].code, knownProducts[ix].shortName);
        addStringLine(buff, 1);
        if (knownProducts[ix].ptr()) success += 1;
        else failure += 1;
        delay(500);
        ix = numProducts;
      }
    }
  }
  Serial.printf("%d devices, success: %d, failure: %d\n", nDevices, success, failure);
  sprintf(buff, "%d devices found", nDevices);
  addStringLine(buff);
  sprintf(buff, "Success: %d", success);
  addStringLine(buff);
  sprintf(buff, "Failure: %d", failure);
  addStringLine(buff);
} /* setup() */

void loop() {

}
