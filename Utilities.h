void subStringLine(char*, byte);
void addStringLine(char*, byte);
void refreshScreen();
void hexDump(uint8_t*, uint16_t, uint8_t);

#undef max
#undef min
#include <string>
#include <vector>
#include <cstring>
#include <algorithm>

using namespace std;
namespace std _GLIBCXX_VISIBILITY(default) {
_GLIBCXX_BEGIN_NAMESPACE_VERSION
void __throw_length_error(char const*) {}
void __throw_bad_alloc() {}
void __throw_out_of_range(char const*) {}
void __throw_logic_error(char const*) {}
void __throw_out_of_range_fmt(char const*, ...) {}
}
template class basic_string<char>; //  https:// github.com/esp8266/Arduino/issues/1136

#include "SparkFunLIS3DH.h" // http:// librarymanager/All#SparkFun-LIS3DH
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // http:// librarymanager/All#SparkFun_u-blox_GNSS
#include "SparkFun_MLX90632_Arduino_Library.h"
//  Click here to get the library: http:// librarymanager/AllSparkFun_MLX90632_Arduino_Library
#include "SparkFun_SHTC3.h"  // Click here to get the library: http:// librarymanager/All#SparkFun_SHTC3
#include "UVlight_LTR390.h"  // Click here to get the library: http:// librarymanager/All#RAK12019_LTR390
#include "SparkFun_SHTC3.h"
// Click here to get the library: http://librarymanager/All#SparkFun_SHTC3
#include <SensirionI2cStc3x.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h> // Click here to get the library: http://librarymanager/All#Adafruit_LPS2X

vector<string> myBuff;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2);
bool hasOLED = false;
uint8_t result[128];
uint8_t addresses[128];
char buff[256];
char header[33];

struct rakSensor {
  uint8_t address;
  char shortName[13];
  char code[12];
  char name[64];
  bool (*ptr)(); //  Function pointer
};

bool testRAK1921() {
  addStringLine("  --> OLED test", 1);
  return hasOLED;
}

bool testRAK1904() {
  LIS3DH SensorTwo(I2C_MODE, 0x18);
  //  read the sensor value
  addStringLine("  --> LIS3DH test", 1);
  if (SensorTwo.begin() != 0) {
    Serial.println("Problem starting the sensor at 0x18.");
    addStringLine("  * Error!", 1);
    return false;
  }
  uint8_t data_to_write = 0;
  SensorTwo.readRegister(&data_to_write, LIS3DH_CTRL_REG1);
  data_to_write |= 0x08;
  SensorTwo.writeRegister(LIS3DH_CTRL_REG1, data_to_write);
  delay(100);
  data_to_write = 0;
  SensorTwo.readRegister(&data_to_write, 0x1E);
  data_to_write |= 0x90;
  SensorTwo.writeRegister(0x1E, data_to_write);
  delay(100);
  for (uint8_t cnt = 0; cnt < 3; cnt++) {
    sprintf(buff, "XYZ: %.2f,%.2f,%.2f", SensorTwo.readFloatAccelX(), SensorTwo.readFloatAccelY(), SensorTwo.readFloatAccelZ());
    Serial.println(buff);
    subStringLine(buff, 1);
    delay(1000);
  }
  return true;
}

bool testRAK12500() {
  addStringLine("  --> GNSS test", 1);
  SFE_UBLOX_GNSS g_myGNSS;
  if (g_myGNSS.begin() == false) {
    Serial.println(F("GNSS not detected at default I2C address. Please check wiring."));
    sprintf(buff, " --> No GPS!");
    subStringLine(buff, 1);
    return false;
  } else {
    Serial.println(F("GNSS detected at default I2C address."));
    sprintf(buff, " --> GNSS detected");
    subStringLine(buff, 1);
  }
  delay(1000);
  return true;
}

bool testRAK12003() {
#define MLX90632_ADDRESS 0x3A
  MLX90632 RAK_TempSensor;
  TwoWire &wirePort = Wire;
  MLX90632::status returnError;
  Serial.println(" --> MLX90632 Test");
  sprintf(buff, "MLX90632 Test");
  addStringLine(buff, 1);
  if (RAK_TempSensor.begin(MLX90632_ADDRESS, wirePort, returnError) == false) {
    Serial.println("MLX90632 Init Failed");
    sprintf(buff, " --> MLX90632 failed!");
    subStringLine(buff, 1);
    return false;
  }
  float object_temp = 0;
  object_temp = RAK_TempSensor.getObjectTemp(); // Get the temperature of the object we're looking at
  Serial.printf("Object temperature: %.2f C\n", object_temp);
  float sensor_temp = 0;
  sensor_temp = RAK_TempSensor.getSensorTemp(); // Get the temperature of the sensor
  Serial.printf("Sensor temperature: %.2f C\n", sensor_temp);
  sprintf(buff, "Object: %.2f C", object_temp);
  subStringLine(buff, 1);
  sprintf(buff, "Sensor: %.2f C", sensor_temp);
  addStringLine(buff, 1);
  delay(1000);
  return true;
}

bool testRAK12019() {
  UVlight_LTR390 ltr = UVlight_LTR390();
  sprintf(buff, " --> RAK12019 test");
  addStringLine(buff, 1);
  Serial.println(buff);
  if (!ltr.init()) {
    sprintf(buff, " --> RAK12019 Error");
    subStringLine(buff, 1);
    Serial.println(buff);
    return false;
  }
  ltr.setMode(LTR390_MODE_ALS); // LTR390_MODE_UVS
  ltr.setGain(LTR390_GAIN_3);
  ltr.setResolution(LTR390_RESOLUTION_16BIT);
  ltr.setThresholds(100, 1000); // Set the interrupt output threshold range for lower and upper.
  ltr.configInterrupt(true, LTR390_MODE_ALS); // Configure the interrupt based on the thresholds in setThresholds()
  sprintf(buff, "ALS Test");
  addStringLine(buff, 1);
  for (uint8_t i = 0; i < 3; i++) {
    sprintf(buff, "Lux: %0.2f ALS: %d", ltr.getLUX(), ltr.readALS()); // calculate the lux
    subStringLine(buff, 1);
    Serial.println(buff);
    delay(500);
  }
  ltr.setMode(LTR390_MODE_UVS);
  ltr.setGain(LTR390_GAIN_3);
  ltr.setResolution(LTR390_RESOLUTION_16BIT);
  ltr.setThresholds(100, 1000); //  Set the interrupt output threshold range for lower and upper.
  ltr.configInterrupt(true, LTR390_MODE_UVS); // Configure the interrupt based on the thresholds in setThresholds()
  sprintf(buff, "UVS Test");
  addStringLine(buff, 1);
  for (uint8_t i = 0; i < 3; i++) {
    sprintf(buff, "UVI: %0.2f UVS: %d", ltr.getUVI(), ltr.readUVS()); // calculate the lux
    subStringLine(buff, 1);
    Serial.println(buff);
    delay(500);
  }
  return true;
}

bool testRAK1901() {
  SHTC3 mySHTC3; // Declare an instance of the SHTC3 class
  sprintf(buff, " --> SHTC3 test");
  addStringLine(buff, 1);
  Serial.println(buff);
  SHTC3_Status_TypeDef message = mySHTC3.begin();
  switch (message) {
    case SHTC3_Status_Nominal:
      sprintf(buff, " - SHTC3 OK");
      addStringLine(buff, 1);
      Serial.println(buff);
    case SHTC3_Status_Error:
      sprintf(buff, " - SHTC3 status error");
      addStringLine(buff, 1);
      Serial.println(buff);
      return false;
    case SHTC3_Status_CRC_Fail:
      sprintf(buff, " - SHTC3 CRC error");
      addStringLine(buff, 1);
      Serial.println(buff);
      return false;
    default:
      sprintf(buff, " - SHTC3 other error");
      addStringLine(buff, 1);
      Serial.println(buff);
      return false;
  }
  delay(5000); // Give time to read the welcome message and device ID.
  SHTC3_Status_TypeDef result = mySHTC3.update();
  sprintf(buff, "RH: %.2f%% T: %.2f C", mySHTC3.toPercent(), mySHTC3.toDegC());
  addStringLine(buff, 1);
  Serial.println(buff);
  return true;
}

bool testRAK12008() {
  SensirionI2cStc3x sensor;
  sensor.begin(Wire, 0x2C);
  uint32_t productId = 0;
  uint64_t serialNumber = 0;
  delay(14);
  int16_t error = sensor.getProductId(productId, serialNumber);
  if (error != 0) {
    sprintf(buff, " - STC31 error");
    addStringLine(buff, 1);
    Serial.println(buff);
    return false;
  }
  sprintf(buff, "ID: %08x", productId);
  addStringLine(buff, 1);
  Serial.println(buff);
  sprintf(buff, "SN: %016x", serialNumber);
  addStringLine(buff, 1);
  Serial.println(buff);
  error = sensor.setBinaryGas(19);
  error = sensor.enableWeakFilter();
  float co2Concentration = 0.0;
  float temperature = 0.0;
  delay(3000);
  error = sensor.setRelativeHumidity(50.0);
  error = sensor.measureGasConcentration(co2Concentration, temperature);
  sprintf(buff, "CO2: %.2f", co2Concentration);
  addStringLine(buff, 1);
  Serial.println(buff);
  sprintf(buff, "T: %.2f", temperature);
  addStringLine(buff, 1);
  Serial.println(buff);
  delay(1000);
  return true;
}

bool testRAK12011() {
  Adafruit_LPS22 g_lps22hb;
  if (!g_lps22hb.begin_I2C(0x5d)) {
    sprintf(buff, " - LPS33 error");
    addStringLine(buff, 1);
    Serial.println(buff);
    return false;
  }
  g_lps22hb.setDataRate(LPS22_RATE_10_HZ);
  for (uint8_t i = 0; i < 3; i++) {
    sensors_event_t temp;
    sensors_event_t pressure;
    g_lps22hb.getEvent(&pressure, &temp);
    sprintf(buff, "T: %.2f", temp.temperature);
    addStringLine(buff, 1);
    Serial.println(buff);
    sprintf(buff, "Pa: %.2f HPa", pressure.pressure);
    addStringLine(buff, 1);
    Serial.println(buff);
    delay(1000);
  }
  return true;
}

bool testRAK1906() {
  return true;
}
bool testRAK12032() {
  return true;
}
bool testRAK14003() {
  return true;
}
bool testRAK12059() {
  return true;
}
bool testRAK1902() {
  return true;
}
bool testRAK14004() {
  return true;
}
bool testRAK12010() {
  return true;
}
bool testRAK12039() {
  return true;
}
bool testRAK12035() {
  return true;
}
bool testRAK13003() {
  return true;
}
bool testRAK14002() {
  return true;
}
bool testRAK12014() {
  return true;
}
bool testRAK12052() {
  return true;
}
bool testRAK14001() {
  return true;
}
bool testRAK16000() {
  return true;
}
bool testRAK1903() {
  return true;
}
bool testRAK13004() {
  return true;
}
bool testRAK15000() {
  return true;
}
bool testRAK12004() {
  return true;
}
bool testRAK12002() {
  return true;
}
bool testRAK12016() {
  return true;
}
bool testRAK12009() {
  return true;
}
bool testRAK12027() {
  return true;
}
bool testRAK12012() {
  return true;
}
bool testRAK5814() {
  return true;
}
bool testRAK12047() {
  return true;
}
bool testRAK13600() {
  return true;
}
bool testRAK16002() {
  return true;
}
bool testRAK12037() {
  return true;
}
bool testRAK16001() {
  return true;
}
bool testRAK1905() {
  return true;
}
bool testRAK12025() {
  return true;
}
bool testRAK12040() {
  return true;
}
bool testRAK12034() {
  return true;
}
bool testRAK14008() {
  return true;
}

rakSensor knownProducts[] = {  {0x3c, "SSD1306", "rak1921", "OLED Display Solomon SSD1306", testRAK1921 },
  {0x18, "LIS3DH", "rak1904", "3-Axis Acceleration Sensor STM LIS3DH", testRAK1904 },
  {0x42, "ZOE-M8Q", "rak12500", "GNSS Location Module u-blox ZOE-M8Q", testRAK12500 },
  {0x76, "BME680", "rak1906", "Environment Sensor BOSCH BME680", testRAK1906 },
  {0x1D, "ADXL313", "rak12032", "ADXL313 accelerometer", testRAK12032 },
  {0x2C, "SCT31", "rak12008", "SCT31 CO2 gas sensor", testRAK12008 },
  {0x3A, "MLX90632", "rak12003", "IR temperature sensor", testRAK12003 },
  {0x04, "MCP23017", "rak14003", "LED bargraph module", testRAK14003 },
  {0x4A, "Water Level", "rak12059", "Water Level sensor", testRAK12059 },
  {0x5c, "LPS22HB", "rak1902", "barometric pressure sensor", testRAK1902 },
  {0x5F, "Keypad", "rak14004", "Keypad interface", testRAK14004 },
  {0x10, "VEML7700", "rak12010", "light sensor", testRAK12010 },
  {0x12, "PMSA003I", "rak12039", "PMSA003I particle matter sensor", testRAK12039 },
  {0x20, "ATTINY441", "rak12035", "soil moisture sensor [x] RAK13003", testRAK12035 },
  {0x20, "IO expander", "rak13003", "IO expander module [x] RAK12035", testRAK13003 },
  {0x28, "CAP1293", "rak14002", "Touch Button module", testRAK14002 },
  {0x29, "VL53L0X", "rak12014", "Laser ToF sensor", testRAK12014 },
  {0x33, "MLX90640", "rak12052", "MLX90640 IR array sensor", testRAK12052 },
  {0x38, "RGB LED", "rak14001", "RGB LED module", testRAK14001 },
  {0x41, "INA219BID", "rak16000", "DC current sensor", testRAK16000 },
  {0x44, "OPT3001", "rak1903", "light sensor", testRAK1903 },
  {0x47, "PWM expander", "rak13004", "PWM expander module", testRAK13004 },
  {0x50, "EEPROM", "rak15000", "EEPROM [x] RAK12008", testRAK15000 },
  {0x51, "MQ2 sensor", "rak12004", "MQ2 CO2 gas sensor [x] RAK15000", testRAK12004 },
  {0x52, "RTC", "rak12002", "RTC module [x] RAK15000", testRAK12002 },
  {0x53, "LTR390", "rak12019", "LTR390 light sensor [x] RAK15000", testRAK12019 },
  {0x54, "Flex sensor", "rak12016", "Flex sensor", testRAK12016 },
  {0x55, "MQ3 sensor", "rak12009", "MQ3 Alcohol gas sensor", testRAK12009 },
  {0x55, "Omron D7S", "rak12027", "D7S seismic sensor", testRAK12027 },
  {0x57, "MAX30102", "rak12012", "MAX30102 heart rate sensor", testRAK12012 },
  {0x59, "ACC608", "rak5814", "ACC608 encryption module [x] RAK12047, RAK13600, RAK13003", testRAK5814 },
  {0x59, "SGP40", "rak12047", "VOC sensor [x] RAK13600, RAK13003, RAK5814", testRAK12047 },
  {0x59, "NFC", "rak13600", "NFC [x] RAK12047, RAK13600, RAK5814", testRAK13600 },
  {0x59, "LTC2941", "rak16002", "Coulomb sensor [x] RAK13600, RAK12047, RAK5814", testRAK16002 },
  {0x61, "SCD30", "rak12037", "CO2 sensor [x] RAK16001", testRAK12037 },
  {0x61, "ADC sensor", "rak16001", "ADC sensor [x] RAK12037", testRAK16001 },
  {0x68, "MPU9250", "rak1905", "MPU9250 9DOF sensor [x] RAK12025", testRAK1905 },
  {0x68, "Gyroscope", "rak12025", "Gyroscope address [x] RAK1905", testRAK12025 },
  {0x68, "AMG8833", "rak12040", "AMG8833 temperature array sensor", testRAK12040 },
  {0x69, "BMX160", "rak12034", "BMX160 9DOF sensor", testRAK12034 },
  {0x70, "SHTC3", "rak1901", "temperature & humidity sensor", testRAK1901 },
  {0x73, "PAJ7620U2", "rak14008", "Gesture sensor", testRAK14008 },
  {0x5d, "LPS33", "rak12011", "Waterproof barometric pressure sensor", testRAK12011 },
};
uint8_t numProducts = sizeof(knownProducts) / sizeof(rakSensor);
/*
   sensors_t found_sensors[] = {
  //  I2C address , I2C bus, found?
  {0x18, false}, //   0 ✔ rak1904 accelerometer
  {0x44, false}, //   1 ✔ rak1903 light sensor
  {0x42, false}, //   2 ✔ rak12500 GNSS sensor
  {0x5c, false}, //   3 ✔ rak1902 barometric pressure sensor
  {0x70, false}, //   4 ✔ rak1901 temperature & humidity sensor
  {0x76, false}, //   5 ✔ rak1906 environment sensor
  {0x20, false}, //   6 ✔ rak12035 soil moisture sensor !! address conflict with rak13003
  {0x10, false}, //   7 ✔ rak12010 light sensor
  {0x51, false}, //   8 ✔ rak12004 MQ2 CO2 gas sensor !! conflict with rak15000
  {0x50, false}, //   9 ✔ rak15000 EEPROM !! conflict with rak12008
  {0x2C, false}, //  10 ✔ rak12008 SCT31 CO2 gas sensor
  {0x55, false}, //  11 ✔ rak12009 MQ3 Alcohol gas sensor
  {0x29, false}, //  12 ✔ rak12014 Laser ToF sensor
  {0x52, false}, //  13 ✔ rak12002 RTC module !! conflict with rak15000
  {0x04, false}, //  14 ✔ rak14003 LED bargraph module
  {0x59, false}, //  15 ✔ rak12047 VOC sensor !! conflict with rak13600, rak13003, RAK5814
  {0x68, false}, //  16 ✔ rak12025 Gyroscope address !! conflict with rak1905
  {0x73, false}, //  17 ✔ rak14008 Gesture sensor
  {0x3C, false}, //  18 ✔ rak1921 OLED display
  {0x53, false}, //  19 ✔ rak12019 LTR390 light sensor !! conflict with rak15000
  {0x28, false}, //  20 ✔ rak14002 Touch Button module
  {0x41, false}, //  21 ✔ rak16000 DC current sensor
  {0x68, false}, //  22 ✔ rak1905 MPU9250 9DOF sensor !! conflict with rak12025
  {0x61, false}, //  23 ✔ rak12037 CO2 sensor !! conflict with rak16001
  {0x3A, false}, //  24 ✔ rak12003 IR temperature sensor
  {0x68, false}, //  25 ✔ rak12040 AMG8833 temperature array sensor
  {0x69, false}, //  26 ✔ rak12034 BMX160 9DOF sensor
  {0x1D, false}, //  27 ✔ rak12032 ADXL313 accelerometer
  {0x12, false}, //  28 ✔ rak12039 PMSA003I particle matter sensor
  {0x55, false}, //  29 ✔ rak12027 D7S seismic sensor
  {0x4A, false}, //  30 ✔ rak12059 Water Level sensor
  {0x57, false}, //  31 rak12012 MAX30102 heart rate sensor
  {0x54, false}, //  32 rak12016 Flex sensor
  {0x47, false}, //  33 rak13004 PWM expander module
  {0x38, false}, //  34 rak14001 RGB LED module
  {0x5F, false}, //  35 rak14004 Keypad interface
  {0x61, false}, //  36 rak16001 ADC sensor !! conflict with rak12037
  {0x59, false}, //  37 rak13600 NFC !! conflict with rak12047, rak13600, RAK5814
  {0x59, false}, //  38 rak16002 Coulomb sensor !! conflict with rak13600, rak12047, RAK5814
  {0x20, false}, //  39 rak13003 IO expander module !! conflict with rak12035
  {0x59, false}, //  40 ✔ RAK5814 ACC608 encryption module (limited I2C speed 100000) !! conflict with rak12047, rak13600, rak13003
  {0x33, false}, //  41 ✔ rak12052 MLX90640 IR array sensor
  };
*/

void hexDump(uint8_t* buf, uint16_t len, uint8_t zero) {
  //  Something similar to the Unix/Linux hexdump -C command
  //  Pretty-prints the contents of a buffer, 16 bytes a row
  char alphabet[17] = "0123456789abcdef";
  uint16_t i, index;
  Serial.print(F("   +------------------------------------------------+ +----------------+\n"));
  Serial.print(F("   |.0 .1 .2 .3 .4 .5 .6 .7 .8 .9 .a .b .c .d .e .f | |      ASCII     |\n"));
  for (i = 0; i < len; i += 16) {
    if (i % 128 == 0) Serial.print(F("   +------------------------------------------------+ +----------------+\n"));
    char s[] = "|                                                | |                |\n";
    //  pre-formated line. We will replace the spaces with text when appropriate.
    uint8_t ix = 1, iy = 52, j;
    for (j = 0; j < 16; j++) {
      if (i + j < len) {
        uint8_t c = buf[i + j];
        //  fastest way to convert a byte to its 2-digit hex equivalent
        if (zero == 0) {
          s[ix++] = alphabet[(c >> 4) & 0x0F];
          s[ix++] = alphabet[c & 0x0F];
          ix++;
        } else {
          if (c == 0) {
            s[ix++] = zero;
            s[ix++] = zero;
          } else {
            s[ix++] = alphabet[(c >> 4) & 0x0F];
            s[ix++] = alphabet[c & 0x0F];
          }
          ix++;
        }
        if (c > 31 && c < 128) s[iy++] = c;
        else s[iy++] = '.'; //  display ASCII code 0x20-0x7F or a dot.
      }
    }
    index = i / 16;
    //  display line number then the text
    if (i < 256) Serial.write(' ');
    Serial.print(index, HEX); Serial.write('.');
    Serial.print(s);
  }
  Serial.print(F("   +------------------------------------------------+ +----------------+\n"));
}

void refreshScreen() {
  u8g2.clearBuffer(); //  clear the internal memory
  u8g2.setFont(u8g2_font_profont11_tf);
  uint8_t py = 0;
  for (uint8_t i = 0; i < myBuff.size(); i++) {
    u8g2.setCursor(0, py);
    u8g2.print(myBuff.at(i).c_str());
    py += 12;
  }
  u8g2.sendBuffer(); //  transfer internal memory to the display
}

void addStringLine(char* lineOfText, byte refresh = 1) {
  myBuff.push_back(lineOfText);
  while (myBuff.size() > 5) myBuff.erase(myBuff.begin() + 1);
  if (refresh) refreshScreen();
}

void subStringLine(char* lineOfText, byte refresh = 1) {
  myBuff.erase(myBuff.end());
  myBuff.push_back(lineOfText);
  if (refresh) refreshScreen();
}


//  EOF
