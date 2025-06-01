#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_BMP3XX.h>
#include <Adafruit_MSA301.h>
#include <Adafruit_Sensor.h>
#include "bmm150.h"
#include "bmm150_defs.h"
#include <TinyGPS++.h>
#include <SPI.h>
#include <SD.h>
#include "SdFat.h"
#include <SoftwareSerial.h>

SoftwareSerial loraSerial(10, 11);

TinyGPSPlus gps;

#define I2C_GPS_ADDR  0x42

BMM150 bmm;
bmm150_mag_data value_offset;
Adafruit_MSA301 msa;
DFRobot_BMP388_I2C bmp388(&Wire, DFRobot_BMP388_I2C::eSDOGND);

const int chipSelect = 53;
const uint8_t ADDR_MSA301 = 0x62;
const uint8_t ADDR_BMP388 = 0x76;
const uint8_t ADDR_BMM150 = 0x13;

float temperature = 0;
float pressure    = 0;
float altitude    = 0;
float ax = 0, ay = 0, az = 0;
float xyHeading     = 0;
float headingDegrees = 0;
float baselineAltitude = 0;
float gpsLat = 0;
float gpsLng = 0;
float gpsAlt = 0;
float gpsSpeed = 0;
int   gpsSats = 0;
int sdworking = 0;
int h;
int m;
int s;
bool  baselineSet     = false;

bool isI2CPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

bool initSD() {
  if (!SD.begin(chipSelect)) {
    return false;
  }
  return true;
}

void calibrate(uint32_t timeout) {
  int16_t x_min, x_max, y_min, y_max, z_min, z_max;
  uint32_t start = millis();

  bmm.read_mag_data();
  x_min = x_max = bmm.raw_mag_data.raw_datax;
  y_min = y_max = bmm.raw_mag_data.raw_datay;
  z_min = z_max = bmm.raw_mag_data.raw_dataz;

  while (millis() - start < timeout) {
    bmm.read_mag_data();
    int16_t rx = bmm.raw_mag_data.raw_datax;
    int16_t ry = bmm.raw_mag_data.raw_datay;
    int16_t rz = bmm.raw_mag_data.raw_dataz;

    x_min = min(x_min, rx);
    x_max = max(x_max, rx);
    y_min = min(y_min, ry);
    y_max = max(y_max, ry);
    z_min = min(z_min, rz);
    z_max = max(z_max, rz);

    Serial.print('.');
    delay(200);
  }

  value_offset.x = x_min + (x_max - x_min) / 2;
  value_offset.y = y_min + (y_max - y_min) / 2;
  value_offset.z = z_min + (z_max - z_min) / 2;
}

void setup() {
  Serial.begin(57600);

  Wire.begin(); 

  loraSerial.begin(57600);

  if (!msa.begin(ADDR_MSA301)) {
    Serial.println("MSA301 not found");
    while (1);
  }
  Serial.println("MSA301 initialized");

  while (bmp388.begin() != ERR_OK) {
    Serial.println("BMP388 not found");
    delay(3000);
  }
  bmp388.setSamplingMode(bmp388.eUltraPrecision);
  Serial.println("BMP388 initialized");

  if (bmm.initialize() == BMM150_E_ID_NOT_CONFORM) {
    Serial.println("BMM150 not found");
    while (1);
  }
  Serial.println("BMM150 initialized");

  while (!initSD()) {
    Serial.println("SD init failed, retrying...");
    delay(2000);
  }
  Serial.println("SD initialized");

  calibrate(10000);
  Serial.println("Magnetometer calibration done");
}
void readGPS_I2C() {

  
    if (!isI2CPresent(I2C_GPS_ADDR)) {
    return;
  }

  Wire.requestFrom((uint8_t)I2C_GPS_ADDR, (uint8_t)32);
  while (Wire.available()) {
    char c = Wire.read();     
    gps.encode(c);         
  }
}

void loop() {
  if (!SD.begin(chipSelect)) {
    initSD();
  }

  if (isI2CPresent(ADDR_MSA301)) {
    sensors_event_t event;
    msa.getEvent(&event);
    ax = event.acceleration.x;
    ay = event.acceleration.y;
    az = event.acceleration.z;
  } else {
    ax = ay = az = 0;
  }

  if (isI2CPresent(ADDR_BMP388)) {
    temperature = bmp388.readTempC();
    pressure    = bmp388.readPressPa();
    altitude    = bmp388.readAltitudeM();
    if (!baselineSet) {
      baselineAltitude = altitude;
      baselineSet = true;
      Serial.print("Baseline altitude: ");
      Serial.println(baselineAltitude);
    }
  } else {
    temperature = pressure = altitude = 0;
  }

  if (isI2CPresent(ADDR_BMM150)) {
    bmm.read_mag_data();
    float mx = bmm.raw_mag_data.raw_datax - value_offset.x;
    float my = bmm.raw_mag_data.raw_datay - value_offset.y;
    xyHeading = atan2(mx, my);
    if (xyHeading < 0) xyHeading += 2 * PI;
    headingDegrees = xyHeading * 180.0 / PI;
  } else {
    headingDegrees = xyHeading = 0;
  }

  readGPS_I2C();

  if (gps.location.isUpdated()) {
    gpsLat = gps.location.lat();
    gpsLng = gps.location.lng();
  }
  if (gps.altitude.isUpdated()) {
    gpsAlt = gps.altitude.meters();
  }
  if (gps.speed.isUpdated()) {
    gpsSpeed = gps.speed.kmph();
  }
  if (gps.satellites.isUpdated()) {
    gpsSats = gps.satellites.value();
  }

  if (gps.time.isUpdated()) {
    h = gps.time.hour();    // 24-hour UTC
    m = gps.time.minute();
    s = gps.time.second();
  }

  if (initSD()) {
    sdworking = 1;
  } else {
    sdworking = 0;
  }

  String line = String(temperature) + ","
    + headingDegrees + ","
    + (pressure / 100.0) + ","
    + (altitude - baselineAltitude) + ","
    + ax + "," + ay + "," + az + ","
    + sdworking + ","
    + gpsLat + "," 
    + gpsLng + ","
    + gpsAlt + ","
    + gpsSpeed + ","
    + gpsSats + ","
    + h + ","
    + m + ","
    + s;
  
  File myFile = SD.open("test.txt", FILE_WRITE);
  if (myFile) {
    myFile.println(line);
    myFile.close();
  } else {
    Serial.println("Error opening test.txt");
  }

  loraSerial.println(line);
  Serial.println(line);

  delay(100);
}
