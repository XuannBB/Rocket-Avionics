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
#include <SoftwareSerial.h>  // 这里只剩下 LoRa 还在用 SoftwareSerial

//—— LoRa（保持不变） ——
SoftwareSerial loraSerial(10, 11);

//—— 不再用 UART 方式读取 GPS — 删除: SoftwareSerial gpsSerial(14, 15); ——

TinyGPSPlus gps;

//—— I2C GPS 地址（请根据你模块的文档自行修改） ——
#define I2C_GPS_ADDR  0x42

//—— 其他传感器初始化 ——
BMM150 bmm;
bmm150_mag_data value_offset;
Adafruit_MSA301 msa;
DFRobot_BMP388_I2C bmp388(&Wire, DFRobot_BMP388_I2C::eSDOGND);

const int chipSelect = 53;
const uint8_t ADDR_MSA301 = 0x62;
const uint8_t ADDR_BMP388 = 0x76;
const uint8_t ADDR_BMM150 = 0x13;

//—— 变量定义 ——
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

//—— I2C GPS 相关函数 ——
// 检测 I²C 设备是否存在
bool isI2CPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

// 初始化 SD 卡
bool initSD() {
  if (!SD.begin(chipSelect)) {
    return false;
  }
  return true;
}

// 用于磁力计校准的函数，与原来保持一致
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

  //—— I2C 初始化 ——//
  Wire.begin();  // 默认 SDA=A4, SCL=A5 (UNO/Nano)，或根据你的板子自行调整

  //—— 不再使用 UART GPS ——//
  // gpsSerial.begin(9600);  // 删除

  //—— LoRa 保持 UART 模式 —— //
  loraSerial.begin(57600);

  //—— 惯性+气压+磁力计 初始化 ——//
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

  //—— SD 卡 初始化 ——//
  while (!initSD()) {
    Serial.println("SD init failed, retrying...");
    delay(2000);
  }
  Serial.println("SD initialized");

  //—— 磁力计校准 ——//
  calibrate(10000);
  Serial.println("Magnetometer calibration done");
}

//—— 从 I2C GPS 模块读取数据，交给 TinyGPSPlus 解析 ——
// 假设 GPS 模块支持一次最多读 32 字节的 NMEA 串流
void readGPS_I2C() {
  // 先检测设备在不在
  if (!isI2CPresent(I2C_GPS_ADDR)) {
    // 如果 I2C GPS 不在线，可以选择直接 return
    return;
  }

  // 请求最多 32 个字节。根据你的模块手册调整长度
  Wire.requestFrom((uint8_t)I2C_GPS_ADDR, (uint8_t)32);
  while (Wire.available()) {
    char c = Wire.read();         // 读取一个字符
    gps.encode(c);                // 交给 TinyGPSPlus 逐字符解析
  }
}

//—— 主循环 ——//
void loop() {
  //—— SD 卡重新检测（防止拔插）——//
  if (!SD.begin(chipSelect)) {
    initSD();
  }

  //—— 读取三轴加速度 ——//
  if (isI2CPresent(ADDR_MSA301)) {
    sensors_event_t event;
    msa.getEvent(&event);
    ax = event.acceleration.x;
    ay = event.acceleration.y;
    az = event.acceleration.z;
  } else {
    ax = ay = az = 0;
  }

  //—— 读取气压/温度/高度 ——//
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

  //—— 读取磁力计，计算航向 ——//
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

  //—— 从 I2C GPS 读 NMEA 数据，给 TinyGPSPlus 解析 ——//
  readGPS_I2C();

  //—— 如果有新的 GPS 定位数据，就更新全局变量 ——//
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

  //—— 检查 SD 卡状态 ——//
  if (initSD()) {
    sdworking = 1;
  } else {
    sdworking = 0;
  }

  //—— 拼接一行数据 ——//
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

  //—— 写到 SD 卡 ——//
  File myFile = SD.open("test.txt", FILE_WRITE);
  if (myFile) {
    myFile.println(line);
    myFile.close();
  } else {
    Serial.println("Error opening test.txt");
  }

  //—— 通过 LoRa 发送 ——//
  loraSerial.println(line);
  Serial.println(line);

  delay(100);
}
