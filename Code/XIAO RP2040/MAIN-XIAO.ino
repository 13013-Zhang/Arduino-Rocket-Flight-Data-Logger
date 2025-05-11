/*
硬件连接定义 (基于XIAO RP2040引脚图):

MPU6050: (I2C)
VCC      -> 3.3V
SCL      -> D5 (P7 SCL)
SDA      -> D4 (P6 SDA)
AD0      -> GND

BME280: (SPI)
VCC  -> 3.3V
SCK  -> D8 (P2 SCK)
MISO -> D9 (P4 MISO)
MOSI -> D10 (P3 MOSI)
CS   -> D2 (P28 A2)

SD Card: (SPI)
VCC  -> 5V
CS   -> D7 (P1 CSn)
MOSI -> D10 (P3 MOSI)
MISO -> D9 (P4 MISO)
SCK  -> D8 (P2 SCK)

Servo:
PWM  -> D0 (P26 A0)
VCC  -> 5V
GND  -> GND

Buzzer:
+    -> D6 (P0 TX)
GND  -> GND 

LM35:
OUT -> A1 (P27 A1/D1)
VCC -> 5V
GND -> GND
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU6050_light.h>

// 引脚定义
#define BME280_CS D2   // P28 A2
#define SD_CS D7       // P1 CSn
#define BUZZER_PIN D6  // P0 TX
#define SERVO_PIN D0   // P26 A0
#define LM35_PIN A1    // P27 A1/D1

// 传感器对象
Adafruit_BME280 bme(BME280_CS);  // SPI
MPU6050 mpu(Wire);
Servo parachuteServo;

// 文件对象
File dataFile;

// 校准数据
struct {
  float baseAlt;
  float initialRoll;
  float initialPitch;
  float initialYaw;
} calData;

// 状态变量
struct {
  bool isFlying = false;
  bool parachuteDeployed = false;
  bool landed = false;
  unsigned long startTime = 0;
  unsigned int landingChecks = 0;
  const unsigned int requiredLandingChecks = 3;
} flightState;

// 传感器数据结构
struct {
  unsigned long time;
  float temperature;
  float pressure;
  float altitude;
  float filteredAltitude;
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float angleX, angleY, angleZ;
  float lm35Temp;
} sensorData;

// 初始化MPU6050
void initMPU6050() {
  Wire.setSDA(D4);  // XIAO RP2040的SDA是D4 (P6)
  Wire.setSCL(D5);  // XIAO RP2040的SCL是D5 (P7)
  Wire.begin();

  byte status = mpu.begin();
  while (status != 0) {
    Serial.println("MPU6050连接失败，错误代码: " + String(status));
    delay(1000);
    status = mpu.begin();
  }
  Serial.println("MPU6050连接成功");

  Serial.println("校准MPU6050...请保持设备静止");
  delay(1000);
  mpu.calcOffsets(true, true);

  mpu.update();
  calData.initialRoll = mpu.getAngleX();
  calData.initialPitch = mpu.getAngleY();
  calData.initialYaw = mpu.getAngleZ();

  Serial.println("校准完成，初始姿态(度):");
  Serial.print("Roll: ");
  Serial.print(calData.initialRoll);
  Serial.print(", Pitch: ");
  Serial.print(calData.initialPitch);
  Serial.print(", Yaw: ");
  Serial.println(calData.initialYaw);
}

// 初始化BME280
void initBME280() {
  SPI.setRX(D9);   // MISO (P4)
  SPI.setTX(D10);  // MOSI (P3)
  SPI.setSCK(D8);  // SCK (P2)

  if (!bme.begin()) {
    Serial.println("BME280初始化失败，请检查接线!");
    while (1)
      ;
  }

  Serial.println("校准BME280基准高度...");
  float sumAlt = 0;
  for (int i = 0; i < 100; i++) {
    sumAlt += bme.readAltitude(1013.25);
    delay(10);
  }
  calData.baseAlt = sumAlt / 100;
  Serial.println("基准高度校准完成: " + String(calData.baseAlt) + "米");
}

// 初始化SD卡
bool initSDCard() {
  SPI.setRX(D9);   // MISO (P4)
  SPI.setTX(D10);  // MOSI (P3)
  SPI.setSCK(D8);  // SCK (P2)

  if (!SD.begin(SD_CS)) {
    Serial.println("SD卡初始化失败!");
    return false;
  }

  char filename[15];
  for (int i = 1; i < 1000; i++) {
    sprintf(filename, "flight%d.csv", i);
    if (!SD.exists(filename)) {
      dataFile = SD.open(filename, FILE_WRITE);
      if (dataFile) {
        dataFile.println("Time(ms),Temp(C),Pressure(hPa),Alt(m),FiltAlt(m),AccX(g),AccY(g),AccZ(g),GyroX(deg/s),GyroY(deg/s),GyroZ(deg/s),AngleX(deg),AngleY(deg),AngleZ(deg),LM35Temp(C)");
        dataFile.flush();
        return true;
      }
    }
  }
  return false;
}

// 读取传感器数据
void readSensors() {
  static float prevAlt = 0;
  static unsigned long lastAltTime = 0;

  sensorData.time = millis() - flightState.startTime;

  mpu.update();
  sensorData.accelX = mpu.getAccX();
  sensorData.accelY = mpu.getAccY();
  sensorData.accelZ = mpu.getAccZ();
  sensorData.gyroX = mpu.getGyroX();
  sensorData.gyroY = mpu.getGyroY();
  sensorData.gyroZ = mpu.getGyroZ();
  sensorData.angleX = mpu.getAngleX() - calData.initialRoll;
  sensorData.angleY = mpu.getAngleY() - calData.initialPitch;
  sensorData.angleZ = mpu.getAngleZ() - calData.initialYaw;

  sensorData.temperature = bme.readTemperature();
  sensorData.pressure = bme.readPressure() / 100.0;

  float currentAlt = bme.readAltitude(1013.25) - calData.baseAlt;
  unsigned long currentTime = millis();
  float dt = (currentTime - lastAltTime) / 1000.0;

  if (dt > 0) {
    sensorData.filteredAltitude = 0.8 * prevAlt + 0.2 * currentAlt;
    prevAlt = sensorData.filteredAltitude;
    lastAltTime = currentTime;
  }

  sensorData.altitude = currentAlt;
  sensorData.lm35Temp = analogRead(LM35_PIN) * (3.3 / 4096.0) * 100;  // 12位ADC，3.3V参考
}

// 写入数据到SD卡
void writeDataToSD() {
  if (dataFile) {
    dataFile.print(sensorData.time);
    dataFile.print(",");
    dataFile.print(sensorData.temperature);
    dataFile.print(",");
    dataFile.print(sensorData.pressure);
    dataFile.print(",");
    dataFile.print(sensorData.altitude);
    dataFile.print(",");
    dataFile.print(sensorData.filteredAltitude);
    dataFile.print(",");
    dataFile.print(sensorData.accelX);
    dataFile.print(",");
    dataFile.print(sensorData.accelY);
    dataFile.print(",");
    dataFile.print(sensorData.accelZ);
    dataFile.print(",");
    dataFile.print(sensorData.gyroX);
    dataFile.print(",");
    dataFile.print(sensorData.gyroY);
    dataFile.print(",");
    dataFile.print(sensorData.gyroZ);
    dataFile.print(",");
    dataFile.print(sensorData.angleX);
    dataFile.print(",");
    dataFile.print(sensorData.angleY);
    dataFile.print(",");
    dataFile.print(sensorData.angleZ);
    dataFile.print(",");
    dataFile.println(sensorData.lm35Temp);
    dataFile.flush();
  }
}

// 发送数据到串口
void sendDataToSerial() {
  Serial.print(sensorData.time);
  Serial.print(",");
  Serial.print(sensorData.temperature);
  Serial.print(",");
  Serial.print(sensorData.pressure);
  Serial.print(",");
  Serial.print(sensorData.altitude);
  Serial.print(",");
  Serial.print(sensorData.filteredAltitude);
  Serial.print(",");
  Serial.print(sensorData.accelX);
  Serial.print(",");
  Serial.print(sensorData.accelY);
  Serial.print(",");
  Serial.print(sensorData.accelZ);
  Serial.print(",");
  Serial.print(sensorData.gyroX);
  Serial.print(",");
  Serial.print(sensorData.gyroY);
  Serial.print(",");
  Serial.print(sensorData.gyroZ);
  Serial.print(",");
  Serial.print(sensorData.angleX);
  Serial.print(",");
  Serial.print(sensorData.angleY);
  Serial.print(",");
  Serial.print(sensorData.angleZ);
  Serial.print(",");
  Serial.println(sensorData.lm35Temp);
}

// 检查飞行状态
void checkFlightStatus() {
  static float maxAltitude = 0;
  static bool descending = false;
  static int angleChecks = 0;
  static float prevAlt = 0;
  static unsigned long lastCheckTime = 0;

  if (!flightState.isFlying && (abs(sensorData.accelZ) > 1.5 || abs(sensorData.accelY) > 1.5 || abs(sensorData.accelX) > 1.5)) {
    flightState.isFlying = true;
    dataFile.println("检测到起飞!");
    dataFile.flush();
    Serial.println("检测到起飞!");
  }

  if (flightState.isFlying && !flightState.landed) {
    if (sensorData.filteredAltitude > maxAltitude) {
      maxAltitude = sensorData.filteredAltitude;
      descending = false;
    } else if (sensorData.filteredAltitude < maxAltitude - 2.0) {
      descending = true;
    }

    float tiltAngle = sqrt(sensorData.angleX * sensorData.angleX + sensorData.angleY * sensorData.angleY);
    if (tiltAngle > 120) {
      angleChecks++;
    } else {
      angleChecks = 0;
    }

    if (!flightState.parachuteDeployed && ((descending && sensorData.filteredAltitude < maxAltitude * 0.9) || angleChecks >= 3)) {
      parachuteServo.write(90);
      flightState.parachuteDeployed = true;
      dataFile.println("降落伞已打开!");
      dataFile.flush();
      Serial.println("降落伞已打开!");
    }

    unsigned long currentTime = millis();
    float dt = (currentTime - lastCheckTime) / 1000.0;
    if (dt > 0.1) {
      float altChangeRate = (sensorData.filteredAltitude - prevAlt) / dt;

      if (abs(altChangeRate) < 0.5 && abs(sensorData.accelZ - 1.0) < 0.3) {
        flightState.landingChecks++;
      } else {
        flightState.landingChecks = 0;
      }

      prevAlt = sensorData.filteredAltitude;
      lastCheckTime = currentTime;
    }

    if (flightState.landingChecks >= flightState.requiredLandingChecks && flightState.parachuteDeployed == true) {
      flightState.landed = true;
      digitalWrite(BUZZER_PIN, HIGH);
      dataFile.println("检测到着陆!");
      dataFile.flush();
      Serial.println("检测到着陆!");
    }
  }

  if (flightState.landed) {
    static unsigned long lastBeepTime = 0;
    if (millis() - lastBeepTime >= 100) {
      digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
      lastBeepTime = millis();
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;
  pinMode(23, INPUT);

  pinMode(BUZZER_PIN, OUTPUT);
  parachuteServo.attach(SERVO_PIN);
  parachuteServo.write(60);

  initMPU6050();
  initBME280();

  if (!initSDCard()) {
    Serial.println("SD卡初始化失败，数据将仅输出到串口");
  }

  flightState.startTime = millis();
  Serial.println("系统初始化完成，准备就绪");
}

void loop() {
  static unsigned long lastLogTime = 0;

  if (millis() - lastLogTime >= 10) {
    readSensors();
    writeDataToSD();
    // sendDataToSerial();
    checkFlightStatus();
    lastLogTime = millis();
  }
}