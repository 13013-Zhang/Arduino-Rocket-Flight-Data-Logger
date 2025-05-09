/*
MPU6050: (I2C)
VCC  -> 3.3V
SCL  -> A5
SDA  -> A4
AD0  -> GND

BME280: (SPI)
VCC  -> 3.3V
SCK  -> D13
MISO -> D12
MOSI -> D11
CS   -> D9

SD Card: (SPI)
VCC  -> 5V
CS   -> D10
MOSI -> D11
MISO -> D12
SCK  -> D13

Servo:
PWM  -> D3
VCC  -> 5V
GND  -> GND

Buzzer:
VCC  -> D2
GND  -> GND 

LM35:
OUT -> A0
VCC -> 5V
GND -> GND
*/

#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU6050_light.h>

// Pins
#define BME280_CS 9
#define SD_CS 10
#define BUZZER_PIN 2
#define SERVO_PIN 3
#define LM35_PIN A0
#define LED 5

// Sensors
Adafruit_BME280 bme(BME280_CS);
MPU6050 mpu(Wire);
Servo parachuteServo;
SdFat SD;
File dataFile;

// Calibration and state
float basePressure = 101325.0;
float seaLevelPressure = 1013.25;
float kalmanAngleX = 0, kalmanAngleY = 0;
float kalmanAltitude = 0;
float velocity = 0;

float angleErrorCov = 1, altitudeErrorCov = 1;
float angleEstimateCov = 1, altitudeEstimateCov = 1;

unsigned long lastKalmanTime = 0;
float altitudeVelocity = 0;

struct {
  float baseAlt;
  float initialRoll;
  float initialPitch;
  float initialYaw;
} calData;

struct {
  bool isFlying = false;
  bool parachuteDeployed = false;
  bool landed = false;
  unsigned long startTime = 0;
  unsigned int landingChecks = 0;
  const unsigned int requiredLandingChecks = 3;
} flightState;

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

// Kalman filter for angle (1D)
float kalmanFilterAngle(float newAngle, float gyroRate, float dt) {
  float Q_angle = 0.001, Q_gyro = 0.003, R_angle = 0.03;
  static float bias = 0;
  static float rate = 0;
  static float P[2][2] = { { 0, 0 }, { 0, 0 } };
  static float angle = 0;

  rate = gyroRate - bias;
  angle += dt * rate;

  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_gyro * dt;

  float S = P[0][0] + R_angle;
  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  float y = newAngle - angle;
  angle += K[0] * y;
  bias += K[1] * y;

  float P00_temp = P[0][0], P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return angle;
}

// BMP altitude via pressure (ISA standard)
float pressureToAltitude(float pressure) {
  return 44330.0 * (1.0 - pow(pressure / (seaLevelPressure * 100), 0.1903));
}

void initMPU6050() {
  Wire.begin();
  byte status = mpu.begin();
  while (status != 0) {
    Serial.println("MPU6050 Connection failed, Error code: " + String(status));
    delay(1000);
    status = mpu.begin();
  }
  Serial.println("MPU6050 Connection successful");
  delay(1000);
  mpu.calcOffsets(true, true);
  mpu.update();
  calData.initialRoll = mpu.getAngleX();
  calData.initialPitch = mpu.getAngleY();
  calData.initialYaw = mpu.getAngleZ();
}

void initBME280() {
  if (!bme.begin()) {
    Serial.println("BME280 initialization failed!");
    while (1)
      ;
  }
  float sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += pressureToAltitude(bme.readPressure());
    delay(10);
  }
  calData.baseAlt = sum / 100;
}

bool initSDCard() {
  if (!SD.begin(SD_CS, SPI_FULL_SPEED)) return false;
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

void readSensors() {
  unsigned long now = millis();
  float dt = (now - lastKalmanTime) / 1000.0;
  if (dt <= 0) dt = 0.01;
  lastKalmanTime = now;

  mpu.update();
  float angleXraw = mpu.getAngleX() - calData.initialRoll;
  float angleYraw = mpu.getAngleY() - calData.initialPitch;

  kalmanAngleX = kalmanFilterAngle(angleXraw, mpu.getGyroX(), dt);
  kalmanAngleY = kalmanFilterAngle(angleYraw, mpu.getGyroY(), dt);

  sensorData.accelX = mpu.getAccX();
  sensorData.accelY = mpu.getAccY();
  sensorData.accelZ = mpu.getAccZ();
  sensorData.gyroX = mpu.getGyroX();
  sensorData.gyroY = mpu.getGyroY();
  sensorData.gyroZ = mpu.getGyroZ();
  sensorData.angleX = kalmanAngleX;
  sensorData.angleY = kalmanAngleY;
  sensorData.angleZ = mpu.getAngleZ() - calData.initialYaw;

  sensorData.pressure = bme.readPressure();
  float currentAlt = pressureToAltitude(sensorData.pressure) - calData.baseAlt;

  float Q = 0.01, R = 2;
  float P = altitudeEstimateCov + Q;
  float K = P / (P + R);
  kalmanAltitude += K * (currentAlt - kalmanAltitude);
  altitudeEstimateCov = (1 - K) * P;

  sensorData.altitude = currentAlt;
  sensorData.filteredAltitude = kalmanAltitude;
  sensorData.temperature = bme.readTemperature();
  sensorData.lm35Temp = analogRead(LM35_PIN) * (5.0 / 1024.0) * 100;
  sensorData.time = now - flightState.startTime;
}

void writeDataToSD() {
  if (!dataFile) return;
  dataFile.print(sensorData.time);
  dataFile.print(",");
  dataFile.print(sensorData.temperature);
  dataFile.print(",");
  dataFile.print(sensorData.pressure / 100.0);
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

void checkFlightStatus() {
  static float maxAlt = 0;
  static bool descending = false;
  static int angleChecks = 0;
  static float prevAlt = 0;
  static unsigned long lastCheckTime = 0;

  if (!flightState.isFlying && (abs(sensorData.accelZ) > 1.5 || abs(sensorData.accelY) > 1.5 || abs(sensorData.accelX) > 1.5)) {
    flightState.isFlying = true;
    dataFile.println("take off");
  }

  if (flightState.isFlying && !flightState.landed) {
    if (sensorData.filteredAltitude > maxAlt) {
      maxAlt = sensorData.filteredAltitude;
      descending = false;
    } else if (sensorData.filteredAltitude < maxAlt - 2) {
      descending = true;
    }

    float tilt = sqrt(sensorData.angleX * sensorData.angleX + sensorData.angleY * sensorData.angleY);
    angleChecks = (tilt > 120) ? angleChecks + 1 : 0;

    if (!flightState.parachuteDeployed && (descending && sensorData.filteredAltitude < 0.9 * maxAlt || angleChecks >= 3)) {
      parachuteServo.write(90);
      flightState.parachuteDeployed = true;
      dataFile.println("Parachute opened");
    }

    unsigned long t = millis();
    float dt = (t - lastCheckTime) / 1000.0;
    if (dt > 0.1) {
      float altChangeRate = (sensorData.filteredAltitude - prevAlt) / dt;
      if (abs(altChangeRate) < 0.5 && abs(sensorData.accelZ - 1.0) < 0.3) {
        flightState.landingChecks++;
      } else {
        flightState.landingChecks = 0;
      }
      prevAlt = sensorData.filteredAltitude;
      lastCheckTime = t;
    }

    if (flightState.landingChecks >= flightState.requiredLandingChecks && flightState.parachuteDeployed) {
      flightState.landed = true;
      digitalWrite(BUZZER_PIN, HIGH);
      dataFile.println("Landing Complete");
    }
  }

  if (flightState.landed) {
    static unsigned long lastBeep = 0;
    if (millis() - lastBeep >= 100) {
      digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
      lastBeep = millis();
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  parachuteServo.attach(SERVO_PIN);
  parachuteServo.write(60);
  initMPU6050();
  initBME280();
  if (!initSDCard()) Serial.println("SD card initialization failed, data is only output to the serial port");
  digitalWrite(LED, HIGH);
  flightState.startTime = millis();
  lastKalmanTime = millis();
  Serial.println("System startup completed");
}

void loop() {
  static unsigned long lastTime = 0;
  if (millis() - lastTime >= 10) {
    readSensors();
    writeDataToSD();
    checkFlightStatus();
    lastTime = millis();
  }
}
