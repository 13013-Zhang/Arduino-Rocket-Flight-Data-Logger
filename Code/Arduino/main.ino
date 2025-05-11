#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU6050_light.h>

#define BME280_CS 9
#define SD_CS 10
#define BUZZER_PIN 2
#define SERVO_PIN 3
#define LM35_PIN A0
#define LED 5

Adafruit_BME280 bme(BME280_CS);
MPU6050 mpu(Wire);
Servo parachuteServo;

File dataFile;

SdFat SD;

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

void initMPU6050() {
  Wire.begin();

  byte status = mpu.begin();
  while (status != 0) {
    Serial.println("MPU6050 connection failed, error code: " + String(status));
    delay(1000);
    status = mpu.begin();
  }
  Serial.println("MPU6050 connected successfully");

  Serial.println("Calibrating MPU6050... Please keep the device still");
  delay(1000);
  mpu.calcOffsets(true, true);

  mpu.update();
  calData.initialRoll = mpu.getAngleX();
  calData.initialPitch = mpu.getAngleY();
  calData.initialYaw = mpu.getAngleZ();

  Serial.println("Calibration complete, initial attitude (degrees):");
  Serial.print("Roll: ");
  Serial.print(calData.initialRoll);
  Serial.print(", Pitch: ");
  Serial.print(calData.initialPitch);
  Serial.print(", Yaw: ");
  Serial.println(calData.initialYaw);
}

void initBME280() {
  if (!bme.begin()) {
    Serial.println("BME280 initialization failed, please check wiring!");
    while (1)
      ;
  }

  Serial.println("Calibrating BME280 base altitude...");
  float sumAlt = 0;
  for (int i = 0; i < 100; i++) {
    sumAlt += bme.readAltitude(1013.25);
    delay(10);
  }
  calData.baseAlt = sumAlt / 100;
  Serial.println("Base altitude calibration complete: " + String(calData.baseAlt) + " meters");
}

bool initSDCard() {
  if (!SD.begin(SD_CS, SPI_FULL_SPEED)) {
    Serial.println("SD card initialization failed!");
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

  sensorData.lm35Temp = analogRead(LM35_PIN) * (5.0 / 1024.0) * 100;
}

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

void checkFlightStatus() {
  static float maxAltitude = 0;
  static bool descending = false;
  static int angleChecks = 0;
  static float prevAlt = 0;
  static unsigned long lastCheckTime = 0;

  if (!flightState.isFlying && (abs(sensorData.accelZ) > 1.5 || abs(sensorData.accelY) > 1.5 || abs(sensorData.accelX) > 1.5)) {
    flightState.isFlying = true;
    dataFile.println("Liftoff detected!");
    dataFile.flush();
    Serial.println("Liftoff detected!");
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
      dataFile.println("Parachute deployed!");
      dataFile.flush();
      Serial.println("Parachute deployed!");
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
      dataFile.println("Landing----------------------------");
      dataFile.flush();
      Serial.println("Landing detected!");
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

  pinMode(BUZZER_PIN, OUTPUT);
  parachuteServo.attach(SERVO_PIN);
  parachuteServo.write(60);

  initMPU6050();
  initBME280();

  while (!initSDCard()) {
    Serial.println("SD card initialization failed, data will only be output to serial");
  }

  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);

  flightState.startTime = millis();
  Serial.println("System initialization complete, ready");
}

void loop() {
  static unsigned long lastLogTime = 0;

  if (millis() - lastLogTime >= 10) {
    readSensors();
    writeDataToSD();
    sendDataToSerial();
    checkFlightStatus();
    lastLogTime = millis();
  }
}