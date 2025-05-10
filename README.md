**Read this in other languages: [English](README.md), [中文](README_ZH.md).**

# 🚀 Arduino-Based Water Rocket Flight Data Logger

An Arduino-powered, high-precision water rocket system designed to collect and log complete flight data from launch to landing. The system uses sensor fusion and filtering algorithms, features automatic parachute deployment, and includes an audio signal upon landing. Compatible with the real-time data visualization platform.

## 🧠 Features

- Sensor fusion using **MPU6050 (I2C)** and **BME280 (SPI)** for flight dynamics and environment data
- **Kalman filter** for angle estimation and **BMP (barometric method)** for altitude correction
- **Millisecond-interval data logging** to SD card with uniform timestamps for detailed analysis
- Real-time data visualization support
- **Automatic parachute deployment** at apogee or when the rocket tilts more than 120° (confirmed 3 times to avoid false triggers)
- **Buzzer alert** activated upon landing to assist in rocket retrieval

## 🛠️ Hardware Wiring
```
MPU6050: (I2C)
VCC      -> 3.3V
SCL      -> A5
SDA      -> A4
AD0      -> GND

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
+    -> D2
GND  -> GND 

LM35:
OUT -> A0
VCC -> 5V
GND -> GND
```

## 📦 Software Capabilities

- ✅ Real-time logging of: orientation angles (x, y, z), acceleration, pressure, altitude, temperature, and timestamp  
- ✅ Kalman filtering for stable angle output from MPU6050  
- ✅ BMP correction for accurate altitude using BME280  
- ✅ Apogee and tilt-based parachute logic (60° open, 90° closed)  
- ✅ Buzzer activation after landing (based on altitude and motion detection)  
- ✅ All data saved in CSV format on SD card  
- ✅ Compatible with Vofa+ desktop tool for visualization

## 📂 Data Format (Saved on SD Card)

Example (logged at consistent millisecond intervals):
| Time(ms) | Temp(C) | Pressure(hPa) | Alt(m) | FiltAlt(m) | AccX(g) | AccY(g) | AccZ(g) | GyroX(deg/s) | GyroY(deg/s) | GyroZ(deg/s) | AngleX(deg) | AngleY(deg) | AngleZ(deg) | LM35Temp(C) |
|----------|---------|---------------|--------|------------|---------|---------|---------|--------------|--------------|--------------|-------------|-------------|-------------|-------------|
| 0        | 21.94   | 1013.11       | 0.55   | 0.11       | 0       | -0.04   | 0.98    | 1.84         | -2.7         | 2.32         | 2.67        | -4.63       | 3.8         | 21.48       |



## ✅ Development & Compatibility

- IDE: Arduino IDE  
- All libraries can be installed via Library Manager  
- Supported Boards: Arduino UNO / Nano / Mega etc.  
- Upper Computer: Vofa+  

## 📸 Images and Demonstrations
![Schematic](../Schematic_Rocket.png)
<!-- > (Insert photos of your system, wiring diagram, or Vofa+ plots here) -->

---

## 📄 License

This project is open-sourced under the MIT License. Feel free to use and modify it. Please provide attribution for commercial use.

---

## 🙌 Acknowledgments

Special thanks to the open-source community and hardware developers who made this project possible.
