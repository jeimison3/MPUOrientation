# MPU Compass IMU Calculations (ESP32 and ESP8266 based)

## Differences
The Arduino IDE with Espressif native port for ESP32 has conflicted with `Wire` library.
So I met the core of [stickbreaker (click here)](https://github.com/stickbreaker/arduino-esp32/tree/patch-2) so currently is not necessary any change in the code between ESP32 and ESP8266. 

## Description about folders
| Project | Functionality |
| --- | --- |
| ESP8266 | A compatible code with ESP8266 accompanying calculation from axis `Pitch`, `Roll` and `Yaw` using magnetometer for orientation. |
| ESP32 | A compatible code with ESP32 accompanying calculation from axis `Pitch`, `Roll` and `Yaw` using magnetometer for orientation. |
| ESP32-Kalman-NO-MAG | Compatible code with ESP32 with Kalman Filter and Complementary Filter applied to calculate the axis `Pitch` and `Roll`. |
| ESP32-Kalman-Compass | Compatible code with ESP32 applying Kalman Filter and Complementary Filter to calculate the axis `Pitch`, `Roll` and `Yaw` |

## Libraries
- **Kalman Filter** [`Download here`](https://github.com/TKJElectronics/KalmanFilter). Use a library simplifies the implementation.
- **MPU9250** [`Download here`](https://github.com/jeimison3/MPU9250). That's a fork from original library from [bolderflight](https://github.com/bolderflight/MPU9250) but with some implementations that I made for ESP8266/ESP32 in the "begin" function.
