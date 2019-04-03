# MPUOrientation - Compass with IMU calculations

## Approach
Not every sensor comes with a library which include orientation calculation.
This library is compatible with C and C++ compilers and solve the calculations by applying the Kalman filter.

### ESP32 native Core problem with I2C
The Arduino IDE with Espressif native port for ESP32 has conflicted with `Wire` library.
So I met the core of [stickbreaker (click here)](https://github.com/stickbreaker/arduino-esp32/tree/patch-2) so currently is not necessary any change in the code between ESP32 and ESP8266.
The code pressupose a valid and calibrated value to sensors input.


## Using
### Including LIB
```C
#include <MPUOrientation.h>
```

### Creating contexts
Contexts are useful in recursive operations (like in Kalman filter). So we need to use one for each MPU.
```C
pCompassContext CNTX = createContext();
```

### Calculating time between recall (Arduino)
```C
//Declare global variable:
uint32_t timer;

...

// Inside loop(), when need to use delta time "dt"
double dt = (double)(micros() - timer) / 1000000; // Calculate as second
timer = micros(); // Reset time

```

### Calculating orientation as IMUFullFusion
```C

IMUFullFusion SENS;
SENS.ACCEL.x = IMU.getAccelX_mss();
SENS.ACCEL.y = IMU.getAccelY_mss();
SENS.ACCEL.z = IMU.getAccelZ_mss();

SENS.GYRO.x = IMU.getGyroX_rads();
SENS.GYRO.y = IMU.getGyroY_rads();
SENS.GYRO.z = IMU.getGyroZ_rads();

SENS.MAG.x = IMU.getMagX_uT();
SENS.MAG.y = IMU.getMagY_uT();
SENS.MAG.z = IMU.getMagZ_uT();


// Context, ALGO, IMUFullFusion and time variance in seconds
IMUOrientation ori = getFullOrientation(CNTX, ALGO_KALMAN_V1, SENS, dt);

```

### Or calculating orientation as IMUFusion (no Mag)
```C

IMUFusion SENS;
SENS.ACCEL.x = IMU.getAccelX_mss();
SENS.ACCEL.y = IMU.getAccelY_mss();
SENS.ACCEL.z = IMU.getAccelZ_mss();

SENS.GYRO.x = IMU.getGyroX_rads();
SENS.GYRO.y = IMU.getGyroY_rads();
SENS.GYRO.z = IMU.getGyroZ_rads();


// Context, ALGO, IMUFusion and time variance in seconds
IMUOrientation ori = getOrientation(CNTX, ALGO_KALMAN_V1, SENS, dt);

```

### Print orientation
```C
Serial.printf( "\t%f\t%f\t%f\n", ori.pitch, ori.roll, ori.yaw );
```

This and other examples you can find in examples folder.

## Some optimizations
We consider to port the [`libfixmath`](https://code.google.com/archive/p/libfixmath/) to reduce cycles of calculation. Maybe a #define will be included in next versions of `compassSet.h`.

## Description about examples
| Examble path | Functionality |
| --- | --- |
| ESP-NO-LIB | Calculate the axis `Pitch`, `Roll` and `Yaw` using magnetometer without the library. |
| ESP-Kalman-NO-MAG | Calculate the axis `Pitch` and `Roll` with the library. |
| ESP-Kalman-Compass | Calculate the axis `Pitch`, `Roll` and `Yaw` with the library. |

## Libraries
- **[`Kalman Filter`](https://github.com/TKJElectronics/KalmanFilter)**. Now is already ported.
- **MPU9250** [`Download here`](https://github.com/jeimison3/MPU9250). That's a fork from original library from [bolderflight](https://github.com/bolderflight/MPU9250) but with some implementations that I made for ESP8266/ESP32 in the "begin" function.
