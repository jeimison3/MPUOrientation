/*
 * Código de exemplo aplicado do algoritmo de Kalman
 * Modificado com cálculo de Pitch, Roll e Yaw por Jeimison Moreno (https://github.com/jeimison3).
 * 
 * ATENÇÃO: Usado core de ESP32 para Arduino IDE da fonte:
 * https://github.com/stickbreaker/arduino-esp32/tree/patch-2
 * 
 * Motivo: Biblioteca Wire nativa do core da espressif não funciona para ESP32.
 * A versão do stickbreaker corrige/reimplementa essa e outras bibliotecas.
 */

/*  Biblioteca pode ser baixada aqui: https://github.com/jeimison3/MPU9250
 *  É uma versão modificada para ESP12/ESP32 da biblioteca deste link: https://github.com/bolderflight/MPU9250
 */
 
#include "MPU9250.h"


/*
 * Uso da biblioteca KalmanFilter para filtragem das leituras usando o Filtro de Kalman:
 * LINK: https://github.com/TKJElectronics/KalmanFilter
 * Descrição completa do algoritmo e fontes podem ser encontradas no site do autor da biblioteca:
 * http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/
 */

#include "Kalman.h"
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;

#include <math.h> // Calculos complexos e uso da constante M_PI

#include <Wire.h>
#define i2cSDA 25
#define i2cSCL 33
TwoWire i2c;

MPU9250 IMU(i2c,0x69); // Conectada com ADO = 1
int statt;


void setup() {
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU 
  statt = IMU.begin(i2cSDA, i2cSCL); // Adaptação do código, pois estou usando uma NodeMCU 1.0
  Serial.print("Iniciando.");
  
  if (statt < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(statt);
    while(1) {}
  }

  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);

  /*IMU.calibrateAccel();
  IMU.calibrateGyro();
  IMU.calibrateMag();*/

  // Calibragem realizada para o ambiente:


  IMU.setAccelCalX(0.0, 1.0);
  IMU.setAccelCalY(0.0, 1.0);
  IMU.setAccelCalZ(0.0, 1.0);
  
  IMU.setGyroBiasX_rads(0.01);
  IMU.setGyroBiasY_rads(-0.03);
  IMU.setGyroBiasZ_rads(0.02);

  // Lembre-se de calibrar o magnetômetro para o seu local.
  
  IMU.setMagCalX(-13.70, 1.43);
  IMU.setMagCalY(-20.12, 1.56);
  IMU.setMagCalZ(-68.43, 0.60);



  delay(100); // Tempo para sensores estabilizarem


  

  double roll  = atan(IMU.getAccelY_mss() / sqrt(IMU.getAccelX_mss() * IMU.getAccelX_mss() + IMU.getAccelZ_mss() * IMU.getAccelZ_mss())) * RAD_TO_DEG;
  double pitch = atan2(-IMU.getAccelX_mss(), IMU.getAccelZ_mss()) * RAD_TO_DEG;

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}

void printBiases(){
  Serial.print("Accel:\t");
  Serial.print(IMU.getAccelBiasX_mss());
  Serial.print("\t");
  Serial.print(IMU.getAccelBiasY_mss());
  Serial.print("\t");
  Serial.println(IMU.getAccelBiasZ_mss());

  Serial.print("AcS:\t");
  Serial.print(IMU.getAccelScaleFactorZ());
  Serial.print("\t");
  Serial.print(IMU.getAccelScaleFactorY());
  Serial.print("\t");
  Serial.println(IMU.getAccelScaleFactorZ());
  

  Serial.print("Gyro:\t");
  Serial.print(IMU.getGyroBiasX_rads());
  Serial.print("\t");
  Serial.print(IMU.getGyroBiasY_rads());
  Serial.print("\t");
  Serial.println(IMU.getGyroBiasZ_rads());

  

  Serial.print("Mag:\t");
  Serial.print(IMU.getMagBiasX_uT());
  Serial.print("\t");
  Serial.print(IMU.getMagBiasY_uT());
  Serial.print("\t");
  Serial.println(IMU.getMagBiasZ_uT());

  Serial.print("MgS:\t");
  Serial.print(IMU.getMagScaleFactorX());
  Serial.print("\t");
  Serial.print(IMU.getMagScaleFactorY());
  Serial.print("\t");
  Serial.println(IMU.getMagScaleFactorZ());
  
  delay(10000);
}

uint32_t cont = 0;

void loop() {
  
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  
  if(Serial.available()){
    int v = Serial.parseInt();
    if(v == 1){
      printBiases();
    }
    else if (v == 10){
      IMU.calibrateMag();
    }
  }
  // read the sensor
  IMU.readSensor();

  
  double roll  = atan(IMU.getAccelY_mss() / sqrt(IMU.getAccelX_mss() * IMU.getAccelX_mss() + IMU.getAccelZ_mss() * IMU.getAccelZ_mss())) * RAD_TO_DEG;
  double pitch = atan2(-IMU.getAccelX_mss(), IMU.getAccelZ_mss()) * RAD_TO_DEG;

  double gyroXrate = IMU.getGyroX_rads() * RAD_TO_DEG; // Convert to deg/s
  double gyroYrate = IMU.getGyroY_rads() * RAD_TO_DEG; // Convert to deg/s

  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;


  // Aplica filtro complementar:

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;


  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;


  if(cont++ % 30 == 29){ // 125 a intervalo de 2ms equivalente a 0.25 seg
    Serial.print(roll); Serial.print("\t");
    Serial.print(gyroXangle); Serial.print("\t");
    Serial.print(compAngleX); Serial.print("\t");
    Serial.print(kalAngleX); Serial.print("\t");
  
    Serial.print("\t");
  
    Serial.print(pitch); Serial.print("\t");
    Serial.print(gyroYangle); Serial.print("\t");
    Serial.print(compAngleY); Serial.print("\t");
    Serial.print(kalAngleY); Serial.print("\t");
  
  
    Serial.print("\r\n");
    cont = 0;
  }

  
  delay(2);

}
