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
Kalman kalmanZ;
double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter

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


  float Yh = (IMU.getMagY_uT() * cos(roll*DEG_TO_RAD)) - (IMU.getMagZ_uT() * sin(roll*DEG_TO_RAD));
  float Xh = (IMU.getMagX_uT() * cos(pitch*DEG_TO_RAD))+(IMU.getMagY_uT() * sin(roll*DEG_TO_RAD)*sin(pitch*DEG_TO_RAD)) + (IMU.getMagZ_uT() * cos(roll*DEG_TO_RAD) * sin(pitch*DEG_TO_RAD));


  float yaw =  atan2(Yh, Xh);

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  kalmanZ.setAngle(yaw);
  gyroXangle = roll;
  gyroYangle = pitch;
  gyroZangle = yaw;
  compAngleX = roll;
  compAngleY = pitch;
  compAngleZ = yaw;
  


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

uint16_t cont = 0;


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
  double gyroZrate = IMU.getGyroZ_rads() * RAD_TO_DEG; // Convert to deg/s



  //Pitch
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter


  

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading

  //Roll
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  gyroZangle += gyroZrate * dt;


  // Aplica filtro complementar:
  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  // Eixo Z (Yaw) fica no fim
  


  // Reseta ângulo do giroscópio quando ocorreram muitos drifts
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;


  // Normaliza leituras do magnetômetro
  float magNormXuT = IMU.getMagX_uT();
  float magNormYuT = IMU.getMagY_uT();
  float magNormZuT = IMU.getMagZ_uT();
  float mag_norm = sqrt( (magNormXuT*magNormXuT)+(magNormYuT*magNormYuT)+(magNormZuT*magNormZuT) );
  
  magNormXuT /= mag_norm;
  magNormYuT /= mag_norm;
  magNormZuT /= mag_norm;
  



  /*
   * Cálculos trazidos da fonte:
   * https://gist.github.com/shoebahmedadeel/0d8ca4eaa65664492cf1db2ab3a9e572
   * 
   * Com algumas adaptações, visto que no código atual é usada uma biblioteca que dá os valores convertidos
   * e calibrados.
   * 
   * Valores se mostram reais. Atente à calibragem do magnetômetro.
   */

  float kalRollRAD = kalAngleX * DEG_TO_RAD;
  float kalPitchRAD = kalAngleY * DEG_TO_RAD;


  //float Yh = (IMU.getMagY_uT() * cos(kalRollRAD)) - (IMU.getMagZ_uT() * sin(kalRollRAD));
  //float Xh = (IMU.getMagX_uT() * cos(kalPitchRAD))+(IMU.getMagY_uT() * sin(kalRollRAD)*sin(kalPitchRAD)) + (IMU.getMagZ_uT() * cos(kalRollRAD) * sin(kalPitchRAD));

  float Yh = (magNormYuT * cos(kalRollRAD)) - (magNormZuT * sin(kalRollRAD));
  float Xh = (magNormXuT * cos(kalPitchRAD))+(magNormYuT * sin(kalRollRAD)*sin(kalPitchRAD)) + (magNormZuT * cos(kalRollRAD) * sin(kalPitchRAD));


  float yaw =  atan2(Yh, Xh) * RAD_TO_DEG;
  //if(yaw < 0) yaw += 360;


  //Yaw
  kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt);

  // Reseta ângulo do giroscópio quando ocorreram muitos drifts
  if (gyroZangle < -180 || gyroZangle > 180)
  gyroZangle = kalAngleZ;

    

  //Filtro complementar Z
  compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;



  if(cont++ % 30 == 29){ // X a intervalo de 3ms equivalente a 3x ms
    Serial.printf("%.1f\t%.1f\t%.1f\n",kalAngleX,kalAngleY,kalAngleZ); //ROLL, PITCH e YAW
    cont = 0;
  }
  
  delay(3);


}
