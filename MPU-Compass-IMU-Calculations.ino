#include "MPU9250.h" // Biblioteca/LIB: https://github.com/bolderflight/MPU9250
#include <math.h>


MPU9250 IMU(Wire,0x69); // Conectada com ADO = 1
int statt;


#include "Filter.h"


void setup() {
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU 
  statt = IMU.begin(D2, D3); // Adaptação do código, pois estou usando uma NodeMCU 1.0
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
  //IMU.setMagCalX(2.02, 1.52);
  //IMU.setMagCalY(34.76, 1.30);
  //IMU.setMagCalZ(-68.02, );


  IMU.setAccelCalX(0.0, 1.0);
  IMU.setAccelCalY(0.0, 1.0);
  IMU.setAccelCalZ(0.0, 1.0);
  
  IMU.setGyroBiasX_rads(0.01);
  IMU.setGyroBiasY_rads(-0.03);
  IMU.setGyroBiasZ_rads(0.02);
  
  IMU.setMagCalX(0.89, 1.55);
  IMU.setMagCalY(35.97, 1.19);
  IMU.setMagCalZ(-68.38, 0.66
);
  
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

void loop() {
  if(Serial.available()){
    int v = Serial.parseInt();
    if(v == 1){
      printBiases();
    }
  }
  // read the sensor
  IMU.readSensor();

  // display the data
  Serial.print(IMU.getAccelX_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelZ_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroX_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroY_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroZ_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagX_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagY_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagZ_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getTemperature_C(),6);
  
  filterUpdate(IMU.getAccelX_mss(), IMU.getAccelY_mss(), IMU.getAccelZ_mss(), IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads(), IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT());
  Serial.print("\t");
  Serial.print(b_x,6);
  Serial.print("\t");
  Serial.println(b_z,6);
  delay(20);
}
