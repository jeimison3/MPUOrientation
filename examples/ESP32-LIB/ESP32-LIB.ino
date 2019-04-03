 
#include "MPU9250.h"
#include "compassSet.h"
#include "compassIMU.h"



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
  
  /*IMU.setMagCalX(-13.70, 1.43);
  IMU.setMagCalY(-20.12, 1.56);
  IMU.setMagCalZ(-68.43, 0.60);*/

  IMU.setMagCalX(-6.244253, 1.433413);
  IMU.setMagCalY(-14.033438, 1.526477);
  IMU.setMagCalZ(-67.622932, 0.607068);


}

void printBiases(){

  Serial.print("  //Acelerometro:\n");
  Serial.printf(
    "  IMU.setAccelCalX(%f, %f);\n  IMU.setAccelCalY(%f, %f);\n  IMU.setAccelCalZ(%f, %f);\n",
    IMU.getAccelBiasX_mss(), IMU.getAccelScaleFactorX(),
    IMU.getAccelBiasY_mss(), IMU.getAccelScaleFactorY(),
    IMU.getAccelBiasZ_mss(), IMU.getAccelScaleFactorZ()
  );

  Serial.print("  //Giroscopio:\n");
  Serial.printf(
    "  IMU.setGyroBiasX_rads(%f);\n  IMU.setGyroBiasY_rads(%f);\n  IMU.setGyroBiasZ_rads(%f);\n",
    IMU.getGyroBiasX_rads(),
    IMU.getGyroBiasY_rads(),
    IMU.getGyroBiasZ_rads()
  );


  Serial.print("  //Magnetometro:\n");
  Serial.printf(
    "  IMU.setMagCalX(%f, %f);\n  IMU.setMagCalY(%f, %f);\n  IMU.setMagCalZ(%f, %f);\n",
    IMU.getMagBiasX_uT(), IMU.getMagScaleFactorX(),
    IMU.getMagBiasY_uT(), IMU.getMagScaleFactorY(),
    IMU.getMagBiasZ_uT(), IMU.getMagScaleFactorZ()
  );

  Serial.println("Retomando em 10 segundos.");
  
  delay(10000);
}

void loop() {
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
  char spacing = '\t'; // Substitua por espaço se preferir


  Serial.println("\n\n\n\n\n\n\n\n\n\n==== MPU =====");
  Serial.printf("Accel:\tX:%f\tY:%f\tZ:%f\n", IMU.getAccelX_mss(), IMU.getAccelY_mss(), IMU.getAccelZ_mss());
  Serial.printf("Gyro:\tX:%f\tY:%f\tZ:%f\n", IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads());
  Serial.printf("Mag:\tX:%f\tY:%f\tZ:%f\n", IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT());
  


  /*
   * Cálculos trazidos desta fonte:
   * https://gist.github.com/shoebahmedadeel/0d8ca4eaa65664492cf1db2ab3a9e572
   * 
   * Com algumas adaptações, visto que aqui é usada uma biblioteca que dá valores convertidos
   * e calibrados.
   * 
   * Valores se mostram reais. Por hora, apenas usando acelerômetro e magnetômetro.
   */

  /*
  
  //Angulo Euler por accel
  float pitch = atan2 (IMU.getAccelY_mss() ,( sqrt ((IMU.getAccelX_mss() * IMU.getAccelX_mss()) + (IMU.getAccelZ_mss() * IMU.getAccelZ_mss()))));
  float roll = atan2(-IMU.getAccelX_mss() ,( sqrt((IMU.getAccelY_mss() * IMU.getAccelY_mss()) + (IMU.getAccelZ_mss() * IMU.getAccelZ_mss()))));

  // yaw por mag

  float Yh = (IMU.getMagY_uT() * cos(roll)) - (IMU.getMagZ_uT() * sin(roll));
  float Xh = (IMU.getMagX_uT() * cos(pitch))+(IMU.getMagY_uT() * sin(roll)*sin(pitch)) + (IMU.getMagZ_uT() * cos(roll) * sin(pitch));

  float yaw =  atan2(Yh, Xh);

  pitch *= 180.0/M_PI;
  roll *= 180.0/M_PI;
  yaw *= 180.0/M_PI;
  if(yaw < 0) yaw += 360;

  Serial.printf("\tYaw:%f\tPitch:%f\tRoll:%f", yaw, pitch, roll);
  */

  SimpleAxis accel,mag;
  
  accel.x = IMU.getAccelX_mss();
  accel.y = IMU.getAccelY_mss();
  accel.z = IMU.getAccelZ_mss();
  
  mag.x = IMU.getMagX_uT();
  mag.y = IMU.getMagY_uT();
  mag.z = IMU.getMagZ_uT();

  SimpleAxis DIREC = poseFromAccelMag(accel, mag);

  Serial.printf("\tYaw:%f\tPitch:%f\tRoll:%f", DIREC.x*180.0/M_PI, DIREC.y*180.0/M_PI, DIREC.z*180.0/M_PI);
  
  delay(230);
}
