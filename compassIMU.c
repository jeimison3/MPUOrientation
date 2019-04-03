#include <math.h>
#include <stdlib.h>
#include "lib/compassSet.h"
#include "lib/compassIMU.h"
#include "lib/vectorMath.h"

#include "lib/KalmanPortFilter.h" // Kalman Filter

#include "lib/MadgwickAHRS.h" // Madgwick quaternions

pCompassContext CONTEXT;


pCompassContext createContext(){
  pCompassContext TMP = malloc(sizeof(CompassContext));

  TMP->gyroXangle = 0; TMP->gyroYangle = 0; TMP->gyroZangle = 0;
  TMP->compAngleX = 0; TMP->compAngleY = 0; TMP->compAngleZ = 0;
  TMP->kalAngleX = 0; TMP->kalAngleY = 0; TMP->kalAngleZ = 0;

  TMP->kalmanX = createKalmanFilter();
  TMP->kalmanY = createKalmanFilter();
  TMP->kalmanZ = createKalmanFilter(); // Nem em todo caso...

  TMP->QUATERN.q0 = 1.0f;
  TMP->QUATERN.q1 = 0.0f;
  TMP->QUATERN.q2 = 0.0f;
  TMP->QUATERN.q3 = 0.0f;

  return TMP;
}

void freeContext(pCompassContext C){
  freeKalmanFilter(C->kalmanX);
  freeKalmanFilter(C->kalmanY);
  freeKalmanFilter(C->kalmanZ);
  free(C);
}


IMUOrientation Kalman_V1_FullOrientation(IMUFullFusion ORIENT, IMUFLOAT DELTHA_TIME_US){
  IMUOrientation ORI;


  ORI.roll  = atan(ORIENT.ACCEL.y / sqrt(ORIENT.ACCEL.x * ORIENT.ACCEL.x + ORIENT.ACCEL.z * ORIENT.ACCEL.z)) * COMP_RAD_TO_DEG;
  ORI.pitch = atan2(-ORIENT.ACCEL.x, ORIENT.ACCEL.z) * COMP_RAD_TO_DEG;

  double gyroXrate = ORIENT.GYRO.x * COMP_RAD_TO_DEG; // Convert to deg/s
  double gyroYrate = ORIENT.GYRO.y * COMP_RAD_TO_DEG; // Convert to deg/s
  double gyroZrate = ORIENT.GYRO.z * COMP_RAD_TO_DEG; // Convert to deg/s


  if ((ORI.pitch < -90 && CONTEXT->kalAngleY > 90) || (ORI.pitch > 90 && CONTEXT->kalAngleY < -90)) {
    kalmanSetAngle(CONTEXT->kalmanY,ORI.pitch);
    CONTEXT->compAngleY = ORI.pitch;
    CONTEXT->kalAngleY = ORI.pitch;
    CONTEXT->gyroYangle = ORI.pitch;
  } else
    CONTEXT->kalAngleY = kalmanGetAngle(CONTEXT->kalmanY, ORI.pitch, gyroYrate, DELTHA_TIME_US); // Calculate the angle using a Kalman filter

  if (abs(CONTEXT->kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading


  CONTEXT->kalAngleX = kalmanGetAngle(CONTEXT->kalmanX, ORI.roll, gyroXrate, DELTHA_TIME_US); // Calculate the angle using a Kalman filter


  CONTEXT->gyroXangle += gyroXrate * DELTHA_TIME_US; // Calculate gyro angle without any filter
  CONTEXT->gyroYangle += gyroYrate * DELTHA_TIME_US;
  CONTEXT->gyroZangle += gyroZrate * DELTHA_TIME_US;


  // Aplica filtro complementar:

  CONTEXT->compAngleX = 0.93 * (CONTEXT->compAngleX + gyroXrate * DELTHA_TIME_US) + 0.07 * ORI.roll; // Calculate the angle using a Complimentary filter
  CONTEXT->compAngleY = 0.93 * (CONTEXT->compAngleY + gyroYrate * DELTHA_TIME_US) + 0.07 * ORI.pitch;
  // Eixo Z (Yaw) fica mais para o fim


  // Reset the gyro angle when it has drifted too much
  if (CONTEXT->gyroXangle < -180 || CONTEXT->gyroXangle > 180)
    CONTEXT->gyroXangle = CONTEXT->kalAngleX;
  if (CONTEXT->gyroYangle < -180 || CONTEXT->gyroYangle > 180)
    CONTEXT->gyroYangle = CONTEXT->kalAngleY;


  // Normaliza leituras do magnetômetro
  IMUFLOAT magNormXuT = ORIENT.MAG.x;
  IMUFLOAT magNormYuT = ORIENT.MAG.y;
  IMUFLOAT magNormZuT = ORIENT.MAG.z;
  IMUFLOAT mag_norm = sqrt( (magNormXuT*magNormXuT)+(magNormYuT*magNormYuT)+(magNormZuT*magNormZuT) );

  magNormXuT /= mag_norm;
  magNormYuT /= mag_norm;
  magNormZuT /= mag_norm;


  /*
   * Cálculos trazidos inicialmente da fonte:
   * https://gist.github.com/shoebahmedadeel/0d8ca4eaa65664492cf1db2ab3a9e572
   * Com diversas adaptações.
   */

   // Cast para RAD
   IMUFLOAT kalRollRAD = CONTEXT->kalAngleX * COMP_DEG_TO_RAD;
   IMUFLOAT kalPitchRAD = CONTEXT->kalAngleY * COMP_DEG_TO_RAD;

   IMUFLOAT Yh = (magNormYuT * cos(kalRollRAD)) - (magNormZuT * sin(kalRollRAD));
   IMUFLOAT Xh = (magNormXuT * cos(kalPitchRAD))+(magNormYuT * sin(kalRollRAD)*sin(kalPitchRAD)) + (magNormZuT * cos(kalRollRAD) * sin(kalPitchRAD));


   // Cast de volta
   ORI.yaw =  atan2(Yh, Xh) * COMP_RAD_TO_DEG;

   CONTEXT->kalAngleZ = kalmanGetAngle(CONTEXT->kalmanZ, ORI.yaw, gyroZrate, DELTHA_TIME_US);


   // Reseta ângulo do giroscópio quando ocorreram muitos drifts
   if (CONTEXT->gyroZangle < -180 || CONTEXT->gyroZangle > 180)
   CONTEXT->gyroZangle = CONTEXT->kalAngleZ;


   //Filtro complementar Z
   CONTEXT->compAngleZ = 0.93 * (CONTEXT->compAngleZ + gyroZrate * DELTHA_TIME_US) + 0.07 * ORI.yaw;


  IMUOrientation TMP;
  TMP.pitch = CONTEXT->kalAngleY;
  TMP.roll = CONTEXT->kalAngleX;
  TMP.yaw = CONTEXT->compAngleZ;

  return TMP;
}

IMUOrientation Kalman_V1_Orientation(IMUFusion ORIENT, IMUFLOAT DELTHA_TIME_US){
  IMUOrientation ORI;


  ORI.roll  = atan(ORIENT.ACCEL.y / sqrt(ORIENT.ACCEL.x * ORIENT.ACCEL.x + ORIENT.ACCEL.z * ORIENT.ACCEL.z)) * COMP_RAD_TO_DEG;
  ORI.pitch = atan2(-ORIENT.ACCEL.x, ORIENT.ACCEL.z) * COMP_RAD_TO_DEG;

  double gyroXrate = ORIENT.GYRO.x * COMP_RAD_TO_DEG; // Convert to deg/s
  double gyroYrate = ORIENT.GYRO.y * COMP_RAD_TO_DEG; // Convert to deg/s


  if ((ORI.pitch < -90 && CONTEXT->kalAngleY > 90) || (ORI.pitch > 90 && CONTEXT->kalAngleY < -90)) {
    kalmanSetAngle(CONTEXT->kalmanY,ORI.pitch);
    CONTEXT->compAngleY = ORI.pitch;
    CONTEXT->kalAngleY = ORI.pitch;
    CONTEXT->gyroYangle = ORI.pitch;
  } else
    CONTEXT->kalAngleY = kalmanGetAngle(CONTEXT->kalmanY, ORI.pitch, gyroYrate, DELTHA_TIME_US); // Calculate the angle using a Kalman filter

  if (abs(CONTEXT->kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading

  CONTEXT->kalAngleX = kalmanGetAngle(CONTEXT->kalmanX, ORI.roll, gyroXrate, DELTHA_TIME_US); // Calculate the angle using a Kalman filter

  CONTEXT->gyroXangle += gyroXrate * DELTHA_TIME_US; // Calculate gyro angle without any filter
  CONTEXT->gyroYangle += gyroYrate * DELTHA_TIME_US;


  // Aplica filtro complementar:

  CONTEXT->compAngleX = 0.93 * (CONTEXT->compAngleX + gyroXrate * DELTHA_TIME_US) + 0.07 * ORI.roll; // Calculate the angle using a Complimentary filter
  CONTEXT->compAngleY = 0.93 * (CONTEXT->compAngleY + gyroYrate * DELTHA_TIME_US) + 0.07 * ORI.pitch;


  // Reset the gyro angle when it has drifted too much
  if (CONTEXT->gyroXangle < -180 || CONTEXT->gyroXangle > 180)
    CONTEXT->gyroXangle = CONTEXT->kalAngleX;
  if (CONTEXT->gyroYangle < -180 || CONTEXT->gyroYangle > 180)
    CONTEXT->gyroYangle = CONTEXT->kalAngleY;


  IMUOrientation TMP;
  TMP.pitch=CONTEXT->kalAngleY;
  TMP.roll=CONTEXT->kalAngleX;
  TMP.yaw = 0;

  return TMP;
}


IMUOrientation MadgwickOrientation(IMUQuaternion QT){
  IMUOrientation TMP;

  // https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU9150/MPU9150_9Axis_MotionApps41.h
  // Get gravity:
  SimpleAxis Grav;
  Grav.x = 2 * (QT.q1*QT.q3 - QT.q0*QT.q2);
  Grav.y = 2 * (QT.q0*QT.q1 + QT.q2*QT.q3);
  Grav.z = QT.q0*QT.q0 - QT.q1*QT.q1 - QT.q2*QT.q2 + QT.q3*QT.q3;

  // https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU9150/MPU9150_9Axis_MotionApps41.h
  // dmpGetYawPitchRoll
  TMP.pitch = atan(Grav.x / sqrt(Grav.y*Grav.y + Grav.z*Grav.z));
  TMP.roll = atan(Grav.y / sqrt(Grav.x*Grav.x + Grav.z*Grav.z));
  TMP.yaw = atan2(2*QT.q1*QT.q2 - 2*QT.q0*QT.q3, 2*QT.q0*QT.q0 + 2*QT.q1*QT.q1 - 1);

  TMP.pitch *= COMP_RAD_TO_DEG;
  TMP.roll *= COMP_RAD_TO_DEG;
  TMP.yaw *= COMP_RAD_TO_DEG;
  return TMP;
}

IMUOrientation MadgwickKalman_V1_Orientation(IMUFusion ORIENT, IMUFLOAT DELTHA_TIME_US){

  MadgwickAHRSupdateIMU(ORIENT.GYRO.x, ORIENT.GYRO.y, ORIENT.GYRO.z,
    ORIENT.ACCEL.x, ORIENT.ACCEL.y, ORIENT.ACCEL.z, &(CONTEXT->QUATERN));

  return MadgwickOrientation(CONTEXT->QUATERN);
}

IMUOrientation MadgwickKalman_V1_FullOrientation(IMUFullFusion ORIENT, IMUFLOAT DELTHA_TIME_US){

  MadgwickAHRSupdate(ORIENT.GYRO.x, ORIENT.GYRO.y, ORIENT.GYRO.z,
    ORIENT.ACCEL.x, ORIENT.ACCEL.y, ORIENT.ACCEL.z,
    ORIENT.MAG.x, ORIENT.MAG.y, ORIENT.MAG.z, &(CONTEXT->QUATERN));

  return MadgwickOrientation(CONTEXT->QUATERN);
}


IMUOrientation getFullOrientation(pCompassContext CONTEXTO, FusionMethod METHOD, IMUFullFusion ORIENT, IMUFLOAT DELTHA_TIME_US){
  CONTEXT = CONTEXTO;
  if(METHOD == ALGO_KALMAN_V1){
    return Kalman_V1_FullOrientation(ORIENT,DELTHA_TIME_US);
  }else if(METHOD == ALGO_MADGWICK_V1){
    return MadgwickKalman_V1_FullOrientation(ORIENT,DELTHA_TIME_US);
  }
}

IMUOrientation getOrientation(pCompassContext CONTEXTO, FusionMethod METHOD, IMUFusion ORIENT, IMUFLOAT DELTHA_TIME_US){
  CONTEXT = CONTEXTO;
  if(METHOD == ALGO_KALMAN_V1){
    return Kalman_V1_Orientation(ORIENT,DELTHA_TIME_US);
  }else if(METHOD == ALGO_MADGWICK_V1){
    return MadgwickKalman_V1_Orientation(ORIENT,DELTHA_TIME_US);
  }
}
