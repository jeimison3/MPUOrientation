#include <math.h>
#include <stdlib.h>
#include "lib/compassSet.h"
#include "lib/compassIMU.h"
#include "lib/vectorMath.h"

#include "lib/KalmanPortFilter.h" // Kalman Filter

pCompassContext CONTEXT;


pCompassContext createContext(){
  pCompassContext TMP = malloc(sizeof(CompassContext));

  TMP->gyroXangle = 0; TMP->gyroYangle = 0; TMP->gyroZangle = 0;
  TMP->compAngleX = 0; TMP->compAngleY = 0; TMP->compAngleZ = 0;
  TMP->kalAngleX = 0; TMP->kalAngleY = 0; TMP->kalAngleZ = 0;

  TMP->kalmanX = createKalmanFilter();
  TMP->kalmanY = createKalmanFilter();
  TMP->kalmanZ = createKalmanFilter(); // Nem em todo caso...

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


IMUOrientation getFullOrientation(pCompassContext CONTEXTO, FusionMethod METHOD, IMUFullFusion ORIENT, IMUFLOAT DELTHA_TIME_US){
  CONTEXT = CONTEXTO;
  if(METHOD == ALGO_KALMAN_V1){
    return Kalman_V1_FullOrientation(ORIENT,DELTHA_TIME_US);
  }
}

IMUOrientation getOrientation(pCompassContext CONTEXTO, FusionMethod METHOD, IMUFusion ORIENT, IMUFLOAT DELTHA_TIME_US){
  CONTEXT = CONTEXTO;
  if(METHOD == ALGO_KALMAN_V1){
    return Kalman_V1_Orientation(ORIENT,DELTHA_TIME_US);
  }
}






SimpleAxis poseFromAccelMag(SimpleAxis accel, SimpleAxis mag){
  SimpleAxis resu = accelToEuler(&accel);

  IMUQuaternion m, q;

  IMUFLOAT cosX2 = cos(resu.x / 2.0f);
  IMUFLOAT sinX2 = sin(resu.x / 2.0f);
  IMUFLOAT cosY2 = cos(resu.y / 2.0f);
  IMUFLOAT sinY2 = sin(resu.y / 2.0f);

  q.scalar = cosX2 * cosY2;
  q.x = sinX2 * cosY2;
  q.y = cosX2 * sinY2;
  q.z = -sinX2 * sinY2;

  m.scalar = 0;
  q.x = mag.x;
  q.y = mag.y;
  q.z = mag.z;

  m = quaternionMulti( quaternionMulti(q,m), quaternionConjugate(q) );
  //m = q * m * q.conjugate();

  resu.z = -atan2(m.y, m.x);
  return resu;
}
