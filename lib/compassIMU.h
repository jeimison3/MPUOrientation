#ifndef __CompassIMU
#define __CompassIMU

#include "lib/compassSet.h"
#include "lib/KalmanPortFilter.h"

// Algoritmos de processamento disponíveis

typedef enum{
  ALGO_KALMAN_V1 = 0,
  ALGO_MADGWICK_V1
} FusionMethod;



// Aplicações com quatérnios
typedef struct{
  IMUFLOAT q0, q1, q2, q3;
} IMUQuaternion;


// Conjuntos de orientação

typedef struct{
  IMUFLOAT pitch, roll, yaw;
} IMUOrientation;

typedef struct{
  IMUFLOAT x, y, z;
} SimpleAxis;


// Comjunto de presets:

typedef struct{
  IMUFLOAT gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
  IMUFLOAT compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter
  IMUFLOAT kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter

  pKalmanFilterSet kalmanX, kalmanY, kalmanZ;
  IMUQuaternion QUATERN;
} CompassContext;

typedef CompassContext* pCompassContext;

// Conjuntos de sensores de entrada.

typedef struct{
  SimpleAxis ACCEL;
  SimpleAxis GYRO;
  SimpleAxis MAG;
} IMUFullFusion;

typedef struct{
  SimpleAxis ACCEL;
  SimpleAxis GYRO;
} IMUFusion;

#ifdef __cplusplus
extern "C" {
#endif

  pCompassContext createContext();
  IMUOrientation getOrientation(pCompassContext CONTEXTO, FusionMethod METHOD, IMUFusion ORIENT, IMUFLOAT DELTHA_TIME_US);
  IMUOrientation getFullOrientation(pCompassContext CONTEXTO, FusionMethod METHOD, IMUFullFusion ORIENT, IMUFLOAT DELTHA_TIME_US);


  SimpleAxis poseFromAccelMag(SimpleAxis accel, SimpleAxis mag);


#ifdef __cplusplus
} // extern "C"
#endif

#endif
