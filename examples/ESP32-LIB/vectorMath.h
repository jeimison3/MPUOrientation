#ifndef __VECTORMATH_IMU
#define __VECTORMATH_IMU

#include "compassSet.h"
#include "compassIMU.h"


#ifdef __cplusplus
extern "C" {
#endif

  void normalizeAxis(SimpleAxis* A);
  SimpleAxis accelToEuler(SimpleAxis* accel);

  IMUQuaternion quaternionMulti(IMUQuaternion A, IMUQuaternion B);
  IMUQuaternion quaternionConjugate(IMUQuaternion A);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
