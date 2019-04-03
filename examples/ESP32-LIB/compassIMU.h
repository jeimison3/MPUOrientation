#ifndef __CompassIMU
#define __CompassIMU

#include "compassSet.h"

typedef struct{
  IMUFLOAT pitch, roll, yaw;
} SimpleOrientation;

typedef struct{
  IMUFLOAT x, y, z;
} SimpleAxis;

typedef struct{
  IMUFLOAT scalar, x, y, z;
} IMUQuaternion;


#ifdef __cplusplus
extern "C" {
#endif


SimpleAxis poseFromAccelMag(SimpleAxis accel, SimpleAxis mag);



#ifdef __cplusplus
} // extern "C"
#endif

#endif
