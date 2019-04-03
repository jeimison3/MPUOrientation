#include <math.h>
#include "lib/compassSet.h"
#include "lib/compassIMU.h"



void normalizeAxis(SimpleAxis* A){
  IMUFLOAT larg = sqrt( (A->x * A->x) + (A->y * A->y) + (A->z * A->z) );
  if(larg == 0) return;
  A->x /= larg;
  A->y /= larg;
  A->z /= larg;
}

SimpleAxis accelToEuler(SimpleAxis* accel){
  normalizeAxis(accel);

  SimpleAxis resu;
  resu.x = atan2(accel->y, accel->z);
  resu.x = -atan2(accel->x, sqrt(accel->y * accel->y + accel->z * accel->z));
  resu.z = 0;

  return resu;
}


// IMUQuaternion:

IMUQuaternion quaternionMulti(IMUQuaternion A, IMUQuaternion B){
  IMUQuaternion C;

  C.scalar = A.scalar * B.scalar - A.x * B.x - A.y * B.y - A.z * B.z;
  C.x = A.scalar * B.x + A.x * B.scalar + A.y * B.z - A.z * B.y;
  C.y = A.scalar * B.y - A.x * B.z + A.y * B.scalar + A.z * B.x;
  C.z = A.scalar * B.z + A.x * B.y - A.y * B.x + A.z * B.scalar;

  return C;
}

IMUQuaternion quaternionConjugate(IMUQuaternion A){
  IMUQuaternion T;T.scalar = A.scalar;
  T.x=-A.x;T.y=-A.y;T.z=-A.z;
  return T;
}
