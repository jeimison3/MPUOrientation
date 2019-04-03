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

  C.q0 = A.q0 * B.q0 - A.q1 * B.q1 - A.q2 * B.q2 - A.q3 * B.q3;
  C.q1 = A.q0 * B.q1 + A.q1 * B.q0 + A.q2 * B.q3 - A.q3 * B.q2;
  C.q2 = A.q0 * B.q2 - A.q1 * B.q3 + A.q2 * B.q0 + A.q3 * B.q1;
  C.q3 = A.q0 * B.q3 + A.q1 * B.q2 - A.q2 * B.q1 + A.q3 * B.q0;

  return C;
}

IMUQuaternion quaternionConjugate(IMUQuaternion A){
  IMUQuaternion T;T.q0 = A.q0;
  T.q1=-A.q1;T.q2=-A.q2;T.q3=-A.q3;
  return T;
}
