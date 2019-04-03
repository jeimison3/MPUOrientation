#include <math.h>
#include "compassSet.h"
#include "compassIMU.h"
#include "vectorMath.h"

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
