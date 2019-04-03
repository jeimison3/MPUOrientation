#ifndef __KALMAN_FILTER
#define __KALMAN_FILTER

typedef struct{
  /* Kalman filter variables */
  float Q_angle; // Process noise variance for the accelerometer
  float Q_bias; // Process noise variance for the gyro bias
  float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

  float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
  float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
  float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

  float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
} KalmanFilterSet;

typedef KalmanFilterSet* pKalmanFilterSet;

#ifdef __cplusplus
extern "C" {
#endif

  pKalmanFilterSet createKalmanFilter();
  void freeKalmanFilter(pKalmanFilterSet P);

  float kalmanGetAngle(pKalmanFilterSet THIS, float newAngle, float newRate, float dt);
  void kalmanSetAngle(pKalmanFilterSet THIS, float angle);

  float kalmanGetRate(pKalmanFilterSet THIS);

  void kalmanSetQangle(pKalmanFilterSet THIS, float Q_angle);
  void kalmanSetQbias(pKalmanFilterSet THIS, float Q_bias);
  void kalmanSetRmeasure(pKalmanFilterSet THIS, float R_measure);

  float kalmanGetQangle(pKalmanFilterSet THIS);
  float kalmanGetQbias(pKalmanFilterSet THIS);
  float kalmanGetRmeasure(pKalmanFilterSet THIS);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
