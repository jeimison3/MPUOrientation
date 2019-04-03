
#include "lib/KalmanPortFilter.h"
#include <stdlib.h>

pKalmanFilterSet createKalmanFilter(){
  pKalmanFilterSet P = malloc( sizeof(KalmanFilterSet) );
  /* We will set the variables like so, these can also be tuned by the user */
  P->Q_angle = 0.001f;
  P->Q_bias = 0.003f;
  P->R_measure = 0.03f;

  P->angle = 0.0f; // Reset the angle
  P->bias = 0.0f; // Reset bias

  P->P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
  P->P[0][1] = 0.0f;
  P->P[1][0] = 0.0f;
  P->P[1][1] = 0.0f;
  return P;
}

void freeKalmanFilter(pKalmanFilterSet P){
  free(P);
}

// Angulos:

float kalmanGetAngle(pKalmanFilterSet THIS, float newAngle, float newRate, float dt){
  // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
  // Modified by Kristian Lauszus
  // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

  // Discrete Kalman filter time update equations - Time Update ("Predict")
  // Update xhat - Project the state ahead
  /* Step 1 */
  THIS->rate = newRate - THIS->bias;
  THIS->angle += dt * THIS->rate;

  // Update estimation error covariance - Project the error covariance ahead
  /* Step 2 */
  THIS->P[0][0] += dt * (dt*THIS->P[1][1] - THIS->P[0][1] - THIS->P[1][0] + THIS->Q_angle);
  THIS->P[0][1] -= dt * THIS->P[1][1];
  THIS->P[1][0] -= dt * THIS->P[1][1];
  THIS->P[1][1] += THIS->Q_bias * dt;

  // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
  // Calculate Kalman gain - Compute the Kalman gain
  /* Step 4 */
  float S = THIS->P[0][0] + THIS->R_measure; // Estimate error
  /* Step 5 */
  float K[2]; // Kalman gain - This is a 2x1 vector
  K[0] = THIS->P[0][0] / S;
  K[1] = THIS->P[1][0] / S;

  // Calculate angle and bias - Update estimate with measurement zk (newAngle)
  /* Step 3 */
  float y = newAngle - THIS->angle; // Angle difference
  /* Step 6 */
  THIS->angle += K[0] * y;
  THIS->bias += K[1] * y;

  // Calculate estimation error covariance - Update the error covariance
  /* Step 7 */
  float P00_temp = THIS->P[0][0];
  float P01_temp = THIS->P[0][1];

  THIS->P[0][0] -= K[0] * P00_temp;
  THIS->P[0][1] -= K[0] * P01_temp;
  THIS->P[1][0] -= K[1] * P00_temp;
  THIS->P[1][1] -= K[1] * P01_temp;

  return THIS->angle;
}

void kalmanSetAngle(pKalmanFilterSet THIS, float angle){
   THIS->angle = angle;
}

float kalmanGetRate(pKalmanFilterSet THIS){
  return THIS->rate;
}


// WRITE

void kalmanSetQangle(pKalmanFilterSet THIS, float Q_angle){
   THIS->Q_angle = Q_angle;
}

void kalmanSetQbias(pKalmanFilterSet THIS, float Q_bias){
   THIS->Q_bias = Q_bias;
}

void kalmanSetRmeasure(pKalmanFilterSet THIS, float R_measure){
   THIS->R_measure = R_measure;
}

// READ

float kalmanGetQangle(pKalmanFilterSet THIS){
  return THIS->Q_angle;
}

float kalmanGetQbias(pKalmanFilterSet THIS){
  return THIS->Q_bias;
}

float kalmanGetRmeasure(pKalmanFilterSet THIS){
  return THIS->R_measure;
}
