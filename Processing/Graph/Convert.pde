//convert all axis
final int minAngle = -90;
final int maxAngle = 90;

final float kMulti = 0.33;

void convert() {
  /* Convert the gyro x-axis */
  if (stringPitch != null) {
    stringPitch = trim(stringPitch); // Trim off any whitespace
    pitch[pitch.length - 1] = scrPitch-map(float(stringPitch), minAngle*2, maxAngle*2, 0, height*kMulti); // Convert to a float and map to the screen height, then save in buffer
  }

  /* Convert the gyro y-axis */
  if (stringPitch2 != null) {
    stringPitch2 = trim(stringPitch2); // Trim off any whitespace
    pitch2[pitch2.length - 1] = scrPitch - map(-float(stringPitch2), minAngle*2, maxAngle*2, 0, height*kMulti); // Convert to a float and map to the screen height, then save in buffer
  }

  /* Convert the accelerometer x-axis */
  if (stringRoll != null) {
    stringRoll = trim(stringRoll); // Trim off any whitespace
    roll[roll.length - 1] = scrRoll - map(float(stringRoll), minAngle, maxAngle, 0, height*kMulti); // Convert to a float and map to the screen height, then save in buffer
  }

  /* Convert the accelerometer y-axis */
  if (stringRoll2 != null) {
    stringRoll2 = trim(stringRoll2); // Trim off any whitespace
    roll2[roll2.length - 1] = scrRoll - map(float(stringRoll2), minAngle, maxAngle, 0, height*kMulti); // Convert to a float and map to the screen height, then save in buffer
  }

  /* Convert the complementary filter x-axis */
  if (stringYaw != null) {
    stringYaw = trim(stringYaw); // Trim off any whitespace
    yaw[yaw.length - 1] = scrYaw - map(float(stringYaw), minAngle*2, maxAngle*2, 0, height*kMulti); // Convert to a float and map to the screen height, then save in buffer
  }

  /* Convert the complementary filter x-axis */
  if (stringYaw2 != null) {
    stringYaw2 = trim(stringYaw2); // Trim off any whitespace
    yaw2[yaw2.length - 1] = scrYaw - map(float(stringYaw2), minAngle*2, maxAngle*2, 0, height*kMulti); // Convert to a float and map to the screen height, then save in buffer
  }

  
  /*if (stringKalmanX != null) {
    stringKalmanX = trim(stringKalmanX); // Trim off any whitespace
    kalmanX[kalmanX.length - 1] = map(float(stringKalmanX), minAngle, maxAngle, 0, height*kMulti); // Convert to a float and map to the screen height, then save in buffer
  }

  
  if (stringKalmanY != null) {
    stringKalmanY = trim(stringKalmanY); // Trim off any whitespace
    kalmanY[kalmanY.length - 1] = map(float(stringKalmanY), minAngle, maxAngle, 0, height*kMulti); // Convert to a float and map to the screen height, then save in buffer
  }*/
}
