void drawAxisX() {
  /* Draw gyro x-axis */
  noFill();
  stroke(0, 200, 255);
  // Redraw everything
  beginShape();
  vertex(0, pitch[0]);
  for (int i = 1; i < pitch.length; i++) {
    if ((pitch[i] < height/3 && pitch[i - 1] > height/3*2) || (pitch[i] > height/3*2 && pitch[i - 1] < height/3)) {
      endShape();
      beginShape();
    }
    vertex(i, pitch[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < pitch.length;i++)
    pitch[i-1] = pitch[i];

  /* Draw acceleromter x-axis */
  noFill();
  //stroke(0, 255, 0); // Green
  // Redraw everything
  beginShape();
  vertex(0, roll[0]);
  for (int i = 1; i < roll.length; i++) {
    if ((roll[i] < height/3 && roll[i - 1] > height/3*2) || (roll[i] > height/3*2 && roll[i - 1] < height/3)) {
      endShape();
      beginShape();
    }
    vertex(i, roll[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < roll.length;i++)
    roll[i-1] = roll[i];

  /* Draw complementary filter x-axis */
  noFill();
  //stroke(0, 0, 255); // Blue
  // Redraw everything
  beginShape();
  vertex(0, yaw[0]);
  for (int i = 1; i < yaw.length; i++) {
    if ((yaw[i] < height/3 && yaw[i - 1] > height/3*2) || (yaw[i] > height/3*2 && yaw[i - 1] < height/3)) {
      endShape();
      beginShape();
    }
    vertex(i, yaw[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < yaw.length; i++)
    yaw[i-1] = yaw[i];
}

void drawAxisY() {
  /* Draw gyro y-axis */
  noFill();
  // stroke(124, 252, 0);
  stroke(255, 190, 0);
  // Redraw everything
  beginShape();
  vertex(0, pitch2[0]);
  for (int i = 1; i < pitch2.length; i++) {
    if ((pitch2[i] < height/3 && pitch2[i - 1] > height/3*2) || (pitch2[i] > height/3*2 && pitch2[i - 1] < height/3)) {
      endShape();
      beginShape();
    }
    vertex(i, pitch2[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < pitch2.length;i++)
   pitch2[i-1] = pitch2[i];

  /* Draw acceleromter y-axis */
  noFill();
  //stroke(0, 255, 255); // Light blue
  // Redraw everything
  beginShape();
  vertex(0, roll2[0]);
  for (int i = 1; i < roll2.length; i++) {
    if ((roll2[i] < height/3 && roll2[i - 1] > height/3*2) || (roll2[i] > height/3*2 && roll2[i - 1] < height/3)) {
      endShape();
      beginShape();
    }
    vertex(i, roll2[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < roll2.length;i++)
    roll2[i-1] = roll2[i];

  /* Draw complementary filter y-axis */
  noFill();
  //stroke(124, 252, 0); // Lawn Green
  // Redraw everything
  beginShape();
  vertex(0, yaw2[0]);
  for (int i = 1; i < yaw2.length; i++) {
    if ((yaw2[i] < height/3 && yaw2[i - 1] > height/3*2) || (yaw2[i] > height/3*2 && yaw2[i - 1] < height/3)) {
      endShape();
      beginShape();
    }
    vertex(i, yaw2[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < yaw2.length;i++)
    yaw2[i-1] = yaw2[i];
}

void drawAxisZ() {
  /* Draw gyro x-axis */
  noFill();
  stroke(0, 200, 0);
  // Redraw everything
  beginShape();
  vertex(0, pitch3[0]);
  for (int i = 1; i < pitch3.length; i++) {
    if ((pitch3[i] < height/3 && pitch3[i - 1] > height/3*2) || (pitch3[i] > height/3*2 && pitch3[i - 1] < height/3)) {
      endShape();
      beginShape();
    }
    vertex(i, pitch3[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < pitch3.length;i++)
    pitch3[i-1] = pitch3[i];

  /* Draw acceleromter x-axis */
  noFill();
  //stroke(0, 255, 0); // Green
  // Redraw everything
  beginShape();
  vertex(0, roll[0]);
  for (int i = 1; i < roll3.length; i++) {
    if ((roll3[i] < height/3 && roll3[i - 1] > height/3*2) || (roll3[i] > height/3*2 && roll3[i - 1] < height/3)) {
      endShape();
      beginShape();
    }
    vertex(i, roll3[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < roll3.length;i++)
    roll3[i-1] = roll3[i];

  /* Draw complementary filter x-axis */
  noFill();
  //stroke(0, 0, 255); // Blue
  // Redraw everything
  beginShape();
  vertex(0, yaw3[0]);
  for (int i = 1; i < yaw3.length; i++) {
    if ((yaw3[i] < height/3 && yaw3[i - 1] > height/3*2) || (yaw3[i] > height/3*2 && yaw3[i - 1] < height/3)) {
      endShape();
      beginShape();
    }
    vertex(i, yaw3[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i <yaw3.length; i++)
    yaw3[i-1] = yaw3[i];
}
