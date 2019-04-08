import processing.serial.*;
Serial serial;

String stringPitch, stringPitch2, stringPitch3;
String stringRoll, stringRoll2, stringRoll3;
String stringYaw, stringYaw2, stringYaw3;

final int width = 500;
final int height = 400;


float[] pitch = new float[width];
float[] pitch2 = new float[width];
float[] pitch3 = new float[width];

float[] roll = new float[width];
float[] roll2 = new float[width];
float[] roll3 = new float[width];

float[] yaw = new float[width];
float[] yaw2 = new float[width];
float[] yaw3 = new float[width];

int scrPitch, scrRoll, scrYaw;

boolean drawValues  = false;

void setup() {
  size(500, 400);
  for(int i = 0; i< Serial.list().length; i++){
    print("["+i+"]"+Serial.list()[i]+" | ");
  }
  
  serial = new Serial(this, Serial.list()[32], 115200); // Set this to your serial port obtained using the line above
  serial.bufferUntil('\n'); // Buffer until line feed
  
  scrPitch = height*2/6;
  scrRoll = height*4/6 ;
  scrYaw = height*6/6;

  for (int i = 0; i < width; i++) { // center all variables
    pitch[i] = scrPitch;
    pitch2[i] = scrPitch;
    pitch3[i] = scrPitch;
    roll[i] = scrRoll;
    roll2[i] = scrRoll;
    roll3[i] = scrRoll;
    yaw[i] = scrYaw;
    yaw2[i] = scrYaw;
    yaw3[i] = scrYaw;
  }

  drawGraph(); // Draw graph at startup
}

void draw() {
  /* Draw Graph */
  if (drawValues) {
    drawValues = false;
    drawGraph();
  }
}

void drawGraph() {
  background(0); // White
  for (int i = 0; i < width; i++) {
    stroke(10); // Grey
    line(i*10, 0, i*10, height);
    line(0, i*10, width, i*10);
  }

  stroke(255); // Black
  for (int i = 1; i <= 2; i++)
    line(0, height/3*i, width, height/3*i); // Draw line, indicating -90 deg, 0 deg and 90 deg

  convert();
  drawAxisX();
  drawAxisY();
  drawAxisZ();
}

void serialEvent (Serial serial) {
  // Get the ASCII strings:
  serial.readStringUntil('\t'); // Lixo
  
  stringPitch = serial.readStringUntil('\t');
  stringPitch2 = serial.readStringUntil('\t');
  stringPitch3 = serial.readStringUntil('\t');
  
  serial.readStringUntil('\t'); // Lixo
  
  stringRoll = serial.readStringUntil('\t');
  stringRoll2 = serial.readStringUntil('\t');
  stringRoll3 = serial.readStringUntil('\t');

  serial.readStringUntil('\t'); // Lixo

  stringYaw = serial.readStringUntil('\t');
  stringYaw2 = serial.readStringUntil('\t');
  stringYaw3 = serial.readStringUntil('\n');

  serial.clear(); // Clear buffer
  drawValues = true; // Draw the graph

  printAxis(); // Used for debugging
}

void printAxis() {
  
  print(stringPitch);
  print(stringRoll);
  print(stringYaw);

  print("|\t");

  print(stringPitch2);
  print(stringRoll2);
  print(stringYaw2);
  
  print("|\t");

  print(stringPitch3);
  print(stringRoll3);
  print(stringYaw3);


  println();
}
