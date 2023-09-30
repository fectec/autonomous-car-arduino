// INCLUDES:
// ULTRASONIC SENSORS
// LINE FOLLOWING SENSORS
// COLOR SENSOR

#include <NewPing.h>
#include <Adafruit_TCS34725.h>

Adafruit_TCS34725 TCS = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

// PINS SETUP

// Ultrasonic sensors 

#define PING_A 2 // Front
#define PING_B 3 // Rigth
#define PING_C 4 // Left

// Line following sensors

#define LINE_LEFT 5 
#define LINE_CENTER 6
#define LINE_RIGTH 7

// CONSTANTS

// Ultrasonic sensors

#define SONAR_NUM 3
#define MAX_DISTANCE 100

// Baud rate

#define BAUD_RATE 115200

// DELAYS

// Communication

#define COMMUNICATION_TIME 100

// GLOBAL VARIABLES

// Ultrasonic sensors

NewPing sonars[SONAR_NUM] = {
  NewPing(PING_A, PING_A, MAX_DISTANCE),
  NewPing(PING_B, PING_B, MAX_DISTANCE),
  NewPing(PING_C, PING_C, MAX_DISTANCE)
};

// Color

String color = "";

// Checkpoints

String checkpoints[] = {"CYAN", "PASTEL PINK", "PINK"};
short int checkpoint = 0;

// Message

String message = "";

// SENSOR FUNCTIONS

// Color sensor

String getColor() {  
  uint16_t R, G, B, C;

  String color = "NULL";

  TCS.getRawData(&R, &G, &B, &C);

  if (R < 30 && G < 30 && B < 30) {
    color = "BLACK";
  }
  else if (R > 1.5 * G && R > 1.5 * B) {
    color = "RED";
  }
  else if (G > R && G > B) {
    color = "GREEN";
  }
  else if (B > 1.5 * G && B > 1.5 * R) {
    color = "BLUE";
  }
  else if (R > 1.5 * B && G > 1.5 * B) {
    color = "YELLOW";
  }
  else if (R > 1.5 * B && B > 1.5 * G) {
    color = "PINK";
  }
  else if (R > B && B > G) {
    color = "PASTEL PINK"; 
  }
  else if (G > 1.5 * R && B > 1.5 * R) {
    color = "CYAN";
  }

  return color;
}

// CAR FUNCTIONS

// Find checkpoint

short int findCheckpointIndex(String color) {
  for (uint8_t i = 0; i < sizeof(checkpoints) / sizeof(checkpoints[0]); i++) {
    if (checkpoints[i] == color) {
      return i;  
    }
  }

  return -1;  
}

// SETUP

void setup() {

  Serial.begin(BAUD_RATE);

  // CONFIGURATION OF PIN MODES

  // Line following sensors

  pinMode(LINE_RIGTH, INPUT);
  pinMode(LINE_CENTER, INPUT);
  pinMode(LINE_LEFT, INPUT);
}

// LOOP

void loop() {

  // COLOR SENSOR

  color = getColor();

  if(findCheckpointIndex(color) != -1) {
    checkpoint = findCheckpointIndex(color) + 1;
  }

  message = color + "," + String(checkpoint); 

  if(checkpoint == 0) {

    // ULTRASONIC SENSORS

    for (uint8_t i = 0; i < SONAR_NUM; i++) {
      message += "," + String(sonars[i].ping_cm());
    }
  }
  else if(checkpoint == 3) {

    // LINE FOLLOWING SENSORS

    for(short unsigned int i = LINE_LEFT; i <= LINE_RIGTH; i++) {
      message += String(digitalRead(i));
    }
  }

  Serial.println(message);

  delay(COMMUNICATION_TIME);
}