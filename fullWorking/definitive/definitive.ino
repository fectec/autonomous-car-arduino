#include <NewPing.h>
#include <Adafruit_TCS34725.h>
#include <LiquidCrystal_I2C.h>

Adafruit_TCS34725 TCS = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
LiquidCrystal_I2C LCD(0x27, 16, 2);

// PINS SETUP

// Motors

#define MOTOR_ENABLE_A 3
#define MOTOR_A_1 2
#define MOTOR_A_2 4

#define MOTOR_ENABLE_B 5
#define MOTOR_B_1 6
#define MOTOR_B_2 7

// Ultrasonic sensors 

#define PING_A 8 // Front
#define PING_B 9 // Rigth

// Transistor

#define TRANSISTOR_BASE 10

// Line sensors

#define LINE_LEFT 11
#define LINE_CENTER 12
#define LINE_RIGTH 13

// CONSTANTS

// Ultrasonic sensors

#define SONAR_NUM 2
#define MAX_DISTANCE 200

// Baud rate

#define BAUD_RATE 115200

// DELAYS

// Movement

#define MOVE_TIME 900
#define ROTATE_TIME 500
#define STOP_TIME 900

// Display

#define DISPLAY_TIME 500

// GLOBAL VARIABLES

// Ultrasonic sensors

NewPing sonar[SONAR_NUM] = {
  NewPing(PING_A, PING_A, MAX_DISTANCE),
  NewPing(PING_B, PING_B, MAX_DISTANCE),
};

// Checkpoints

String checkpoints[] = {"CYAN", "PASTEL PINK", "PINK"};
short int checkpoint = 0;

// MOTOR FUNCTIONS

// On and off

void turnOnMotorsA() {
  digitalWrite(MOTOR_ENABLE_A, HIGH);
}

void turnOffMotorsA() {
  digitalWrite(MOTOR_ENABLE_A, LOW);
}

void turnOnMotorsB() {
  digitalWrite(MOTOR_ENABLE_B, HIGH);
}

void turnOffMotorsB() {
  digitalWrite(MOTOR_ENABLE_B, LOW);
}

void turnOnMotors() {
  turnOnMotorsA();
  turnOnMotorsB();
  digitalWrite(TRANSISTOR_BASE, 255);
}

void turnOffMotors() {
  turnOffMotorsA();
  turnOffMotorsB();
  digitalWrite(TRANSISTOR_BASE, 0);
}

// Movement

void moveMotorsForwardA() {
  digitalWrite(MOTOR_A_1, HIGH);
  digitalWrite(MOTOR_A_2, LOW);
}

void moveMotorsBackwardA() {
  digitalWrite(MOTOR_A_1, LOW);
  digitalWrite(MOTOR_A_2, HIGH);
}

void moveMotorsForwardB() {
  digitalWrite(MOTOR_B_1, LOW);
  digitalWrite(MOTOR_B_2, HIGH);
}

void moveMotorsBackwardB() {
  digitalWrite(MOTOR_B_1, HIGH);
  digitalWrite(MOTOR_B_2, LOW);
}

void moveForward(short unsigned int duration) {
  moveMotorsForwardA();
  moveMotorsForwardB();
  delay(duration);
} 

void moveBackward(short unsigned int duration) {
  moveMotorsBackwardA();
  moveMotorsBackwardB();
  delay(duration);
} 

// Rotation

void rotateClockwise(short unsigned int duration) {
  moveMotorsForwardA();
  moveMotorsBackwardB();
  delay(duration);
}

void rotateCounterclockwise(short unsigned int duration) {
  moveMotorsBackwardA();
  moveMotorsForwardB();
  delay(duration);
}

// Stop

void stopMotorsA() {
  digitalWrite(MOTOR_A_1, HIGH);
  digitalWrite(MOTOR_A_2, HIGH);
}

void stopMotorsB() {
  digitalWrite(MOTOR_B_1, LOW);
  digitalWrite(MOTOR_B_2, LOW);
}

void stopMotors(short unsigned int duration) {
  stopMotorsA();
  stopMotorsB();
  delay(duration);
}

// DISCRETE MOVEMENT FUNCTIONS

void moveUp() {
  moveForward(MOVE_TIME);
  stopMotors(STOP_TIME);
}

void moveDown() {
  moveBackward(MOVE_TIME);
  stopMotors(STOP_TIME);
}

void moveLeft() {
  rotateCounterclockwise(ROTATE_TIME);
  stopMotors(STOP_TIME);
  moveForward(MOVE_TIME);
  stopMotors(STOP_TIME);
}

void moveRigth() {
  rotateClockwise(ROTATE_TIME);
  stopMotors(STOP_TIME);
  moveForward(MOVE_TIME);
  stopMotors(STOP_TIME);
}

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

// PERIPHEREAL FUNCTIONS

// Display

void displayText(String text) {
  LCD.print(text);
  delay(DISPLAY_TIME);
  LCD.clear();
}

// CAR FUNCTIONS

// Find checkpoint

int findCheckpointIndex(String color) {
  for (uint8_t i = 0; i < sizeof(checkpoints) / sizeof(checkpoints[0]); i++) {
    if (checkpoints[i] == color) {
      return i;  
    }
  }

  return -1;  
}

// SECTION FUNCTIONS

// Drive

void drive() {

}

// Ramp

void goDownRamp() {

}

// Cubes

void moveCubes() {

}

// Lines

void followLine() {

  short unsigned int left = digitalRead(LINE_LEFT);
  short unsigned int center = digitalRead(LINE_CENTER);
  short unsigned int right = digitalRead(LINE_RIGTH);

  if (center == 1 && left == 0 && right == 0) {
    // Función de avanzar
  }
  else if (left == 1 && center == 0 && right == 0) {
    // Función de girar a la izquierda (debe ser un giro infinito no marcado a pasos)
  }
  else if (right == 1 && center == 0 && left == 0) {
    // Función de girar a la derecha (debe ser un giro infinito no marcado a pasos)
  }
}

// SETUP

void setup() {

  Serial.begin(BAUD_RATE); // Open serial monitor at 115200 baud to see ping results

  // CONFIGURATION OF PIN MODES

  // Motors

  pinMode(MOTOR_ENABLE_A, OUTPUT);
  pinMode(MOTOR_A_1, OUTPUT);
  pinMode(MOTOR_A_2, OUTPUT);

  pinMode(MOTOR_ENABLE_B, OUTPUT);
  pinMode(MOTOR_B_1, OUTPUT);
  pinMode(MOTOR_B_2, OUTPUT);

  // Transistor

  pinMode(OUTPUT, TRANSISTOR_BASE);

  // START CAR

  turnOnMotors();
}

// LOOP

void loop() {
  String color = getColor();
  displayText(color);

  if (findCheckpointIndex(color) != -1) {
    checkpoint = findCheckpointIndex(color) + 1;
  }

  if (checkpoint == 0) {
    drive();
  }
  else if (checkpoint == 1) {
    goDownRamp();
  }
  else if (checkpoint == 2) {
    moveCubes();
  }
  else if (checkpoint == 3) {
    followLine();
  }
}