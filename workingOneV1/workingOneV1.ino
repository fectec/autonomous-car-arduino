#include <NewPing.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_TCS34725.h>
 
LiquidCrystal_I2C LCD(0x27, 16, 2);
Adafruit_TCS34725 TCS = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

// PINS SETUP

// Motors

#define MOTOR_ENABLE_A 3
#define MOTOR_A_1 2
#define MOTOR_A_2 4

#define MOTOR_ENABLE_B 5
#define MOTOR_B_1 6
#define MOTOR_B_2 7

// Ultrasonic sensors 

#define PING_A 8
#define PING_B 9
#define PING_C 10
#define PING_D 11

// CONSTANTS

// Ultrasonic sensors

#define SONAR_NUM 4
#define MAX_DISTANCE 200
#define WALL_DISTANCE 10

// Delays

#define FORWARD_TIME 700 
#define ROTATE_TIME 700
#define DISPLAY_TIME 500

// Baud rate

#define BAUD_RATE 115200

// GLOBAL VARIABLES

// Ultrasonic sensors

NewPing sonar[SONAR_NUM] = {
  NewPing(PING_A, PING_A, MAX_DISTANCE),
  NewPing(PING_B, PING_B, MAX_DISTANCE),
  NewPing(PING_C, PING_C, MAX_DISTANCE),
  NewPing(PING_D, PING_D, MAX_DISTANCE)
};

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
}

void turnOffMotors() {
  turnOffMotorsA();
  turnOffMotorsB();
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
  digitalWrite (MOTOR_A_1, HIGH);
  digitalWrite (MOTOR_A_2, HIGH);
}

void stopMotorsB() {
  digitalWrite (MOTOR_B_1, LOW);
  digitalWrite (MOTOR_B_2, LOW);
}

void stopMotors(short unsigned int duration) {
  stopMotorsA();
  stopMotorsB();
  delay(duration);
}

// SENSOR FUNCTIONS

// Color sensor

/* String getColor() {

  return color;
} */

// PERIPHEREAL FUNCTIONS

// Display

/* void displayText(String text) {
  LCD.print(text);
  delay(DISPLAY_TIME);
  LCD.clear();

} */

// CAR FUNCTIONS

// Drive

void drive() {

  if(sonar[1].ping_cm() > WALL_DISTANCE) {
    rotateClockwise(ROTATE_TIME);
  }
  else if(sonar[0].ping_cm() > WALL_DISTANCE) {
    moveForward(FORWARD_TIME);
  }
  else {
    rotateCounterclockwise(ROTATE_TIME);
  }
}

/* void displayColor() {
  displayText(getColor());
}  */

// SETUP

void setup() {
  
  // Open serial monitor at 115200 baud to see ping results

  Serial.begin(BAUD_RATE);

  // Configuration of pin modes

  // Motors

  pinMode(MOTOR_ENABLE_A, OUTPUT);
  pinMode(MOTOR_A_1, OUTPUT);
  pinMode(MOTOR_A_2, OUTPUT);

  pinMode(MOTOR_ENABLE_B, OUTPUT);
  pinMode(MOTOR_B_1, OUTPUT);
  pinMode(MOTOR_B_2, OUTPUT);

  // Start car

  turnOnMotors();
}

// LOOP

void loop() {
  drive();
}