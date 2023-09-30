// INCLUDES:
// MOTORS
// DISPLAY

#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Arduino.h>

LiquidCrystal_I2C LCD(0x27, 16, 2);

// PINS SETUP

// Motors

#define MOTOR_ENABLE_A 3
#define MOTOR_A_1 2
#define MOTOR_A_2 4

#define MOTOR_ENABLE_B 5
#define MOTOR_B_1 6
#define MOTOR_B_2 7

// Transistor

#define TRANSISTOR_BASE 8

// CONSTANTS

// Baud rate

#define BAUD_RATE 115200

// DELAYS

// Movement

#define MOVE_TIME 900
#define ROTATE_TIME 500
#define STOP_TIME 900

// Display

#define DISPLAY_TIME 500

// Communication

#define COMMUNICATION_TIME 100

// GLOBAL VARIABLES

// Color

String color = "";

// Message

String receivedMessage = "";

// Checkpoint

short int checkpoint = 0;

// Distances

unsigned short int distanceA = 0, distanceB = 0, distanceC = 0;

// Lines

unsigned short int lineRigth = 0, lineCenter = 0, lineLeft = 0;

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

// PERIPHEREAL FUNCTIONS

// Display

void displayText(String text) {
  LCD.print(text);
  delay(DISPLAY_TIME);
  LCD.clear();
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
/* 
void followLine() {

  // LINE FOLLOWING SENSORS
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
 */
// SETUP

void setup() {

  Serial.begin(BAUD_RATE);

  // CONFIGURATION OF PIN MODES

  // Motors

  pinMode(MOTOR_ENABLE_A, OUTPUT);
  pinMode(MOTOR_A_1, OUTPUT);
  pinMode(MOTOR_A_2, OUTPUT);

  pinMode(MOTOR_ENABLE_B, OUTPUT);
  pinMode(MOTOR_B_1, OUTPUT);
  pinMode(MOTOR_B_2, OUTPUT);

  // Transistor

  pinMode(TRANSISTOR_BASE, OUTPUT);

  pinMode(8, OUTPUT);
}

// LOOP

void loop() { 

  while(Serial.available()) {
    
    char c = Serial.read();

    if(c == '\n') {

      short unsigned int receivedMessageSize = receivedMessage.length();
      String aux = "";
      short unsigned int counter = 0;

      for (short unsigned int i = 0; i < receivedMessageSize; i++) {

        if(receivedMessage[i] != ',') { 
          aux += receivedMessage[i];
        }
        else {
          if(counter == 0) {
            color = aux;
          }
          else if(counter == 1) {
            checkpoint = aux.toInt();
          }

          if(checkpoint ==  0) {
            if(counter == 2) {
              distanceA = aux.toInt();
            }
            else if(counter == 3) {
              distanceB = aux.toInt();
            }
            else {
              distanceC = aux.toInt();
            }
          }
          else if(checkpoint == 3) {
            if(counter == 2) {
              lineRigth = aux.toInt();
            }
            else if(counter == 3) {
              lineCenter = aux.toInt();
            }
            else {
              lineLeft = aux.toInt();
            }
          }

          aux = "";
          counter++;
        }
      }

      receivedMessage = ""; // Reiniciar el mensaje para la próxima lectura
    } 
    else {
      receivedMessage += c;
    }
  }

  delay(COMMUNICATION_TIME);
  Serial.println(color);
  Serial.println(distanceA);
  Serial.println(distanceB);
  Serial.println(distanceC);
}