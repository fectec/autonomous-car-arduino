// INCLUDES:
// MOTORS
// DISPLAY
// GYROSCOPE AND ACCELOREMETER

#include <LiquidCrystal_I2C.h>
#include <Wire.h>

LiquidCrystal_I2C LCD(0x3F, 16, 2);

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

// Motors

const int MPU = 0x68;

#define MAX_SPEED 255
#define MIN_SPEED 160

// Ultrasonic sensors

#define WALL_DISTANCE 10

// Baud rate

#define BAUD_RATE 115200

// DELAYS

// Movement

#define START_DELAY 8000
/* #define MOVE_TIME 900
#define ROTATE_TIME 500 */
#define STOP_TIME 2000

// Display

#define DISPLAY_TIME 1000

// Communication

#define COMMUNICATION_TIME 50

// GLOBAL VARIABLES

// Motors

float AccX = 0, AccY = 0, AccZ = 0;
float GyroX = 0, GyroY = 0, GyroZ = 0;
float accAngleX = 0, accAngleY = 0, gyroAngleX = 0, gyroAngleY = 0, gyroAngleZ = 0;
float roll = 0, pitch = 0, yaw = 0;
float AccErrorX = 0, AccErrorY = 0, GyroErrorX = 0, GyroErrorY = 0, GyroErrorZ = 0;
float elapsedTime = 0, currentTime = 0, previousTime = 0;
int c = 0;

float angle = 0;
float targetAngle = 0;
int equilibriumSpeed = 248;
int leftSpeedVal = 0;
int rightSpeedVal = 0;
bool isDriving = false; 
bool prevIsDriving = true;
bool paused = false;

// Color

String color = "";

// Message

String receivedMessage = "";

// Checkpoint

short int checkpoint = 0;

// Distances

unsigned short int distanceA = 0, distanceB = 0, distanceC = 0;

// Lines

unsigned short int lineRight = 0, lineCenter = 0, lineLeft = 0;

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

void moveForward(short unsigned int duration = 0) {
  moveMotorsForwardA();
  moveMotorsForwardB();
  delay(duration);
} 

void moveBackward(short unsigned int duration = 0) {
  moveMotorsBackwardA();
  moveMotorsBackwardB();
  delay(duration);
} 

// Rotation

void rotateClockwise(short unsigned int duration = 0) {
  moveMotorsForwardA();
  moveMotorsBackwardB();
  delay(duration);
}

void rotateCounterclockwise(short unsigned int duration = 0) {
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

void stopMotors(short unsigned int duration = 0) {
  stopMotorsA();
  stopMotorsB();
  delay(duration);
}

// PERIPHEREAL FUNCTIONS

// Display

void displayText(String text) {
  LCD.setCursor(0, 2);
  LCD.print(text);
  delay(DISPLAY_TIME);
  LCD.clear();
}

// SECTION FUNCTIONS

// Drive

void drive() {

  if (distanceA > 10) {
    isDriving = true;
  } 
  else if(distanceC > distanceB) {
    targetAngle += 90;
    if(targetAngle > 180) {
        targetAngle -= 360;
    }
      isDriving = false;
  } 
  else {
    targetAngle -= 90;
    if(targetAngle <= -180) {
        targetAngle += 360;
    }
    isDriving = false;
  }
}

// Ramp

void goDownRamp() {

  moveForward(5000);
  moveBackward(150000);
}

// Cubes

void moveCubes() {
  while(color != "GREEN") {
    drive();
  }

  while(color == "GREEN") {
    moveForward();
  }
}

// Lines

void followLine() {

  if (lineCenter == 1 && lineLeft == 0 && lineRight == 0) {
    moveForward();
  }
  else if (lineLeft == 1 && lineCenter == 0 && lineRight == 0) {
    rotateCounterclockwise();
  }
  else if (lineRight == 1 && lineCenter == 0 && lineLeft == 0) {
    rotateClockwise();
  }
}

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

  // CONFIGURATION OF GYROSCOPE ACCELOREMETER
  
  Wire.begin();        
  Wire.beginTransmission(MPU);     
  Wire.write(0x6B);              
  Wire.write(0x00);              
  Wire.endTransmission(true);     

  calculateError();
  delay(START_DELAY);

  currentTime = micros();

  // START CAR

  turnOnMotors();
}

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
              lineRight = aux.toInt();
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

      receivedMessage = "";
    } 
    else {
      receivedMessage += c;
    }
  }

  delay(COMMUNICATION_TIME);

/*   Serial.println(color);
  Serial.println(distanceA);
  Serial.println(distanceB);
  Serial.println(distanceC); */

  if(checkpoint == 0) {

    readAcceleration();

    accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX;
    accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;

    previousTime = currentTime;
    currentTime = micros();
    elapsedTime = (currentTime - previousTime) / 1000000;

    readGyro();

    GyroX -= GyroErrorX;
    GyroY -= GyroErrorY;
    GyroZ -= GyroErrorZ;

    gyroAngleX += GyroX * elapsedTime; 
    gyroAngleY += GyroY * elapsedTime;
    yaw += GyroZ * elapsedTime;

    roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
    pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
    angle = roll;

    Serial.println("---------------");
    Serial.println(roll);
    Serial.println(pitch);
    Serial.println(angle);

    drive();

    static int count;
    static int countStraight;

    if(count < 6) {  
      count ++;
    } 
    else { 
      count = 0;
      if(!paused) {
        if(isDriving != prevIsDriving) {
            leftSpeedVal = equilibriumSpeed;
            countStraight = 0;
        }
        if(isDriving) {
          if(abs(targetAngle - angle) < 3) {
            if(countStraight < 20) {
              countStraight ++;
            }
            else {
              countStraight = 0;
              equilibriumSpeed = leftSpeedVal; 
            }
          } 
          else {
            countStraight = 0;
          }
          driving();
        } 
        else {
          rotate();
        }
        prevIsDriving = isDriving;
      }
    }

    stopMotors(STOP_TIME);
  }
  else if(checkpoint == 1) {
    goDownRamp();
  }
  else if(checkpoint == 2) {
    moveCubes();
  }
  else if(checkpoint == 3) {
    followLine();
  }
}

void driving () {
  int deltaAngle = round(targetAngle - angle);
  moveForward();

  if(deltaAngle != 0) {
    controlSpeed();
    rightSpeedVal = MAX_SPEED;
    analogWrite(MOTOR_ENABLE_A, rightSpeedVal);
    analogWrite(MOTOR_ENABLE_B, leftSpeedVal);
  }
}

void controlSpeed (){
  int deltaAngle = round(targetAngle - angle);
  int targetGyroX;
  
  if(deltaAngle > 30) {
      targetGyroX = 60;
  } 
  else if(deltaAngle < -30) {
    targetGyroX = -60;
  } 
  else {
    targetGyroX = 2 * deltaAngle;
  }
  
  if(round(targetGyroX - GyroX) == 0) {
    ;
  }
  else if(targetGyroX > GyroX) {
    leftSpeedVal = changeSpeed(leftSpeedVal, -1);
  } 
  else {
    leftSpeedVal = changeSpeed(leftSpeedVal, +1);
  }
}

void rotate (){
  int deltaAngle = round(targetAngle - angle);
  int targetGyroX;

  if(abs(deltaAngle) <= 1) {
    stopMotors();
  } 
  else {
    if(angle > targetAngle) {
      rotateCounterclockwise();
    } 
    else if(angle < targetAngle) {
      rotateClockwise();
    }

    if(abs(deltaAngle) > 30) {
      targetGyroX = 60;
    }
    else {
      targetGyroX = 2 * abs(deltaAngle);
    }
    
    if(round(targetGyroX - abs(GyroX)) == 0) {
      ;
    }
    else if (targetGyroX > abs(GyroX)) {
      leftSpeedVal = changeSpeed(leftSpeedVal, +1);
    } 
    else {
      leftSpeedVal = changeSpeed(leftSpeedVal, -1);
    }

    rightSpeedVal = leftSpeedVal;
    analogWrite(MOTOR_ENABLE_A, rightSpeedVal);
    analogWrite(MOTOR_ENABLE_B, leftSpeedVal);
  }
}   

int changeSpeed (int motorSpeed, int increment){
  motorSpeed += increment;
  if(motorSpeed > MAX_SPEED) {
    motorSpeed = MAX_SPEED;
  }
  else if (motorSpeed < MIN_SPEED) {
    motorSpeed = MIN_SPEED;
  }
  return motorSpeed;
}

void calculateError() {
  c = 0;

  while (c < 200) {
    readAcceleration();

    AccErrorX += (atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI);
    AccErrorY += (atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI);
    c++;
  }

  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  
  c = 0;
  
  while(c < 200) {
    readGyro();

    GyroErrorX += GyroX;
    GyroErrorY += GyroY;
    GyroErrorZ += GyroZ;
    c++;
  }

  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
}

void readAcceleration() {

  while(!Serial.available()) {
    ;
  }

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); 

  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
}

void readGyro() {
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);

  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
}