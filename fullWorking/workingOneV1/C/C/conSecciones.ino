//With A Star Algorithm

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

#define PING_A 8 // Front
#define PING_B 9 // Rigth
#define PING_C 10 // Back
#define PING_D 11 // Left

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
String Color = "";  
int checkpoint = 0;  
String checkpoints[] = {"Cian", "Rosa Pastel", "Rosa"};


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

String getColor(){
  uint16_t R,G,B,C;
  String color = "Null";
  tcs.getRawData(&R, &G, &B, &C);
  
  if (R < 30 && G < 30 && B < 30){
    color = "Negro";
  }

  else if (R > 1.5 * G && R > 1.5 * B){
    color = "Rojo";
  }

  else if (G > R && G > B){
    color = "Verde";
  }

  else if (B > 1.5 * G && B > 1.5 * R){
    color = "Azul";
  }

  else if (R > 1.5 * B && G > 1.5 * B){
    color = "Amarillo";
  }

  else if (R > 1.5 * B && B > 1.5 * G){
    color = "Rosa";
  }

  else if (R > B && B > G){
    color = "Rosa Pastel"
  }

  else if (G > 1.5 * R && B > 1.5 * R){
    color = "Cian";
  }

  return color;
}

int findCheckpointIndex(String color) {
  for (int i = 0; i < sizeof(checkpoints) / sizeof(checkpoints[0]); i++) {
    if (checkpoints[i] == color) {
      return i;  
    }
  }
  return -1;  
}










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

/*   Serial.println(sonar[0].ping_cm());
  Serial.println(sonar[1].ping_cm());
  Serial.println(sonar[2].ping_cm()); */
  
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


void seguirLinea(){
  int left = digitalRead(2);
  int center = digitalRead(3);
  int right = digitalRead(4);
  if ( center == 1 && left == 0 && right == 0){
    // Función de avanzar
  }
  else if (left == 1 && center == 0 && right == 0){
    // Función de girar a la izquierda (debe ser un giro infinito no marcado a pasos)
  }
  else if (right == 1 && center == 0 && left == 0){
    // Función de girar a la derecha (debe ser un giro infinito no marcado a pasos)
  }
}




// SETUP

void setup() {
  
  // Open serial monitor at 115200 baud to see ping results

  Serial.begin(BAUD_RATE);
  //  Serial.begin(9600); Parametros utilizados para el sensor RGB


  // Configuration of pin modes



  // Pines utilizados para los seguidores de línea
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);

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

void loop(){
  color = getColor();
  if (findCheckpointIndex(color != -1)){
    checkpoint = findCheckpointIndex(color);
  }
  if (checkpoint == 0){
    drive();
  }
  else if (checkpoint == 0){
    // Función para bajar la rampa
  }
  else if (checkpoint == 1){
    // Función para segunda sección
  }
  else if (checkpoint == 2){
    // Función para seguir línea
  }
}
