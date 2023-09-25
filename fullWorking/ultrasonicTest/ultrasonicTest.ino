// Ultrasonic sensors 

#define trigPin 8 

// 1

#define echoPin1 9 // Enfrente
#define echoPin2 10 // 
#define echoPin3 11 

double getDistance(unsigned short int triggerPin, unsigned short int echoPin) {
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(1000);
  digitalWrite(triggerPin, LOW);
  
  return pulseIn(echoPin, HIGH) / 58.2;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(echoPin2, INPUT);
  pinMode(echoPin3, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  double distance = getDistance(trigPin, echoPin1);
  Serial.println(distance);
}
