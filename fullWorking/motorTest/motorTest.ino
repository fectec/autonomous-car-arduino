// A

#define enableA 3
#define pinA1 2
#define pinA2 4

// B

#define enableB 5
#define pinB1 6
#define pinB2 7

void setup() {
  // put your setup code here, to run once:
    pinMode(enableA, OUTPUT);
  pinMode(pinA1, OUTPUT);
  pinMode(pinA2, OUTPUT);

      pinMode(enableB, OUTPUT);
  pinMode(pinB1, OUTPUT);
  pinMode(pinB2, OUTPUT);

}

void loop() {
  digitalWrite(enableA, 255);
  digitalWrite(pinA1, HIGH);
  digitalWrite(pinA2, LOW);

    digitalWrite(enableB, 255);
  digitalWrite(pinB1, HIGH);
  digitalWrite(pinB2, LOW);

}
