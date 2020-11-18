byte PUL=7; //define Pulse pin
byte DIR=6; //define Direction pin

void setup() {
  pinMode (PUL, OUTPUT);
  pinMode (DIR, OUTPUT);

}

void loop() {
  for (int i=0; i<6400; i++)    //Forward 5000 steps
  {
    digitalWrite(DIR,LOW);
    digitalWrite(PUL,HIGH);
    delayMicroseconds(500);
    digitalWrite(PUL,LOW);
    delayMicroseconds(500);
  }
  for (int i=0; i<6400; i++)   //Backward 5000 steps
  {
    digitalWrite(DIR,HIGH);
    digitalWrite(PUL,HIGH);
    delayMicroseconds(500);
    digitalWrite(PUL,LOW);
    delayMicroseconds(500);
  }
}

