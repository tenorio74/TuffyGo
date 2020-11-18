// testing a stepper motor with a Pololu A4988 driver board or equivalent
// on an Uno the onboard led will flash with each step
// this version uses delay() to manage timing

//https://forum.arduino.cc/index.php?topic=307883.0

byte directionPin = 9;
byte stepPin = 8;
int numberOfSteps = 10000;
byte ledPin = 13;
int pulseWidthMicros = 10;  // microseconds
int microsbetweenSteps = 1; // milliseconds


void setup() {

  Serial.begin(9600);
  Serial.println("Starting StepperTest");
  digitalWrite(ledPin, LOW);
 
  delay(2000);

  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
 
 
  digitalWrite(directionPin, HIGH);
  for(int n = 0; n < numberOfSteps; n++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulseWidthMicros); // this line is probably unnecessary
    digitalWrite(stepPin, LOW);
   
    delay(microsbetweenSteps);
   
    digitalWrite(ledPin, !digitalRead(ledPin));
  }
 
  delay(3000);
 

  digitalWrite(directionPin, LOW);
  for(int n = 0; n < numberOfSteps; n++) {
    digitalWrite(stepPin, HIGH);
    // delayMicroseconds(pulseWidthMicros); // probably not needed
    digitalWrite(stepPin, LOW);
   
    delay(microsbetweenSteps);
   
    digitalWrite(ledPin, !digitalRead(ledPin));
  }
}

void readPotentiometer()
{
const byte potPin = A0;
int potValue;

potValue = analogRead(potPin);


 
 
}


void loop() {
 }
