#include <Stepper.h>

// Signal from Jetson to tell the gear shifter to do something
#define doSomething 8

// Go forward and go backwards switches (will be replaced by signal from jetson
#define bearing 9

// Signal from Jetson to park the vehicle and put it in neutral
#define park 10

// Forward switch
#define fSwitch 27
// Reverse switch
#define rSwitch 26

// Number of steps from neutral to drive
int steps = 100;

// Relays to turn power on
int RELAY1 = 36;
int RELAY2 = 37;

// Status of the forward and reverse buttons
// These buttons are implemented so the cart knows
// if the transmission is set for forward or reverse
int fButtonState = LOW;
int rButtonState = LOW;

// Values that check which way the cart is going
bool forward = false;
bool backward = false;

Stepper stepper(steps,22,23,24,25);

void initialize(){
  digitalWrite(RELAY1,LOW);
  while(fButtonState != HIGH){
    fButtonState = digitalRead(fSwitch);
    stepper.step(3);
  }

  int count = 0;
  while(rButtonState != HIGH){
    rButtonState = digitalRead(rSwitch);
    stepper.step(-5);
    count += 5;
  }

  steps = count / 2;
  Serial.println(count);
  Serial.println(steps);

  stepper.step(steps);
  delay(3000);
}

void setup(){
  pinMode(fSwitch,INPUT);
  pinMode(rSwitch,INPUT);
  digitalWrite(fSwitch,LOW);
  digitalWrite(rSwitch,LOW);
  pinMode(RELAY1,OUTPUT);
  pinMode(RELAY2,OUTPUT);
  digitalWrite(RELAY2,HIGH);
  Serial.begin(9600);
  stepper.setSpeed(220);
  initialize();
  digitalWrite(RELAY1,HIGH);
}

/***********************************************************
 * Changes the gear shifter from one direction to the other
 * INPUT: Bearing recieved from the Jetson
 * OUTPUT: NONE
 **********************************************************/
void changeGear(int bear){
  //Move back "steps" to get to neutral
  // Move back "steps" to get to next gear
}

/***********************************************************
 * Once vehicle wants to park, put vehicle in neutral
 * INPUT: NONE
 * OUTPUT: NONE
 **********************************************************/
void parking(){
  //If in drive move back "steps" number of steps
  //If in reverse move back "steps" number of steps
}
int counter = 0;
void loop(){
  // Wait for signal from Jetson
  int sign = digitalRead(doSomething);
  
  if(sign == HIGH){
    // Once Arduino receives THE signal, it will read on what it needs to do
    // If "bearing" is HIGH, go forward
    // If "bearing" is LOW, go backward
    int way = digitalRead(bearing);
    changeGear(way);
  }
}



