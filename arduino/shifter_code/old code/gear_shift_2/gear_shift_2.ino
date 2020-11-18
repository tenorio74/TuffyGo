#include <Stepper.h>

// Forward switch
#define fSwitch 12
// Reverse switch
#define rSwitch 11

// Number of steps from neutral to drive
int steps = 200;

// Status of the forward and reverse buttons
// These buttons are implemented so the cart knows
// if the transmission is set for forward or reverse
int fButtonState = LOW;
int rButtonState = LOW;

// Values that check which way the cart is going
bool forward = false;
bool backward = false;

Stepper stepper(steps,4,5,6,7);

void setup(){
  pinMode(fSwitch,INPUT);
  pinMode(rSwitch,INPUT);
  digitalWrite(fSwitch,LOW);
  digitalWrite(rSwitch,LOW);
  Serial.begin(9600);
  stepper.setSpeed(60);

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
}

void loop(){
//  stepper.step(-steps);
//  stepper.step(steps);
}

