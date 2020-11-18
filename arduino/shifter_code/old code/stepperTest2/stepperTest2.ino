#include <Stepper.h>

#define fSwitch 8
#define rSwitch 9
#define STEPS 50

int fButtonState = LOW;
int rButtonState = LOW;
long lastDebounceTime = 0;
long debounceDelay = 50;
bool forward = false;
bool backward = false;

Stepper stepper(STEPS, 4, 5, 6, 7);

void setup() {
  pinMode(fSwitch,INPUT);

  pinMode(rSwitch,INPUT);
  digitalWrite(fSwitch,HIGH);
  digitalWrite(rSwitch,HIGH);
  Serial.begin(9600);
  Serial.println("Stepper test!");    
  // set the speed of the motor to 30 RPMs
  stepper.setSpeed(60);
}

void loop() {
  forward = false;
  backward = false;
  //Sample the state of the button - is it pressed or not?
  fButtonState = digitalRead(fSwitch);
  rButtonState = digitalRead(rSwitch);
  
  //filter out any noice by setting a time buffer
  if((millis() - lastDebounceTime) > debounceDelay){
    if((fButtonState == LOW) && (forward == false)){
      forward = true;
      backward = false;
      lastDebounceTime = millis();
    }
    else if((rButtonState == LOW) && (backward == false)){
      backward = true;
      forward = false;
      lastDebounceTime = millis();
    }
    else if((fButtonState == LOW) && (forward == true)){
      forward = false;
      lastDebounceTime = millis();
    }
    else if((rButtonState == LOW) && (backward == true)){
      backward = false;
      lastDebounceTime = millis();
    }
  }
  Serial.println(forward);
  Serial.println(backward);
  if(forward == backward == false){
    stepper.step(0);
  }
  if(forward){
    stepper.step(STEPS);
  }
//  else{
//    digitalWrite(fRelay,HIGH);
//  }
  if(backward){
    stepper.step(-STEPS);
  }
//  else{
//    digitalWrite(rRelay,HIGH);
//  }
}

