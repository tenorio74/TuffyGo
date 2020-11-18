#include <Stepper.h>

// Forward switch
#define fSwitch 8
// Reverse switch
#define rSwitch 9

// Number of steps from neutral to drive
int steps = 0;

// Status of the forward and reverse buttons
// These buttons are implemented so the cart knows
// if the transmission is set for forward or reverse
int fButtonState = LOW;
int rButtonState = LOW;

// Button debouncing
long lastDebounceTime = 0;
long debounceDelay = 50;

// Values that check which way the cart is going
bool forward = false;
bool backward = false;

Stepper stepper(steps, 4, 5, 6, 7);

/*
 * This function will initialise the gear shifter when the golf cart is turned on
 *  this will allow to make sure the stepper motor does not loose track of what
 *  gear the vehicle is in.
 * INPUT: none
 * OUTPUT: integer 
 *  1 = vehicle is ready to go
 *  2 = error occured do not move the golf cart
 */
int startupRoutine(void){
  // Move the gear shifter all the way to the forward switch
  if((millis() - lastDebounceTime) > debounceDelay){
    while(digitalRead(fSwitch != LOW)){
      stepper.step(2);
    }
    lastDebounceTime = millis();
  }

  // Move the gear shifter all the way to the reverse switch
  // Keep count of the steps until it gets there.
  int count = 0;
  if((millis() - lastDebounceTime) > debounceDelay){
    while(digitalRead(rSwitch != LOW)){
      stepper.step(-1);
      count += 1;
      Serial.println("testing");
    }
  }

  // Calculate the number of steps from neutral to reverse
  steps = count/2;

  // Number of steps should not exceed 20
  if(steps > 20){
    return 2;
  }

  // Move shifter to the neutral position
  stepper.step(steps);
  
  Serial.println("READY!");
  return 1;
}

void setup(){
  
  pinMode(fSwitch,INPUT);
  pinMode(rSwitch,INPUT);
  digitalWrite(fSwitch,HIGH);
  digitalWrite(rSwitch,HIGH);
  Serial.begin(9600);
  // Set the speed of the motor to 30RPMs
  stepper.setSpeed(60);
  int set = startupRoutine();
}

void loop(){
  forward = false;
  backward = false;
}



