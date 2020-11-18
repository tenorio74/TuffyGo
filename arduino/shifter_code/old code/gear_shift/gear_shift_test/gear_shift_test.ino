#include <Stepper.h>

// Forward switch
#define fSwitch 12
// Reverse switch
#define rSwitch 11

// Number of steps from neutral to drive
int steps = 0;

// Status of the forward and reverse buttons
// These buttons are implemented so the cart knows
// if the transmission is set for forward or reverse
int fButtonState = LOW;
int rButtonState = LOW;

// Button debouncing
long lastDebounceTime = 0;
long debounceDelay = 100;

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
  while(fButtonState != HIGH){
    Serial.println("Going forward");
    fButtonState = digitalRead(fSwitch);
    stepper.step(3);
  }
  
  // Move the gear shifter all the way to the reverse switch
  // Keep count of the steps until it gets there.
  int count = 0;
  while(rButtonState != HIGH){
    Serial.println("Going backwards");
    rButtonState = digitalRead(rSwitch);
    stepper.step(-3);
    count += 1;
  }
  Serial.println(count);
  // Calculate the number of steps from neutral to reverse
  steps = count/2;
  Serial.println(steps);

  // Number of steps should not exceed 20
//  if(steps > 20){
//    return 1;
//  }

  // Move shifter to the neutral position
  int test = 0;
  while(test == 1){
    Serial.println("blah");
    stepper.step(steps);
    test += 1;
  }

  Serial.println(steps);
  Serial.println("READY!");
  return 0;
}

void setup(){
  
  pinMode(fSwitch,INPUT);
  pinMode(rSwitch,INPUT);
  digitalWrite(fSwitch,LOW);
  digitalWrite(rSwitch,LOW);
  Serial.begin(9600);
  // Set the speed of the motor to 30RPMs
  stepper.setSpeed(60);
  int set = startupRoutine();
}

void loop(){
  forward = false;
  backward = false;
  delayMicroseconds(5000); 
  stepper.step(-steps);
}



