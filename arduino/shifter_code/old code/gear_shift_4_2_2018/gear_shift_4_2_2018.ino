/*
* if gearSwitchState == HIGH and reverseSwitchState == LOW : you are in DRIVE
* if gearSwitchState == HIGH and reverseSwitchState == HIGH : you are in REVERSE
*/

#include <Stepper.h>

/*
 * "doSomething" indicates to the arduino that something needs to be done.
 */
int doSomething = 0;

/*********************************
 * directionStatus
 * 
 * directionStatus == 0 => Park
 * directionStatus == 1 => Drive
 *********************************/
int directionStatus = 0;

/*
 * forwardOrBack
 * 
 * forwardOrBack == 0 => Drive
 * forwardOrBack == 1 => Reverse
 */
int forwardOrBack = 0;

// Signal from Jetson to park the vehicle and put it in neutral
#define park 10

// Switch that turns on when the car is in gear
#define gearSwitch 28

// Switch that turns on when the car is in reverse
#define reverseSwitch 29

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
int gearSwitchState;
int reverseSwitchState;

// Values that check which way the cart is going
bool forward = false;
bool backward = false;

Stepper stepper(steps,22,23,24,25);

/***********************************************************
 * This is the first and last thing the cart will do.
 *  The purpose of that is to find the neutral position and 
 *  make sure cart is in neutral at startup
 **********************************************************/
void initialize(){
  digitalWrite(RELAY1,LOW);
  gearSwitchState = digitalRead(gearSwitch);
  reverseSwitchState = digitalRead(reverseSwitch);
  Serial.println("Initializing");
  while((gearSwitchState != 0) && (reverseSwitchState == 1)){
    gearSwitchState = digitalRead(gearSwitch);
    reverseSwitchState = digitalRead(reverseSwitch);
    stepper.step(3);
  }

  int count = 0;
  while(reverseSwitchState != 0){
    reverseSwitchState = digitalRead(reverseSwitch);
    gearSwitchState = digitalRead(gearSwitch);
    stepper.step(-5);
    count += 5;
  }

  steps = count / 2;
  Serial.println(count);
  Serial.println(steps);

  stepper.step(steps);
  delay(3000);
  Serial.println("Initialize Finished");
}

/***********************************************************
 * Changes the gear shifter from one direction to the other
 * INPUT: "forwardOrBack" indicates the direction of 
 *          movement desired.
 *        0 indicates a forward movement is requested
 *        1 indicates a backdard movement is requested
 * OUTPUT: NONE
 **********************************************************/
void changeGear(int forwardOrBack){
  digitalWrite(RELAY1,LOW);
  reverseSwitchState = digitalRead(reverseSwitch);
  gearSwitchState = digitalRead(gearSwitch);

  // Move back "steps" to get to next gear
  if(forwardOrBack == 1){
    if((gearSwitchState == 0) && (reverseSwitchState == 1)){
      Serial.println("Already in drive...");
    }
    else if(reverseSwitchState == 0){
      moveForward();
      moveForward();
    }
  }
  else if(forwardOrBack == 0){
    if(reverseSwitchState == 0){
      Serial.println("Already in reverse...");
    }
    else if((gearSwitchState == 1) && (reverseSwitchState == LOW)){
      moveBack();
      moveBack();
    }
  }
  digitalWrite(RELAY1,HIGH);
}

/***********************************************************
 * Once vehicle wants to park, put vehicle in neutral
 * INPUT: NONE
 * OUTPUT: NONE
 **********************************************************/
void parking(){
  reverseSwitchState = digitalRead(reverseSwitch);
  gearSwitchState = digitalRead(gearSwitch);
  //If in drive move back "steps" number of steps
  //If in reverse move back "steps" number of steps
  if((gearSwitchState == HIGH) && (reverseSwitchState == LOW)){
    moveBack();
  }
  else if((gearSwitchState == HIGH) && (reverseSwitchState == HIGH)){
    moveForward();
  }
}

/***********************************************************
 * Function to move the shifter from Neutral to Drive or
 *    from Reverse to Neutral
 * INPUT: NONE
 * OUTPUT: NONE
 **********************************************************/
void moveBack(){
  digitalWrite(RELAY1,LOW);
  stepper.step(steps);
  delay(3000);
  digitalWrite(RELAY1,HIGH);
}

/***********************************************************
 * Function to move the shifter from Drive to Neutral or
 *    from Neutral to Reverse
 * INPUT: NONE
 * OUTPUT: NONE
 **********************************************************/
void moveForward(){
  digitalWrite(RELAY1,LOW);
  stepper.step(-steps);
  delay(3000);
  digitalWrite(RELAY1,HIGH);
}

/***********************************************************
 *
 **********************************************************/
void setup(){
  pinMode(fSwitch,INPUT);
  pinMode(rSwitch,INPUT);
  pinMode(gearSwitch,INPUT);
  pinMode(reverseSwitch,INPUT);

  digitalWrite(fSwitch,LOW);
  digitalWrite(rSwitch,LOW);
  digitalWrite(gearSwitch,HIGH);
  digitalWrite(reverseSwitch,HIGH);

  pinMode(RELAY1,OUTPUT);
  pinMode(RELAY2,OUTPUT);
  digitalWrite(RELAY2,HIGH);
  Serial.begin(9600);
  stepper.setSpeed(220);
  initialize();
//  moveForward();
  digitalWrite(RELAY2,LOW);
  digitalWrite(RELAY1,HIGH);
}

void loop(){
  /*
   * Read the buttons and go in the direction requested.
   * If no button is pressed or if both buttons are pressed, go to neutral.
   */
  fButtonState = digitalRead(fSwitch);
  rButtonState = digitalRead(rSwitch);
  if((fButtonState == HIGH) && (rButtonState == LOW)){
    Serial.println("fbutton");
    directionStatus = 1;
    doSomething = 1;
  }
  else if((fButtonState == LOW) && (rButtonState == HIGH)){
    Serial.println("rbutton");
    directionStatus = 0;
    doSomething = 1;
  }
  else{
    doSomething = 0;
  }
  
  if(doSomething == 1){
    changeGear(directionStatus);
    digitalWrite(RELAY2,LOW);
  }
  else if(doSomething == 0){
//    parking();
  Serial.println("NOTHING");
    digitalWrite(RELAY2,HIGH);
  }
}




