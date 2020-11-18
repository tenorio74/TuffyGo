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

/***********************************************************
 * This is the first and last thing the cart will do.
 *  The purpose of that is to find the neutral position and 
 *  make sure cart is in neutral at startup
 **********************************************************/
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

/***********************************************************
 * Changes the gear shifter from one direction to the other
 * INPUT: "forwardOrBack" indicates the direction of 
 *          movement desired.
 *        0 indicates a forward movement is requested
 *        1 indicates a backdard movement is requested
 * OUTPUT: NONE
 **********************************************************/
void changeGear(int forwardOrBack){
  fButtonState = digitalRead(fSwitch);
  rButtonState = digitalRead(rSwitch);
  //Move back "steps" to get to neutral
  // Move back "steps" to get to next gear
  if(forwardOrBack == 0){
    if(fButtonState == HIGH){
      Serial.println("Already in drive...");
    }
    else if(rButtonState == HIGH){
      moveForward();
      moveForward();
    }
  }
  else if(forwardOrBack == 1){
    if(rButtonState == HIGH){
      Serial.println("Already in reverse...");
    }
    else if(fButtonState == HIGH){
      moveBack();
      moveBack();
    }
  }
}

/***********************************************************
 * Once vehicle wants to park, put vehicle in neutral
 * INPUT: NONE
 * OUTPUT: NONE
 **********************************************************/
void parking(){
  fButtonState = digitalRead(fSwitch);
  rButtonState = digitalRead(rSwitch);
  //If in drive move back "steps" number of steps
  //If in reverse move back "steps" number of steps
  if(fButtonState == HIGH){
    moveBack();
  }
  else if(rButtonState == HIGH){
    moveForward();
  }
}

/***********************************************************
 * Function to move the shifter from Neutral to Drive or
 *    from Reverse to Neutral
 * INPUT: NONE
 * OUTPUT: NONE
 **********************************************************/
void moveForward(){
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
void moveBack(){
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

void loop(){
  if(doSomething == 1){
    if(directionStatus == 1){
      parking();
      digitalWrite(RELAY2,HIGH);
    }
    else if(directionStatus == 0){
      changeGear(directionStatus);
      digitalWrite(RELAY2,LOW);
    }
  }
  
  /*
   * If the forward switch is high
   *   and reverse switch is low
   *   turn on relay 2
   */
//  int fButton = digitalRead(fSwitch);
//  int rButton = digitalRead(rSwitch);
//  if(fButton == HIGH && rButton == LOW){
//    digitalWrite(RELAY2,LOW);
//  }
//  else{
//    digitalWrite(RELAY2,HIGH);
//  }
}



