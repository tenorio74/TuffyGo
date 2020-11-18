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
#define gearSwitch 29

// Switch that turns on when the car is in reverse
#define reverseSwitch 28

// Forward switch
#define fSwitch 27
// Reverse switch
#define rSwitch 26
//Neutral Switch
#define nSwitch 30
//Quit switch
#define qSwitch 32
//Demo switch
#define dSwitch 33
#define eSwitch 43

// Number of steps from neutral to drive
int steps = 150;
int pedalSteps = 20;
int bStep = 20;

// Relays to turn power on
int RELAY1 = 36;
int RELAY2 = 37;

// Status of the forward and reverse buttons
// These buttons are implemented so the cart knows
// if the transmission is set for forward or reverse
int fButtonState = LOW;
int rButtonState = LOW;
int nButtonState = LOW;
int gearSwitchState;
int reverseSwitchState;

// Values that check which way the cart is going
bool forward = false;
bool backward = false;

Stepper stepper(steps,22,23,24,25);
Stepper gasStepper(pedalSteps,38,39,40,41);
Stepper bStepper(bStep,44,45,46,47);

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
  while(reverseSwitchState != 0){
    gearSwitchState = digitalRead(gearSwitch);
    reverseSwitchState = digitalRead(reverseSwitch);
    stepper.step(3);
  }

  int count = 0;
  while(gearSwitchState != 0){
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
  if(forwardOrBack == 0){
    if(reverseSwitchState == 0){
      Serial.println("Already in reverse...");
      return;
    }
    while((reverseSwitchState != 0)){
    gearSwitchState = digitalRead(gearSwitch);
    reverseSwitchState = digitalRead(reverseSwitch);
    stepper.step(3);
    } 
    Serial.println("Ceaser Sucks");
  }
  else if(forwardOrBack == 1){
    if(gearSwitchState == 0){
      Serial.println("Already in drive...");
      return;
    }
    while(gearSwitchState != 0){
    gearSwitchState = digitalRead(gearSwitch);
    reverseSwitchState = digitalRead(reverseSwitch);
    stepper.step(-3);
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
  if(gearSwitchState == 0){
    Serial.println("test");
    moveBack();
  }
  else if(reverseSwitchState == 0){
    moveForward();
  }
  else{
    return;
  }
  //else initialize();
}

/***********************************************************
 * 
 **********************************************************/
void gasPedalPush(){
  gasStepper.step(9000);
}

void gasPedalRelease(){
  gasStepper.step(-9000);
}

/***********************************************************
 * Function to move the shifter from Neutral to Drive or
 *    from Reverse to Neutral
 * INPUT: NONE
 * OUTPUT: NONE
 **********************************************************/
void moveBack(){
  reverseSwitchState = digitalRead(reverseSwitch);

  if(reverseSwitchState == 0){
    Serial.println("Already Backward");
    return;
  }
  digitalWrite(RELAY1,LOW);
  stepper.step(steps);
  delay(1000);
  digitalWrite(RELAY1,HIGH);
}

/***********************************************************
 * Function to move the shifter from Drive to Neutral or
 *    from Neutral to Reverse
 * INPUT: NONE
 * OUTPUT: NONE
 **********************************************************/
void moveForward(){
  gearSwitchState = digitalRead(gearSwitch);

  if(gearSwitchState == 0){
    Serial.println("Already Forward");
    return;
  }
  else{
    digitalWrite(RELAY1,LOW);
    stepper.step(-steps);
    delay(1000);
    digitalWrite(RELAY1,HIGH);
  }
}

/***********************************************************
 * Function to check if button has been pressed
 * INPUT: NONE
 * OUTPUT: 
 *    0 = no button
 *    1 = Forward
 *    2 = Backward
 **********************************************************/
int checkButtons(){
  /*
   * Read the buttons and go in the direction requested.
   * If no button is pressed or if both buttons are pressed, go to neutral.
   */
  fButtonState = digitalRead(fSwitch);
  rButtonState = digitalRead(rSwitch);
  nButtonState = digitalRead(nSwitch);
  if((fButtonState == HIGH) && (rButtonState == LOW)){
    Serial.println("fbutton");
    // directionStatus = 1;
    // doSomething = 1;
    return 1;
  }
  else if((fButtonState == LOW) && (rButtonState == HIGH)){
    Serial.println("rbutton");
    // directionStatus = 0;
    // doSomething = 1;
    return 2;
  }
  else if(nButtonState == HIGH){
    return 3;
  }
  else{
    return 0;
  }
  
}

/***********************************************************
 *
 **********************************************************/
void setup(){
  pinMode(fSwitch,INPUT);
  pinMode(rSwitch,INPUT);
  pinMode(gearSwitch,INPUT);
  pinMode(reverseSwitch,INPUT);
  pinMode(qSwitch,INPUT);
  pinMode(dSwitch,INPUT);
  pinMode(eSwitch,INPUT);

  digitalWrite(fSwitch,LOW);
  digitalWrite(rSwitch,LOW);
  digitalWrite(qSwitch,LOW);
  digitalWrite(dSwitch,LOW);
  digitalWrite(eSwitch,LOW);
  digitalWrite(gearSwitch,HIGH);
  digitalWrite(reverseSwitch,HIGH);

  pinMode(RELAY1,OUTPUT);
  pinMode(RELAY2,OUTPUT);
  digitalWrite(RELAY2,HIGH);
  Serial.begin(9600);
  stepper.setSpeed(200);
  gasStepper.setSpeed(2000);
  bStepper.setSpeed(3000);
  initialize();
  bStepper.step(5000);
  bStepper.step(-5000);
//  moveForward();
  digitalWrite(RELAY2,HIGH);
  digitalWrite(RELAY1,HIGH);
}

void loop(){
  int action = checkButtons();

  int test = digitalRead(fSwitch);
  int test2 = digitalRead(dSwitch);
  int test3 = digitalRead(eSwitch);
  int quit = digitalRead(qSwitch);
  

  if(action == 1){
    changeGear(1);
  }
  else if(action == 2){
    changeGear(0);
  }
  else if(action == 3){
    parking();
  }
  //Testing cart with button to go
  if(quit == HIGH){
//    digitalWrite(RELAY2,LOW);
    Serial.println(RELAY2);
    gasStepper.step(9000); 
//    brakeStepper.step(-9000);
  }

  if(test3 == HIGH){
    gasStepper.step(-9000);
//    brakeStepper.step(9000);
  }

  if(test2 == HIGH){
    digitalWrite(RELAY2,LOW);
  }
  else{
    digitalWrite(RELAY2,HIGH);
  }
}




