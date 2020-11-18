/*
* if gearSwitchState == HIGH and reverseSwitchState == LOW : you are in DRIVE
* if gearSwitchState == HIGH and reverseSwitchState == HIGH : you are in REVERSE
*/

#include <Stepper.h>

int stp = 10200;
//int stp = 100000;

// Number of steps from neutral to drive
const int steps = 150;

Stepper stepper(steps,8,9,10,11);

void setup(){
  stepper.setSpeed(200);
  
}

void loop(){
  stepper.step(stp);
  delay(1000);
  stepper.step(-stp);
  delay(1000);
}




