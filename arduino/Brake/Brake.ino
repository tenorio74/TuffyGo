volatile int currentSteps;
int maxSteps;
int brakeSignalIn;
volatile int waitTime;


/************************************
 *INPUT: NONE
 *OUTPUT: NONE
 *Rotate the motor to press the brakes and set wait time to 5 seconds
 ************************************/
void brake()
{
  while (currentSteps < maxSteps)
  {
    //Write code to do 1 step;
    currentSteps++;
  }
  waitTime = 5;         /*Check if an object is not detected for 5 seconds*/
}


/************************************
 * INPUT: NONE
 * OUTPUT: NONE
 * Depress the brakes when nothing is detected for 5 seconds
 ************************************/
void depress()
{
  while((waitTime == 0) && (currentSteps > 0))
  {
    //Write code to do 1 reverse step;
    currentSteps--;
  }
}


void setup() 
{
  attachInterrupt(digitalPinToInterrupt(brakeSignalIn), brake, HIGH);
  currentSteps = 0;
//  write the max amount of steps below
//  maxSteps = ;

}

void loop() 
{
  while (waitTime > 0)     /*After the 5 seconds is up, depress the brakes*/
  {
    delay(1000);
    waitTime--;
  }
  if (waitTime == 0)
  {
    depress();
  }

}
