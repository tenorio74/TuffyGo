
long duration;
int distance;

/*****************************************************************
Takes data from the sensors
INPUT: Echo and trig pin of the sensors
OUTPUT: Distance of detected object
*****************************************************************/
int sensor(int trigPin, int echoPin)
{
 digitalWrite(trigPin, LOW);
 delayMicroseconds(2);
 digitalWrite(trigPin, HIGH);
 delayMicroseconds(10);
 digitalWrite(trigPin,LOW);

 duration = pulseIn(echoPin, HIGH);

 distance = duration * 0.034/2;

 return distance;

}

const int fTrigPin = 11;
const int fEchoPin = 10;
const int frTrigPin = 13;
const int frEchoPin = 12;
const int flTrigPin = 9;
const int flEchoPin = 8;
const int brakeSignal = 5;

void setup()
{
  pinMode(fTrigPin, OUTPUT);
  pinMode(fEchoPin, INPUT);
  pinMode(frTrigPin, OUTPUT);
  pinMode(frEchoPin, INPUT);
  pinMode(flTrigPin, OUTPUT);
  pinMode(flEchoPin, INPUT);
  pinMode(brakeSignal, OUTPUT);
  Serial.begin(9600);  
}


int fDistance,frDistance, flDistance;

void loop() {
  frDistance = sensor (frTrigPin, frEchoPin);
  fDistance = sensor(fTrigPin, fEchoPin);
  flDistance = sensor(flTrigPin, flEchoPin);
  
    if ((frDistance < 200) && (frDistance != 0))  //Check if an object is detected by the front sensor
    {
      digitalWrite(brakeSignal, HIGH);
      Serial.print("Object Detected - Front Right - ");
      Serial.print(frDistance);
      Serial.print (" cm");
      Serial.println();
    }
    
    else if ((fDistance < 200) && (fDistance != 0))
    {
      digitalWrite(brakeSignal, HIGH);
      Serial.print("Object Detected - Front - ");
      Serial.print(fDistance);
      Serial.print(" cm");
      Serial.println();
    }

    else if ((flDistance < 200) && (flDistance != 0))
    {
      digitalWrite(brakeSignal, HIGH);
      Serial.print("Object Detected - Front Left - ");
      Serial.print(flDistance);
      Serial.print( " cm");
      Serial.println();
    }

    else
    {
      digitalWrite(brakeSignal, LOW);
    } 
}

