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



const int brTrigPin = 13;
const int brEchoPin = 12;
const int blTrigPin = 9;
const int blEchoPin = 8;
const int bBrakeSignal = 5;

void setup()
{
  pinMode(brTrigPin, OUTPUT);
  pinMode(brEchoPin, INPUT);
  pinMode(blTrigPin, OUTPUT);
  pinMode(blEchoPin, INPUT);
  pinMode(bBrakeSignal, OUTPUT);
  Serial.begin(9600);  
}



int brDistance, blDistance;

void loop() {
  brDistance = sensor(brTrigPin, brEchoPin);
  blDistance = sensor(blTrigPin, blEchoPin);

    if ((brDistance < 200) && (brDistance != 0))
    {
      digitalWrite(bBrakeSignal, HIGH);
      Serial.print("Object Detected - Back Right - ");
      Serial.print(brDistance);
      Serial.print(" cm");
      Serial.println();
    }

    else if ((blDistance < 200) && (blDistance != 0))
    {
      digitalWrite(bBrakeSignal, HIGH);
      Serial.print("Object Detected - Back Left - ");
      Serial.print(blDistance);
      Serial.print( " cm");
      Serial.println();
    }

    else
    {
      digitalWrite(bBrakeSignal, LOW);
    }  
}
