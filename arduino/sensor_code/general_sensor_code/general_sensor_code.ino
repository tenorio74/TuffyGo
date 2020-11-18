const int trigPin1 = 11;
const int echoPin1 = 10;
const int trigPin2 = 9;
const int echoPin2 = 8;
const int trigPin3 = 7;
const int echoPin3 = 6;

long duration;
int distance, distance1, distance2, distance3;

void setup() {
  // put your setup code here, to run once:
 pinMode(echoPin1, INPUT);
 pinMode(trigPin1, OUTPUT);
 pinMode(echoPin2, INPUT);
 pinMode(trigPin2, OUTPUT);
 pinMode(echoPin3, INPUT);
 pinMode(trigPin3, OUTPUT);
 Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
distance1 = sensor(trigPin1, echoPin1);
distance2 = sensor(trigPin2, echoPin2);
distance3 = sensor(trigPin3, echoPin3);

Serial.print("Right - ");
Serial.print(distance1);
Serial.print("Left - ");
Serial.print(distance2);
Serial.print("Back - ");
Serial.print(distance3);
Serial.println();
}

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

