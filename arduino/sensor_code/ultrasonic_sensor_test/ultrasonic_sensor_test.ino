/*

*/

#define trigPin 12
#define echoPin 13
#define echoPin2 11
#define trigPin2 10


void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);  
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT); 
}

void loop() {
  long duration1, distance1;
  long duration2, distance2;
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration1 = pulseIn(echoPin, HIGH);
  distance1 = (duration1/2)/29.1;
  Serial.print(distance1);
  Serial.println(" cm sensor 1");
 
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  distance2 = (duration2/2)/29.1;
  Serial.print(distance2);
  Serial.println(" cm sensor 2");
 
  delay(500);
  

}

