/*
 * Controls the RC car: Steering servo and throttle motor
 *
 * Steering range: 70 = right limit, 100 = midpoint, 130 = left limit
 *    Servo range: 25* - 155*
 *
 * Throttle Range: 0 = Full Forward, 90 = Neutral, 180 = Full Reverse
 *
 * Harkishan Singh Grewal and King
 * Updated: February 13, 2018
 * February 1, 2018
 */

#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int8.h>

Servo steeringServo;    // create servo object to control steering servo
Servo throttleMotor;    // create servo object to control the throttleMotor
int throttleValue;
int brakeValue;
const int fTrigPin = 9;
const int fEchoPin = 8;
const int bTrigPin = 10;
const int bEchoPin = 11;

long duration;
int distance, fDistance, bDistance;

ros::NodeHandle nh;


void forward(const std_msgs::Int8& fwd)
{
  throttleValue = fwd.data;
  if (throttleValue != 90)
    {
      throttleMotor.write(throttleValue);   //0-89 = Forward; 91-180 = Reverse
    }
}



ros::Subscriber<std_msgs::Int8> sub("RosRCCarTest", &forward);

void setup() {
  Serial.begin(9600);   // Serial comm with 9600 baud rate
  steeringServo.attach(3);  // attach steering servo on pin 3, BOTH SERVOS MUST BE CONNECTED TO WORK
  throttleMotor.attach(5); // attach throttle motor on pin 5
  pinMode(fTrigPin, OUTPUT);
  pinMode(fEchoPin, INPUT);
  pinMode(bTrigPin, OUTPUT);
  pinMode(bEchoPin, OUTPUT);
  
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
//  throttleValue = 80;         //Slowest running value... I think
//  throttleMotor.write(throttleValue);
  Serial.print(throttleValue);
  Serial.println();
  fDistance = sensor (fTrigPin, fEchoPin);
//  if ((fDistance <50) && (fDistance != 0))
//  {
//    brake(throttleValue);
//    while ((fDistance < 50) && (fDistance != 0))
//    {
//      fDistance = sensor (fTrigPin, fEchoPin);
//      Serial.print(fDistance);
//      Serial.println();
//      delay(2000);
//    }
//  }
  nh.spinOnce();
  delay(1);
}

void brake(int value)            //To brake, give opposite value
{
  int temp;
  if(value > 90 && value < 180)
  {
    temp = value - 90;
    brakeValue = 90 - (temp * 0.8);
  }
  else if ( value > 0 && value < 90)
  {
    temp = 90 - value;
    brakeValue = 90 + temp;
  }

  throttleMotor.write(brakeValue);
  delay(150);
  throttleMotor.write(90);        //Keep neutral
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
