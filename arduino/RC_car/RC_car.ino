/*
 * Controls the RC car: Steering servo and throttle motor
 *
 * Steering range: 70 = right limit, 100 = midpoint, 130 = left limit
 *    Servo range: 25* - 155*
 *
 * Harkishan Singh Grewal
 * Updated: February 13, 2018
 * February 1, 2018
 */

#include <Servo.h>

Servo steeringServo;    // create servo object to control steering servo
Servo throttleMotor;    // create servo object to control the throttleMotor
int throttleValue;

void setup() {
  Serial.begin(9600);   // Serial comm with 9600 baud rate
  steeringServo.attach(3);  // attach steering servo on pin 3
  throttleMotor.attach(10); // attach throttle motor on pin 10
}

void loop() {
  // Map function to get correct value to send to throttleMotor
  throttleValue = (int) map(10, -1, 1, 0, 180);
  throttleMotor.write(throttleValue);
  delay (5000);
  throttleValue = (int) map(0, -1, 1, 0, 180);
  throttleMotor.write(throttleValue);
  steeringServo.write(70);
  delay(5000);
  steeringServo.write(130);
  delay(5000);
  steeringServo.write(100);
  delay(5000);
}
