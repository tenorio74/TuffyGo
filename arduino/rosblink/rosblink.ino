/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Int8.h>

ros::NodeHandle nh;

void toggle(int data){
  for (int i = 0 ; i < data; i++){
    digitalWrite(13,HIGH); // Turn led on
    delay(200);
    digitalWrite(13, LOW);  // Turn led off
    delay(200);
  }
}

void messageCb( const std_msgs::Int8& toggle_msg){
  if (toggle_msg.data == 0){
    digitalWrite(13, LOW);  // Turn led off
  } else if (toggle_msg.data == 1){
    digitalWrite(13,HIGH); // Turn led on
  } else {
    int temp = toggle_msg.data;
    toggle(temp);
  }
}

ros::Subscriber<std_msgs::Int8> sub("toggle_led", &messageCb );

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
