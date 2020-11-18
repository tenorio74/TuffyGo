#include <ros.h>

// #include "geometry_msgs/Twist.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

const int DELAY = 100;
ros::NodeHandle n;

// Cart odom configuration
const float ODOM_RADIUS = 0.406;      // Radius of wheels
const int ODOM_TICK = 600;            // Number of ticks for encoder
const float ODOM_LEN = 1.219;         // Distance between two wheels
const float ODOM_DT = DELAY/1000.0;   // Delta time
float odom_x_vel = 0.0;

volatile int odom_counterL = 0;       // Measure rotation of left encoder
volatile int odom_counterR = 0;       // Measure rotation of right encoder
int odom_last_counterL = 0;           // Prev value of left encoder
int odom_last_counterR = 0;           // Prev value of right encoder

const char odom_base_link_path[] = "/base_link";  // Base topic
const char odom_path[] = "/odom";                 // odom topic

// Transform movements
geometry_msgs::TransformStamped odom_trans;
tf::TransformBroadcaster odom_broadcaster;

double odom_x = 0.0;                  // Initial X position
double odom_y = 0.0;                  // Initial Y position
double odom_theta = 0.0;              // Initial Theta angle

// Interrupt for Left Encoder
void odom_ISR0()
{
  // ai0 is activated if DigitalPin 2 goes from LOW to HIGH
  // Check pin 4 to determine the direction
  if ( digitalRead(4) == LOW )
    odom_counterL++;
  else
    odom_counterL--;
}

// Interrupt for Right Encoder
void odom_ISR1()
{
  // ai0 is activated if DigitalPin 3 is going from LOW to HIGH
  // Check with pin 5 to determine the direction
  if ( digitalRead(5) == LOW )
    odom_counterR--;
  else
    odom_counterR++;
}

// Process odom data before sending to ROS
void odom_process()
{
  // Temp variabiles to use for calculations
  const int temp_counterL = odom_counterL;
  const int temp_counterR = odom_counterR;

  // Calculate change since last value
  const int deltaL = temp_counterL - odom_last_counterL;
  const int deltaR = temp_counterR - odom_last_counterR;

  // Distance calculations
  // Dl = 2*PI*R*(lefttick/totaltick)
  const float distL = 2*M_PI*ODOM_RADIUS*(deltaL/(float)ODOM_TICK);
  // Dr = 2*PI*R*(righttick/totaltick)
  const float distR = 2*M_PI*ODOM_RADIUS*(deltaR/(float)ODOM_TICK);
  const float dC = (distL + distR) / 2;
  odom_x_vel = dC / ODOM_DT;

  const float dtheta = (distR - distL) / ODOM_LEN;
  const float dx = dC * cos(odom_theta);
  const float dy = dC * sin(odom_theta);

  odom_theta += dtheta;   // calculates new theta angle based on encoder values
  odom_x += dx;           // calculates new X position based on wheel revolution
  odom_y += dy;           // calculates new Y position based on wheel revolution

  // publish the transform over tf
  odom_trans.header.stamp = n.now();
  odom_trans.header.frame_id = odom_path;  // odom data publishes on Odom topic
  odom_trans.child_frame_id = odom_base_link_path;

  odom_trans.transform.translation.x = odom_x;
  odom_trans.transform.translation.y = odom_y;
  odom_trans.transform.translation.z = 0;
  // converting from euler angle to quaternion form
  odom_trans.transform.rotation = tf::createQuaternionFromYaw(odom_theta);

  // send the transform
  odom_broadcaster.sendTransform(odom_trans);   // broadcasting updated result

  // Update last readings
  odom_last_counterL = temp_counterL;
  odom_last_counterR = temp_counterR;
}


void setup()
{
  n.getHardware()->setBaud(115200);
  n.initNode();         // Initializing node handler

  // Call initialize function
  odom_broadcaster.init(n);           // Initialize odom data broadcaster

  // Left encoder input
  pinMode(2, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);

  // attachInterrupt(digitalPinToInterrupt(pin), ISR, mode);
  attachInterrupt(digitalPinToInterrupt(2), odom_ISR0, RISING);

  // Right encoder input
  pinMode(3, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3), odom_ISR1, RISING);
}

void loop()
{
  // Call process function
  odom_process();

  // Sync with ROS
  n.spinOnce();
  delay(DELAY);
}
