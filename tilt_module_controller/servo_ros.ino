#include <Servo.h>

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>


Servo futaba_servo;  // create servo object to control a servo


int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin

//Function declaration
void servoAngleCB(const std_msgs::Int16& msg);


//ROS communications
ros::NodeHandle nh;
ros::Subscriber <std_msgs::Int16> servo_angle_sub("servo/angle", servoAngleCB);
std_msgs::Empty hb_msg;
ros::Publisher arduino_hb_pub("servo/hb", &hb_msg);

void setup() {
  nh.initNode();
  futaba_servo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  myservo.write(val);                  // sets the servo position according to the scaled value
  delay(15);                           // waits for the servo to get there
}
