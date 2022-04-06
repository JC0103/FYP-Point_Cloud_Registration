#include <Servo.h>

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>


Servo L_servo, R_servo;  // create servo object to control a servo


int angle; // servo motor angle
const int min_angle = 0;
const int max_angle = 180;
unsigned long previous_write_time = -999;

// For callibration
const int L_zeros_angle = 37;
const int R_zeros_angle = 140;

//Function declaration
void servoAngleCB(const std_msgs::Int16& msg);


//ROS communications
ros::NodeHandle nh;
ros::Subscriber <std_msgs::Int16> servo_angle_sub("multiastra_subscriber1/servo/angle", &servoAngleCB);
std_msgs::Empty hb_msg;
ros::Publisher arduino_hb_pub("servo/hb", &hb_msg);

void setup() {
  Serial.begin(115200);
  nh.initNode();  //initialize ros node handler
  L_servo.attach(9);  // attaches the servos on pin 9, 10 to the servo object
  R_servo.attach(10);
  nh.subscribe(servo_angle_sub);
  nh.advertise(arduino_hb_pub);
}

void loop() {
  // waits for the servo to get there (0.22 sec/60°at 4.8V)
  if (abs(millis() - previous_write_time) > 600){
    arduino_hb_pub.publish(&hb_msg);
    L_servo.write(-angle + L_zeros_angle);                  // sets the servo position according to the scaled value
    R_servo.write(R_zeros_angle + angle);
  }                      
  nh.spinOnce();
  delay(3);
}

void servoAngleCB(const std_msgs::Int16& msg){
    if ((msg.data + L_zeros_angle < min_angle && msg.data + L_zeros_angle > max_angle) ||
    (R_zeros_angle - msg.data < min_angle &&  R_zeros_angle - msg.data > max_angle)){
        nh.logwarn("The input angle is out of range");
    }
    else angle = msg.data;
}