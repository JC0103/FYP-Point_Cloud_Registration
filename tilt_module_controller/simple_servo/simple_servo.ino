#include <Servo.h>

//#define servo 9

Servo myservo;  // create servo object to control a servo

int potpin = 0;  // analog pin used to connect the potentiometer
int val = ;    // variable to read the value from the analog pin
bool dir = false;

void setup() {
   myservo.attach(9);  // attaches the servo on pin 9 to the servo object
//  pinMode(servo, OUTPUT);
//  Serial.begin(9600);
//  analogWrite(servo, 0);
//  Serial.println(5);
}

void loop() {
  // val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  // val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
   myservo.write(val);                  // sets the servo position according to the scaled value
//  analogWrite(servo, 0);
//  delay(15);
//  Serial.println(val);
//  delay(50);                           // waits for the servo to get there
//  if (dir == false) val +=1;
//  else val-=1;
//  
//  if (val == 255) dir = true;
//  else if (val == 0) dir = false;
//  analogWrite(servo, 255);
//  Serial.println("2nd");
//  delay(2000);

}
