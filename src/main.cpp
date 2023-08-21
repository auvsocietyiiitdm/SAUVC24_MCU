#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

#define NO_OF_THRUSTERS 7

const uint8_t pinMap[NO_OF_THRUSTERS] = {PA0, PA1, PA2, PA3, PA6, PA7, PB0};

void pwm_values_cb(const std_msgs::Int32MultiArray& pwm_values);

Servo thrusters[NO_OF_THRUSTERS];
ros::NodeHandle nh;
ros::Subscriber <std_msgs::Int32MultiArray> sub("pwm_values", &pwm_values_cb);

void setup(){
  // Initializing the thrusters
  for (int i = 0; i <= NO_OF_THRUSTERS; i++){
    thrusters[i].attach(pinMap[i]);
    thrusters[i].writeMicroseconds(1500);
  }
  delay(2000);

  // Initializing ROS
  nh.initNode();
  nh.subscribe(sub);

}

void loop(){
  nh.spinOnce();
}

void pwm_values_cb(const std_msgs::Int32MultiArray& pwm_values){

  for (int i = 0; i <= NO_OF_THRUSTERS; i++){
    int pwm_value = pwm_values.data[i];
    thrusters[i].writeMicroseconds(pwm_value);
  }
}