#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <Wire.h>
#include "FXAS21002C_Basic.h"
#include "FXOS8700Q_Basic.h"
#include "MadgwickAHRS.h"
#include "MS5837.h"

#define NO_OF_THRUSTERS 7
#define ACCELEROMETER_MAGNETOMETER_ADDRESS 0x1F
#define GYROSCOPE_ADDRESS 0x21
#define SDA_PIN PB7
#define SCL_PIN PB6
#define UPDATE_RATE 10
#define PUBLISH_RATE 1000

const uint8_t pinMap[NO_OF_THRUSTERS] = {PA0, PA1, PA2, PA3, PA6, PB0, PA7};
float accelerometer_readings[3];
float magnetometer_readings[3];
float gyroscope_readings[3];
float yaw, pitch, roll, depth;
int prev_time_update, prev_time_publish;
std_msgs::Float32MultiArray sensor_data;
void pwm_values_cb(const std_msgs::Int32MultiArray& pwm_values);

Servo thrusters[NO_OF_THRUSTERS];
ros::NodeHandle nh;
ros::Subscriber <std_msgs::Int32MultiArray> sub("pwm_values", &pwm_values_cb);
ros::Publisher pub("sensor_data", &sensor_data);

TwoWire wire(SDA_PIN, SCL_PIN);
FXAS21002CBasic Gyroscope;
FXOS8700QBasic Accelerometer_Magnetometer;
MS5837 Depth_Sensor;
Madgwick Filter;


void setup(){
  // Initializing ROS
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);

  // Initializing the thrusters
  for (int i = 0; i < NO_OF_THRUSTERS; i++){
    thrusters[i].attach(pinMap[i]);
    thrusters[i].writeMicroseconds(1472);
  }
  delay(2000);

  // Initializing I2C Interface
  wire.begin();
  Accelerometer_Magnetometer = FXOS8700QBasic(1, ACCELEROMETER_MAGNETOMETER_ADDRESS, &wire);
  Gyroscope = FXAS21002CBasic(GYROSCOPE_ADDRESS, &wire);
  Depth_Sensor.init(&wire);
  Depth_Sensor.setFluidDensity(997);

  prev_time_update = millis();
  prev_time_publish = millis();
}

void loop(){
  // Updating Sensor Data
  if(millis() - prev_time_update >= UPDATE_RATE){
    Depth_Sensor.read();
    Gyroscope.updateGyroData(gyroscope_readings);
    Accelerometer_Magnetometer.updateAccelMagData(accelerometer_readings, magnetometer_readings);
    Filter.update(gyroscope_readings[0], gyroscope_readings[1], gyroscope_readings[2], accelerometer_readings[0], accelerometer_readings[1], accelerometer_readings[2], magnetometer_readings[0], magnetometer_readings[1], magnetometer_readings[2]);
    yaw = Filter.getYaw();
    pitch = Filter.getPitch();
    roll = Filter.getRoll();
    depth = Depth_Sensor.depth();
    prev_time_update = millis();
  }
  // Publishing Sensor Data
  if(millis() - prev_time_publish >= PUBLISH_RATE){
    float values[4] = {yaw, pitch, roll, depth};
    sensor_data.data = values;
    sensor_data.data_length = 4;
    pub.publish(&sensor_data);
    prev_time_publish = millis();
  }
  nh.spinOnce();
}
// Setting Thruster Speed
void pwm_values_cb(const std_msgs::Int32MultiArray& pwm_values){
  for (int i = 0; i < NO_OF_THRUSTERS; i++){
    int pwm_value = pwm_values.data[i];
    thrusters[i].writeMicroseconds(pwm_value);
  }
}