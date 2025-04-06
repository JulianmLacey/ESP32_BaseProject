#include <debug.h>
#include <Arduino.h> // This is needed for the ESP32 to work with base Arduino libraries
#include "common.h"
#include "dc_motors.h"
#include "servo_motors.h"
#include "communications.h"
#include "stdint.h"

#define myID 5

RobotPose myPose;

int ballNum;

double d_x = 1000;
double d_y = 1000;
int16_t x = 0;
int16_t y = 0;
double theta, omega_1, omega_2, trueOmega_1, trueOmega_2;

double error_x = 0;
double  error_y = 0;
double  error_d = 0;
double prev_error_d = 0;
double error_theta = 0;
double prev_error_theta = 0;

double Kp1 = 1;
double Kd1 = 3;

double Kp2 = 9;
double Kd2 = 0.5;

void setup() {

  Serial.begin(115200);
  setupServos();
  setupCommunications();

  servo3.attach(SERVO2_PIN, 1300, 1700);
  servo4.attach(SERVO1_PIN, 1300, 1700);
  //servo2.attach(SERVO3_PIN);
  //servo3.writeMicroseconds(1500);
  //servo4.writeMicroseconds(1500);


}

void pickupBlock() {
  while (1) {
    Serial.println("Pickup block");

  }

}


void loop() {

  RobotPose myPose = getRobotPose(myID);

  //myPose.x = 0;
  //myPose.y = 1000;
  //myPose.theta = 190;
  //myPose.valid = true;

  if (myPose.valid == true) {
    x = myPose.x;
    y = myPose.y;
    theta = myPose.theta;
  } else {
    Serial.println("Invalid Pose");
  }

  error_x = d_x - x;
  error_y = d_y - y;

  error_d = sqrt(abs(error_x * error_x) + abs(error_y * error_y));
  error_theta = (atan2(error_y, error_x) - (theta / 1000)) * (180 / (PI)) - 90;

  if (error_theta < -180) {
    error_theta = error_theta + 360;
  } else if (error_theta > 180) {
    error_theta = error_theta - 360;
  }

  omega_1 = 0.2 * (Kp1 * error_d - Kp2 * error_theta + Kd1 * (error_d - prev_error_d) - Kd2 * (error_theta - prev_error_theta));
  omega_2 = 0.2 * (Kp1 * error_d + Kp2 * error_theta + Kd1 * (error_d - prev_error_d) + Kd2 * (error_theta - prev_error_theta));

  omega_1 = constrain(omega_1, -100, 100);
  omega_2 = constrain(omega_2, -100, 100);


  trueOmega_1 = map(omega_1, -100, 100, 1300, 1700);
  trueOmega_2 = map(-omega_2, -100, 100, 1300, 1700);

  servo3.writeMicroseconds(trueOmega_1);
  servo4.writeMicroseconds(trueOmega_2);


  Serial.print(omega_1);
  Serial.print(" ");
  Serial.print(omega_2);
  Serial.print(" ");
  Serial.print(error_d);
  Serial.print(" ");
  Serial.println(error_theta);


  if (error_d < 50) {
    servo3.writeMicroseconds(1500);
    servo4.writeMicroseconds(1500);
    delay(1000);
    pickupBlock();
  }


  // Serial.print(error_x);
  // Serial.print(" ");
  // Serial.print(error_y);
  // Serial.print(" ");
  // Serial.print(error_d);
  // Serial.print(" ");
  // Serial.println(error_theta);


  prev_error_d = error_d;
  prev_error_theta = error_theta;

}