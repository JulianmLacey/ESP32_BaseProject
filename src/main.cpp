#include <debug.h>
#include <Arduino.h> // This is needed for the ESP32 to work with base Arduino libraries
#include "common.h"
#include "dc_motors.h"
#include "servo_motors.h"
#include "communications.h"
#include "stdint.h"

#define myID 5
#define TEAMCOL 3

//----------------------------------FUNCS----------------------------------
int goToLoction();
void hunting();
void open();
void close();
void defending();
void capturing();

//----------------------------------VARS----------------------------------
/*
RobotPose: rotation, and location of the robot.
  .x = x location in pixels
  .y = y location in pixels
  .theta = rotation in radians
  .valid = true if the robot is in a valid location
*/
RobotPose myPose;
BallPosition currentBallPoss[ 20 ];

/*
clawServo: servo object for the robot claw
  .attach() = attaches the servo to the pin
  .write() = sets the position of the servo
*/
Servo clawServo;


int ballNum; //number of balls/cylinders in the field
int16_t x = 0;  //x location of robot in pixels
int16_t y = 0;  //y location of robot in pixels
double theta = 0; //rotation of robot in radians
/*
omega1 & omega2: speed of the left and right motors from -100 - 0 - 100 (100 = full speed forward, -100 = full speed backward)
  omega1 = speed of the left motor
  omega2 = speed of the right motor
  (to help understand the robot's movement)


  trueOmega1 & trueOmega2: speed of the left and right motors mapped to microseconds(1300 - 1700) for the motorshield
*/
double omega_1, omega_2, trueOmega_1, trueOmega_2;

/*
  Error x&y: the difference between the robot's location and the target location
  Error d: the distance between the robot and the target location
  Error theta: the difference between the robot's rotation and the vector between the robot and the target location
*/
double error_x = 0;
double  error_y = 0;
double  error_d = 0;
double error_theta = 0;


/*
  prev_error_d & prev_error_theta: the previous error values for the PID controller
  Kp1, Kd1, Kp2, Kd2: the PID controller constants
*/
double prev_error_d = 0;
double prev_error_theta = 0;

double Kp1 = 1;
double Kd1 = 3;

double Kp2 = 9;
double Kd2 = 0.5;


//-------------------SETUP----------------------------------
void setup() {

  Serial.begin(115200); //begin serial communication at 115200 baud rate
  setupServos();      //setup the servos
  setupCommunications();  //setup the communications with the camera

  //setup servos
  servo3.attach(SERVO2_PIN, 1300, 1700);
  servo4.attach(SERVO1_PIN, 1300, 1700);
  clawServo.attach(SERVO3_PIN);
}

//-------------------STATES----------------------------------

void hunting() {
  open();
  int ballnum = 0;
  BallPosition closestBall;

  double minDist = INFINITY;

  getBallPositions(currentBallPoss);

  for (int i = 0; i < 3; i++) {
    if (currentBallPoss[ i ].hue == TEAMCOL) {
      int dx = currentBallPoss[ i ].x - x;
      int dy = currentBallPoss[ i ].y - y;
      int dist = sqrt(dx * dx + dy * dy);
      if (dist < minDist) {
        minDist = dist;
        closestBall = currentBallPoss[ i ];
        ballnum = i;
      }
    }
  }


  goToLoction();
  //close();
  //goToLoction(260, 260);
  open();

  while (1) {

    Serial.println("finished");
  }

}

/*
  //go to the cylinder
goToLoction(closestBall.x, closestBall.y);
//pick up the cylinder
pickupBlock();
//go to home
goToLoction(0, 0); //SET HOME LOCATION
//open claw
open();
*/

void defending() {}
void capturing() {}


//Open the claw
void open() {
  clawServo.write(20);  //Figure out OPEN AND CLOSE VALUES*****
  delay(15);
  return;
}

//Close the claw
void close() {
  clawServo.write(85);
  delay(15);
  return;
}




void pickupBlock() {
  //open
  //go to location
  //close
  //go to home
  //open;


}



int goToLoction() {
  BallPosition closestBall;
  RobotPose myPose = getRobotPose(myID);
  bool inReach = false;
  int ballnum = 0;
  while (inReach == false) {
    double minDist = INFINITY;

    getBallPositions(currentBallPoss);
    myPose = getRobotPose(myID);
    for (int i = 0; i < 3; i++) {
      if (currentBallPoss[ i ].hue == TEAMCOL) {
        int dx = currentBallPoss[ i ].x - x;
        int dy = currentBallPoss[ i ].y - y;
        int dist = sqrt(dx * dx + dy * dy);
        if (dist < minDist) {
          minDist = dist;
          closestBall = currentBallPoss[ i ];
          ballnum = i;
        }
      }
    }
    //if valid pose, update x, y, and theta to new location
    if (myPose.valid == true) {
      x = myPose.x;
      y = myPose.y;
      theta = myPose.theta;
    } else {
      Serial.println("Invalid Pose");
      return 1;
    }

    int d_x = closestBall.x; //target x location
    int d_y = closestBall.y; //target y location

    //calculate difference between robot and target location
    error_x = d_x - x;
    error_y = d_y - y;

    //calculate distance between robot and target location
    error_d = sqrt(abs(error_x * error_x) + abs(error_y * error_y));
    error_theta = (atan2(error_y, error_x) - (theta / 1000)) * (180 / (PI)) - 90;

    //direction of rotation for the robot to spin
    if (error_theta < -180) {
      error_theta = error_theta + 360;
    } else if (error_theta > 180) {
      error_theta = error_theta - 360;
    }

    //PID controller for left and right motor
    omega_1 = 0.2 * (Kp1 * error_d - Kp2 * error_theta + Kd1 * (error_d - prev_error_d) - Kd2 * (error_theta - prev_error_theta));
    omega_2 = 0.2 * (Kp1 * error_d + Kp2 * error_theta + Kd1 * (error_d - prev_error_d) + Kd2 * (error_theta - prev_error_theta));

    //limit PID controller
    omega_1 = constrain(omega_1, -100, 100);
    omega_2 = constrain(omega_2, -100, 100);

    //map omega1 and omega2 to microseconds for the motorshield
    trueOmega_1 = map(omega_1, -100, 100, 1300, 1700);
    trueOmega_2 = map(-omega_2, -100, 100, 1300, 1700);

    //write the microseconds to the servos
    servo3.writeMicroseconds(trueOmega_1);
    servo4.writeMicroseconds(trueOmega_2);

    Serial.print(d_x);
    Serial.print(" ");
    Serial.print(d_y);
    Serial.print(" ");
    Serial.print(x);
    Serial.print(" ");
    Serial.print(y);
    Serial.print(" ");
    Serial.print(omega_1);
    Serial.print(" ");
    Serial.print(omega_2);
    Serial.print(" ");
    Serial.print(error_d);
    Serial.print(" ");
    Serial.println(error_theta);

    //if close enough to the target location, stop the motors
    if (error_d < 100) {
      servo3.writeMicroseconds(1500);
      servo4.writeMicroseconds(1500);
      delay(1000);
      close();
      inReach = true;
      while (1) {}
    }
    prev_error_d = error_d;
    prev_error_theta = error_theta;
  }
  return 0;
}


void loop() {

  //hunting();
    //defending();
    //capturing();

  goToLoction();
  //pickupBlock();
  //open();
  //delay(1000);
  //goToLoction(0, 1000);
  //pickupBlock();
  //open();
  //delay(1000);
  //goToLoction(0, -1000);
  //pickupBlock();
  //open();
  //delay(1000);

}