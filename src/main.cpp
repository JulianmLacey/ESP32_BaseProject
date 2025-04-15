#define Motion

#define Debug
//#define Production

#include <Arduino.h> // This is needed for the ESP32 to work with base Arduino libraries
#include "common.h"

#include <debug.h>
#include "servo_motors.h"
#include "communications.h"
#include "stdint.h"
#include "MedianFilterLib.h"
#include "PID_v1.h"

#define myID 5 //Robot ID
#define TEAMCOL 3 //Team Cylinder Color
#define SPEEDLIMIT 100 //Speed Limit for the motors as a percent
#define HOMEX 220 //Home location
#define HOMEY 220 //Home location
#define BUFFLENGTH 5//Buffer Length for Median Filter
#define IRDISTBUFFLENGTH 20//Buffer Length for Median Filter
#define IRSENSORPIN 36  // IR Sensor Pin
#define SWEEPANGLE 90 // Sweep angle for the servo
#define NUMBALLS 3 // Number of balls in the field
#define pinSW 39 //limit switch pin
#define IRSENSORENABLE 16 // IR Sensor Enable Pin

//----------------------------------FUNCS----------------------------------
void setupLimitInterupt();
void IRAM_ATTR onSwitchInterupt();
void setupEncoderInterupt();
void IRAM_ATTR onEncoderTimer();
void setupPIDInterupt();
void IRAM_ATTR onPIDTimer();
void setupMotor();
void DCMotorCalibration();


void hunting();
void defending();
void capturing();
void goToHome();
int goToLoction();
double* transformRobotPos(uint16_t X, uint16_t Y);
void open(Servo* clawservo);
void close(Servo* clawservo);
void DCMotorCalibration();
//----------------------------------VARS----------------------------------

RobotPose myPose;
BallPosition currentBallPoss[ 20 ];
Servo clawServo;


int ballNum; //number of balls/cylinders in the field
double x = 0;  //x location of robot in pixels
double y = 0;  //y location of robot in pixels
double theta = 0; //rotation of robot in radians
double d_x = 0, d_y = 0; //x and y location of the target
double omega_1, omega_2, trueOmega_1, trueOmega_2;

double error_x = 0;
double error_y = 0;
double error_d = 0;
double error_theta = 0;

double prev_error_d = 0;
double prev_error_theta = 0;

const double Kp1 = 1;//1
const double Kd1 = 2.7;//1.1

const double Kp2 = 19;//12
const double Kd2 = 1.95;//1.5


static int IRSensorValue = 0; // IR Sensor Value

const int PWMfreq = 5000; // PWM frequency
const int PWMresolution = 12; // PWM resolution
const int maxDutyCycle = (int)(pow(2, PWMresolution) - 1);; // Max duty cycle for the PWM
int pidSampleTime = 10; // milliseconds

const int motCH1 = 4, motCH2 = 5; // Motor channels
const char enCHApin = 14, enCHBpin = 27;
const char motIN1pin = 13, motIN2pin = 12; // Motor IN pins
volatile long position = 0; // Encoder position
volatile bool lastEncA = 0, lastEncB = 0;
volatile bool newEncA = 0, newEncB = 0;
volatile bool error;
volatile bool isSwitchDown = false;

double input = 0, output = 0, setpoint = 60, angle = 0;
double kp = 135, ki = 16, kd = 0.68;//1.5;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

hw_timer_t* encoderTimer = NULL;
hw_timer_t* PIDtimer = NULL;
hw_timer_t* switchTimer = NULL;

MedianFilter<double>robotXFilter(BUFFLENGTH);
MedianFilter<double>robotYFilter(BUFFLENGTH);
MedianFilter<double>robotThetaFilter(BUFFLENGTH);
MedianFilter<double>IRSensorMagnitudeFilter(IRDISTBUFFLENGTH);
static RobotPose myPose;



//-------------------SETUP----------------------------------
void setup() {
  //---------General Setup---------
  Serial.begin(115200);
  setupServos();

#ifdef Production
  setupCommunications();
#define D_print(...)
#define D_write(...)
#define D_println(...)
#define D_printf(...)
#else
#define D_print(...)    Serial.print(__VA_ARGS__)
#define D_write(...)    Serial.print(__VA_ARGS__)
#define D_println(...)  Serial.println(__VA_ARGS__)
#define D_printf(...)  Serial.printf(__VA_ARGS__)
#define DEBUG_COMMUNICATIONS
#endif

#ifdef Motion
  setupMotor();
#endif


//---------Drive Servo Setup---------
  servo3.attach(SERVO2_PIN, 1300, 1700);
  servo4.attach(SERVO1_PIN, 1300, 1700);
  clawServo.attach(SERVO3_PIN);


  //---------DC Motor Setup---------
  pinMode(enCHApin, INPUT);
  pinMode(enCHBpin, INPUT);
  pinMode(pinSW, INPUT);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(pidSampleTime);
  myPID.SetOutputLimits(-4095, 4095);

  //---------Timer Interupts---------
  setupEncoderInterupt();
  setupPIDInterupt();
  setupLimitInterupt();

  //---------IR Sensor Setup---------
  pinMode(IRSENSORENABLE, OUTPUT);

  delay(1000);
}

/******************************************************Main Loop****************************************************** */
void loop() {
  hunting();
  capturing();
}


//-------------------STATES----------------------------------

void hunting() {

  BallPosition* closestBall = updateClosestCylinder();
  d_x = closestBall->x;
  d_y = closestBall->y;
  goToLocation(d_x, d_y);
  return();


}
void defending() {
  //fuck this shit
  //Center backs dont win the ballon d'Or
}
void capturing() {
  goToLocation(HOMEX, HOMEY);
}


int updateRobotPosition() {
  myPose = getRobotPose(myID);
  if (error_d > 0 && error_d < 200) {
    open(&clawServo);
  } else {
    close(&clawServo);
  }

  if (myPose.valid) {
    x = robotXFilter.AddValue(myPose.x);
    y = robotYFilter.AddValue(myPose.y);
    theta = robotThetaFilter.AddValue(myPose.theta);
    return 1;
  } else {
    Serial.println("Invalid Pose");
    return 0;
  }
}


int goToLocation(double dx, double dy) {

  bool atLocation = false;

  while (!atLocation) {
    error_x = dx - x;
    error_y = dy - y;
    error_d = sqrt(abs(error_x * error_x) + abs(error_y * error_y));
    error_theta = (atan2(error_y, error_x) - (theta / 1000)) * (180 / (PI)) - 90;

    if (error_theta < -180) {
      error_theta = error_theta + 360;
    } else if (error_theta > 180) {
      error_theta = error_theta - 360;
    }

    //PID controller for left and right motor
    omega_1 = 0.2 * (Kp1 * error_d - Kp2 * error_theta + Kd1 * (error_d - prev_error_d) - Kd2 * (error_theta - prev_error_theta));
    omega_2 = 0.2 * (Kp1 * error_d + Kp2 * error_theta + Kd1 * (error_d - prev_error_d) + Kd2 * (error_theta - prev_error_theta));

    //limit PID controller
    omega_1 = constrain(omega_1, -SPEEDLIMIT, SPEEDLIMIT);
    omega_2 = constrain(omega_2, -SPEEDLIMIT, SPEEDLIMIT);

    trueOmega_1 = map(omega_1, -100, 100, 1300, 1700);
    trueOmega_2 = map(-omega_2, -100, 100, 1300, 1700);

    //write the microseconds to the servos
    servo3.writeMicroseconds(trueOmega_1);
    servo4.writeMicroseconds(trueOmega_2);

    D_print(d_x);
    D_print(" ");
    D_print(d_y);
    D_print("   ");
    D_print(x);
    D_print(" ");
    D_print(y);
    D_print("   ");
    D_print(omega_1);
    D_print(" ");
    D_print(omega_2);
    D_print("   ");
    D_print(error_d);
    D_print("   ");
    D_println(error_theta);

    if (error_d < 150) {
      servo3.writeMicroseconds(1500);
      servo4.writeMicroseconds(1500);
      atLocation = true;
      return;
    }
    prev_error_d = error_d;
    prev_error_theta = error_theta;
  }
}

BallPosition* updateClosestCylinder() {
  static BallPosition closestBall;

  bool inReach = false;
  int ballnum = 0;

  while (inReach == false) {
    double minDist = INFINITY;

    getBallPositions(currentBallPoss);
    updateRobotPosition();
    for (int i = 0; i < NUMBALLS; i++) {
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

  }
  return &closestBall;
}

void huntingClosestBall() {
  BallPosition* closestBall = updateClosestCylinder();
  d_x = closestBall->x;
  d_y = closestBall->y;
  goToLocation(d_x, d_y);


}

void DCMotorCalibration() {

  D_print("Setpoint: ");    D_print(setpoint); D_print("  ");
  D_print("Measured : ");   D_print(input);    D_print("  ");
  D_print("PWM Output: ");  D_print(output);   D_print("  ");
  D_print("Angle: ");      D_print(angle);    D_print("  ");
  D_println("");

  delay(100);
}

void calibrateIRSensor() {
  digitalWrite(IRSENSORENABLE, HIGH);
  IRSensorValue = analogRead(IRSENSORPIN);
  Serial.println(IRSensorValue);
  delay(50);
}

void setupMotor() {
  pinMode(enCHApin, INPUT_PULLDOWN);
  pinMode(enCHBpin, INPUT_PULLDOWN);
  ledcSetup(motCH1, PWMfreq, PWMresolution);
  ledcSetup(motCH2, PWMfreq, PWMresolution);
  ledcAttachPin(motIN1pin, motCH1);
  ledcAttachPin(motIN2pin, motCH2);
  ledcWrite(motCH1, 0);
  ledcWrite(motCH2, 0);
}

void setupEncoderInterupt() {
  encoderTimer = timerBegin(0, 80, true);  // timer 0, prescalewr of 80 give 1 microsecond tiks
  timerAttachInterrupt(encoderTimer, &onEncoderTimer, true); // connect interrupt function to hardware with pointer
  timerAlarmWrite(encoderTimer, 10, true);  // 10 microsecond timer interrupt
  timerAlarmEnable(encoderTimer);
}
void IRAM_ATTR onEncoderTimer() {
  newEncA = digitalRead(enCHApin); //read encoder value
  newEncB = digitalRead(enCHBpin); //read encoder value
  position += (newEncA ^ lastEncB) - (lastEncA ^ newEncB); // determine new position from encoder readings
  if ((lastEncA ^ newEncA) & (lastEncB ^ newEncB)) {
    error = true;
  }
  lastEncA = newEncA;
  lastEncB = newEncB;

}

void setupPIDInterupt() {

  PIDtimer = timerBegin(1, 80, true);  // timer 1, prescalewr of 80 give 1 microsecond tiks
  timerAttachInterrupt(PIDtimer, &onPIDTimer, true); // connect interrupt function to hardware with pointer
  timerAlarmWrite(PIDtimer, 10000, true);  // 10 millisecond timer interrupt
  timerAlarmEnable(PIDtimer);

}

void IRAM_ATTR onPIDTimer() {

  input = position / (2940.0 / 360.0);
  //setpoint = (input < -60) || (input > 60) ? -setpoint : setpoint;
  myPID.Compute();
  if (output > 0) { // drive motor based off pid output
    ledcWrite(motCH1, abs(output));
    ledcWrite(motCH2, 0);
  } else {
    ledcWrite(motCH2, abs(output));
    ledcWrite(motCH1, 0);
  }

}

void setupLimitInterupt() {
  switchTimer = timerBegin(2, 80, true);  // timer 0, prescalewr of 80 give 1 microsecond tiks
  timerAttachInterrupt(switchTimer, &onSwitchInterupt, true); // connect interrupt function to hardware with pointer
  timerAlarmWrite(switchTimer, 10000, true);  // 10 microsecond timer interrupt
  timerAlarmEnable(switchTimer);

}
void IRAM_ATTR onSwitchInterupt() {
  isSwitchDown = digitalRead(pinSW);
  if (isSwitchDown == true) {
    Serial.println("Sum Behind u Dipshit");
  }
}

//Open the claw
void open(Servo* clawservo) {
  clawservo->write(20);  //Figure out OPEN AND CLOSE VALUES*****
  delay(15);
  return;
}

//Close the claw
void close(Servo* clawservo) {
  clawservo->write(85);
  delay(15);
  return;
}