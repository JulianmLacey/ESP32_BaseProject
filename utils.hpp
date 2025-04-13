#pragma once

//#include "utils.cpp"
#include <debug.h>
#include <Arduino.h> // This is needed for the ESP32 to work with base Arduino libraries
#include "debug.h"
#include "common.h"
#include "dc_motors.h"
#include "servo_motors.h"
#include "communications.h"
#include "stdint.h"
#include "MedianFilterLib.h"
#include "PID_v1.h"



//----------------------------------FUNCS----------------------------------



double* transformRobotPos(uint16_t X, uint16_t Y);

void DCMotorCalibration();
void setupSwitchInterupt();
void onSwitchInterupt();
void setupEncoderInterupt(hw_timer_t** encoderTimer);
void IRAM_ATTR onEncoderTimer();

void IRAM_ATTR onPIDTimer(PID* mypid);
void setupPIDInterupt();
void setupDCMotor();

//----------------------------------VARS----------------------------------
extern const int PWMFreq = 500; //5KHz
extern const int PWMResolution = 12;
extern const int MAXDutyCycle = int(pow(2, PWMResolution) - 1);

extern const int PIDSampleTime = 10; //ms
extern const int MOT_CH_IN1 = 4;
extern const int MOT_CH_IN2 = 5;
extern const char ENC_CHANA_PIN = 14;
extern const char ENC_CHANB_PIN = 27;
extern const char MOT_IN1_PIN = 13;
extern const char MOT_IN2_PIN = 12;
extern volatile long position = 0;
extern volatile bool lastEncA = 0;
extern volatile bool lastEncB = 0;
extern volatile bool newEncA = 0;
extern volatile bool newEncB = 0;
extern volatile bool error;
extern double input = 0, output = 0, setpoint = 0;
extern double kp = 5.0, ki = 2.0, kd = 0.5;



