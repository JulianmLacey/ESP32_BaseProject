#include "utils.hpp"
#include <debug.h>
#include <Arduino.h> // This is needed for the ESP32 to work with base Arduino libraries
#include "debug.h"
#include "common.h"





void DCMotorCalibration() {
    if (Serial.available() > 0) {
        int incomingByte = Serial.read();
        if (incomingByte == 'a') {
            setpoint += 300; // ticks
        } else if (incomingByte == 'z') {
            setpoint -= 300; // ticks
        }
    }
    D_print("Setpoint: ");    D_print(setpoint); D_print(" ");
    D_print("Measured : ");   D_print(input);    D_print(" ");
    D_print("PWM Output: ");  D_print(output);   D_print(" ");
    D_println("");

    delay(100);
}

void setupSwitchInterupt() {


}
void onSwitchInterupt() {


}
void setupEncoderInterupt(hw_timer_t** encoderTimer) {
    *encoderTimer = timerBegin(0, 80, true);  // timer 0, prescalewr of 80 give 1 microsecond tiks
    timerAttachInterrupt(*encoderTimer, &onEncoderTimer, true); // connect interrupt function to hardware with pointer
    timerAlarmWrite(*encoderTimer, 10, true);  // 10 microsecond timer interrupt
    timerAlarmEnable(*encoderTimer);
}
void IRAM_ATTR onEncoderTimer() {
    newEncA = digitalRead(ENC_CHANA_PIN); //read encoder value
    newEncB = digitalRead(ENC_CHANB_PIN); //read encoder value
    position += (newEncA ^ lastEncB) - (lastEncA ^ newEncB); // determine new position from encoder readings
    if ((lastEncA ^ newEncA) & (lastEncB ^ newEncB)) {
        error = true;
    }
    lastEncA = newEncA;
    lastEncB = newEncB;

}

void setupPIDInterupt(hw_timer_t** pidTimer) {
    //auto pidFunc = std::bind(&onPIDTimer, *pidTimer);
    *pidTimer = timerBegin(1, 80, true);  // timer 1, prescalewr of 80 give 1 microsecond tiks
    timerAttachInterrupt(*pidTimer, &onPIDTimer(mypid), true); // connect interrupt function to hardware with pointer
    timerAlarmWrite(*pidTimer, 10000, true);  // 10 millisecond timer interrupt
    timerAlarmEnable(*pidTimer);

}
void IRAM_ATTR onPIDTimer() {
    input = position;
    mypid->Compute();
    if (output > 0) { // drive motor based off pid output
        ledcWrite(MOT_CH_IN1, abs(output));
        ledcWrite(MOT_CH_IN2, 0);
    } else {
        ledcWrite(MOT_CH_IN2, abs(output));
        ledcWrite(MOT_CH_IN1, 0);
    }

}
void setupDCMotor() {


}