// pinout.cpp
#include "pinout.h"
#include "motor_handler.h"

void setupPINs(){
    // Set pin modes
    pinMode(mainMotorPWM_PIN, OUTPUT);
    pinMode(turnCW_PIN, OUTPUT);
    pinMode(turnCCW_PIN, OUTPUT);
    pinMode(pot_pin, INPUT_PULLDOWN);
    pinMode(ble_led_PIN, OUTPUT);

    // Initialize motor control pins
    setMotorDirection(NONE);
}