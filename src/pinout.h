// pinout.h
#ifndef PINOUT_H
#define PINOUT_H

#include <Arduino.h>

// Pin definitions
const int mainMotorPWM_PIN = 10;
const int turnCW_PIN = 9;
const int turnCCW_PIN = 8;
const int pot_pin = 0;
const int ble_led_PIN = 3;

void setupPINs();

#endif // PINOUT_H