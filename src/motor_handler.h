// motor_handler.h
#ifndef MOTOR_HANDLER_H
#define MOTOR_HANDLER_H

#include <Arduino.h>

// Function declaration
void runMotor();
void runMotorMaxPWM(int direction);
void updateMotorSpeed();
int getMedianValue();
void setMotorDirection(uint8_t dir);

#endif // MOTOR_HANDLER_H