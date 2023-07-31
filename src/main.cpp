#include <Arduino.h>
#include "ble_handler.h"
#include "pinout.h"
#include "motor_handler.h"

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize BLE
  setupBLE();
  // Setup hardware pins
  setupPINs();
}

void loop() {
  // Handle BLE connection and commands
  handleCentralCommands();
  // Run motor towards setpoint even without BLE connection
  runMotor();
}