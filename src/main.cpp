#include <Arduino.h>
#include "ble_handler.h"
#include "pinout.h"

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize BLE
  setupBLE();
  // Setup hardware pins
  setupPINs();
}

void loop() {
  handleCentralCommands();
}