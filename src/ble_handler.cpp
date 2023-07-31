// ble_handler.cpp
#include "ble_handler.h"
#include "pinout.h"
#include "motor_handler.h"

// BLE UUIDs
const char* serviceUUID = "76ad7aaa-3782-11ed-a261-0242ac120002";
const char* setpointCharUUID = "76ad7aa4-3782-11ed-a261-0242ac120002";
const char* potValueCharUUID = "76ad7aa5-3782-11ed-a261-0242ac120002";
const char* contorlCharUUID = "76ad7aa6-3782-11ed-a261-0242ac120002";

BLEService service(serviceUUID);
BLEUnsignedIntCharacteristic setpointChar(setpointCharUUID, BLERead | BLEWrite);
BLEUnsignedIntCharacteristic potValueChar(potValueCharUUID, BLERead | BLENotify);
BLEIntCharacteristic contorlChar(contorlCharUUID, BLERead | BLEWrite);

double setpoint = 252;
int control_output = 0;

void setupBLE() {
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }

  BLE.setLocalName("BoatMotorControl");
  BLE.setAdvertisedService(service);
  service.addCharacteristic(setpointChar);
  service.addCharacteristic(potValueChar);
  service.addCharacteristic(contorlChar);
  BLE.addService(service);
  BLE.advertise();
}

void handleCentralCommands() {
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(ble_led_PIN, HIGH);

    while (central.connected()) {
      if (setpointChar.written()) {
        setpoint = setpointChar.value();
        Serial.print("Setpoint: ");
        Serial.println(setpoint);
      }
      if (contorlChar.written())
      {
        control_output = contorlChar.value();
      }
      runMotor();
    }
  } 
  else 
  {
    digitalWrite(ble_led_PIN, LOW);
  }
}

void updatePotValueCharacteristic(uint32_t pot_value) {
  // Update the potValueChar characteristic with the potentiometer value
  potValueChar.writeValue(pot_value);
}