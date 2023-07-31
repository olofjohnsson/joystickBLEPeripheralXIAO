// ble_handler.h

#ifndef BLE_HANDLER_H
#define BLE_HANDLER_H

#include <ArduinoBLE.h>

extern double setpoint;
extern int control_output;

void setupBLE();
void handleCentralCommands();
void updatePotValueCharacteristic(uint32_t pot_value);

#endif // BLE_HANDLER_H
