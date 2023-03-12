/*
  Read analog value and send over BLE on standard battery monitor characteristic
*/

#include <Arduino.h>
#include <ArduinoBLE.h>
#define UPDATE_THRESH 0
#define UPDATE_INTERVAL 10

#define UUID_Service 76ad7aaa-3782-11ed-a261-0242ac120002
#define UUID_Char_1 76ad7aa1-3782-11ed-a261-0242ac120002
#define UUID_Char_2 76ad7aa2-3782-11ed-a261-0242ac120002

#define BUTTON_1_PIN 2

#define X_VALUE_PIN A0
#define Y_VALUE_PIN A1

bool stateChanged = false;

bool button_1_State = false;  
bool turningDirection = 0; //Default direction is left
bool runningDirection = 0;//Default is forward

int x_readingRaw = 0;
int y_readingRaw = 0;
int x_reading = 0;
int y_reading = 0;

 // Bluetooth® Low Energy Battery Service
BLEService batteryService("180F");
BLEService joystickService("76ad7aaa-3782-11ed-a261-0242ac120002");

// Bluetooth® Low Energy Battery Level Characteristic
BLEUnsignedCharCharacteristic batteryLevelChar("2A19",  // standard 16-bit characteristic UUID
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes

BLEUnsignedIntCharacteristic x_readingChar("76ad7aa1-3782-11ed-a261-0242ac120002", BLERead | BLENotify);
BLEUnsignedIntCharacteristic y_readingChar("76ad7aa2-3782-11ed-a261-0242ac120002", BLERead | BLENotify);
BLEUnsignedIntCharacteristic x_rawReadingChar("76ad7ab1-3782-11ed-a261-0242ac120002", BLERead | BLENotify);
BLEUnsignedIntCharacteristic y_rawReadingChar("76ad7ab2-3782-11ed-a261-0242ac120002", BLERead | BLENotify);
BLEBooleanCharacteristic button1Char("76ad7aa3-3782-11ed-a261-0242ac120002", BLERead | BLENotify);
BLEUnsignedIntCharacteristic turningDirectionChar("76ad7aa6-3782-11ed-a261-0242ac120002", BLERead | BLENotify);
BLEUnsignedIntCharacteristic runningDirectionChar("76ad7aa7-3782-11ed-a261-0242ac120002", BLERead | BLENotify);

int x_prevReading = 0;  // last battery level reading from analog input
int y_prevReading = 0;  // last battery level reading from analog input
long previousMillis = 0;  // last time the analog reading, in ms

void setButton_1_State();
void printButtonState();
void printDirectionState();
void updateAnalogReading();

void setup() {
  BLE.setConnectionInterval(0x0006, 0x0006);
  Serial.begin(9600);    // initialize serial communication
  //while (!Serial);

  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected
  
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_1_PIN), setButton_1_State, CHANGE);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }

  /* Set a local name for the Bluetooth® Low Energy device
     This name will appear in advertising packets
     and can be used by remote devices to identify this Bluetooth® Low Energy device
     The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  BLE.setLocalName("JoyStickReading");
  BLE.setAdvertisedService(batteryService); // add the service UUID
  batteryService.addCharacteristic(batteryLevelChar); // add the battery level characteristic
  BLE.addService(batteryService); // Add the battery service

  BLE.setAdvertisedService(joystickService); // add the service UUID
  joystickService.addCharacteristic(x_readingChar); // add the x reading characteristic
  joystickService.addCharacteristic(y_readingChar); // add the y reading characteristic
  joystickService.addCharacteristic(x_rawReadingChar); // add the x reading characteristic
  joystickService.addCharacteristic(y_rawReadingChar); // add the y reading characteristic
  joystickService.addCharacteristic(button1Char);
  joystickService.addCharacteristic(turningDirectionChar);
  joystickService.addCharacteristic(runningDirectionChar);
  BLE.addService(joystickService); // Add the joystick service
  
  batteryLevelChar.writeValue(x_prevReading); // set initial value for this characteristic
  batteryLevelChar.writeValue(y_prevReading); // set initial value for this characteristic
  batteryLevelChar.writeValue(button_1_State);

  /* Start advertising Bluetooth® Low Energy.  It will start continuously transmitting Bluetooth® Low Energy
     advertising packets and will be visible to remote Bluetooth® Low Energy central devices
     until it receives a new connection */

  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");
}

void setButton_1_State()
{
  stateChanged = true;
  button_1_State = !button_1_State;
}

void printButtonState()
{
  Serial.print("Button1: ");
  Serial.println(button_1_State);
  button1Char.writeValue(button_1_State);
  
  Serial.print("\n");
  stateChanged = false;
}

void printDirectionState()
{
  Serial.print("runningDirection: ");
  Serial.println(runningDirection);
  runningDirectionChar.writeValue(runningDirection);

  Serial.print("turningDirection: ");
  Serial.println(turningDirection);
  turningDirectionChar.writeValue(turningDirection);
}

void updateAnalogReading() 
{
  
  /* Read the current voltage level on the A0, A1 analog input pins.
     This is used here to read Joystick pot values
  */
  x_readingRaw = analogRead(X_VALUE_PIN);
  y_readingRaw = analogRead(Y_VALUE_PIN);
  if (x_readingRaw < 511)
  {
    x_reading = map(x_readingRaw, 0, 511, 100, 0);
    if (x_reading > 15)
    {  
      turningDirection = 0;
    }
  }
  if (x_readingRaw > 511)
  {
    x_reading = map(x_readingRaw, 512, 1024, 0, 100);
    if (x_reading > 15)
    {  
      turningDirection = 1;
    }
  }

  if (y_readingRaw < 511)
  {
    y_reading = map(y_readingRaw, 0, 511, 100, 0);
    if (y_reading > 15)
    {  
      runningDirection = 0;
    }
  }
  if (y_readingRaw > 511)
  {
    y_reading = map(y_readingRaw, 512, 1024, 0, 100);
    if (y_reading > 15)
    {  
      runningDirection = 1;
    }
  }
  if (x_reading > 90)
  {
    x_reading = 100;
  }
  if (y_reading > 90)
  {
    y_reading = 100;
  }
  if (abs(x_reading-x_prevReading)>UPDATE_THRESH) // if the analog reading level has changed beyond preset threshold
    {      
      x_readingChar.writeValue((byte)x_reading);  // and update the characteristic
      turningDirectionChar.writeValue((byte)turningDirection);
      x_prevReading = x_reading;           // save the level for next comparison
    }
  
  if (abs(y_reading-y_prevReading)>UPDATE_THRESH)
  {      
    y_readingChar.writeValue((byte)y_reading); // Update characteristic
    runningDirectionChar.writeValue((byte)runningDirection);
    /* save the level for next comparison */
    y_prevReading = y_reading;
  }
  x_reading = 0;
  y_reading = 0;
}

void loop() {
  // wait for a Bluetooth® Low Energy central
  BLEDevice central = BLE.central();
  
  /* if a central is connected to the peripheral: */
  if (central) {
    Serial.print("Connected to central: ");
    /* print the central's BT address: */
    Serial.println(central.address());

    /* update analog readings while the central is connected: */
    while (central.connected()) {
      updateAnalogReading();
      if (stateChanged == true)
      {
        printButtonState();
        stateChanged = false;
      }
    }
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}
