#include <Arduino.h>
#include <ArduinoBLE.h>

// Pin definitions
const int mainMotorPWM_PIN = 10;
const int turnCW_PIN = 9;
const int turnCCW_PIN = 8;
const int pot_pin = 0;
int led_PIN = 3;

// BLE UUIDs
const char* serviceUUID = "76ad7aaa-3782-11ed-a261-0242ac120002";
const char* setpointCharUUID = "76ad7aa4-3782-11ed-a261-0242ac120002";
const char* potValueCharUUID = "76ad7aa5-3782-11ed-a261-0242ac120002";

BLEService service(serviceUUID);
BLEUnsignedIntCharacteristic setpointChar(setpointCharUUID, BLERead | BLEWrite);
BLEUnsignedIntCharacteristic potValueChar(potValueCharUUID, BLERead | BLENotify);

// PID variables
double setpoint = 127;
double input = 0;
double pot_value = 0;
double pot_value_raw = 0;
double output = 0;
double Kp = 13;  // Proportional constant
double Ki = 0.5;  // Integral constant
double Kd = 0;  // Derivative constant
double integralTerm = 0;
double previousInput = 0;
double previousOutput = 0;
int pwm = 255;

// Moving average filter
const int numReadings = 50; // Number of readings to average
int readings[numReadings];  // Array to store readings
int readIndex = 0;          // Index for the next reading to be stored
int total = 0;              // Running total of readings


// Function prototypes
void setupBLE();
void handleCentralCommands();
void runMotor();
void runMotorMaxPWM(int direction);
void updateMotorSpeed();
void updatePotValueCharacteristic();
int getSmoothedValue();

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize BLE
  setupBLE();

  // Set pin modes
  pinMode(mainMotorPWM_PIN, OUTPUT);
  pinMode(turnCW_PIN, OUTPUT);
  pinMode(turnCCW_PIN, OUTPUT);
  pinMode(pot_pin, INPUT_PULLDOWN);
  pinMode(led_PIN, OUTPUT);

  // Initialize motor control pins
  digitalWrite(turnCW_PIN, LOW);
  digitalWrite(turnCCW_PIN, LOW);

  // Initialize the readings array to 0
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }
}

void loop() {
  handleCentralCommands();
  runMotor();
}

void setupBLE() {
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }

  BLE.setLocalName("BoatMotorControl");
  BLE.setAdvertisedService(service);
  service.addCharacteristic(setpointChar);
  service.addCharacteristic(potValueChar);
  BLE.addService(service);
  BLE.advertise();
}

void handleCentralCommands() {
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(led_PIN, HIGH);

    while (central.connected()) {
      if (setpointChar.written()) {
        setpoint = setpointChar.value();
        Serial.print("Setpoint, input: ");
        Serial.print(setpoint);
        Serial.print(" , ");
        Serial.println(analogRead(pot_pin));
        Serial.print("output: ");
        Serial.println(output);
        Serial.print("PWM: ");
        Serial.println(pwm);
      }
      runMotor();
    }
  } else {
    digitalWrite(led_PIN, LOW);
  }
}

void runMotor() {
  // Define the hysteresis value (adjust this based on your requirements)
  const int hysteresis = 5;
  // Read input from potentiometer
  //pot_value = analogRead(pot_pin);
  // Smooth out the raw value using the moving average filter
  pot_value = getSmoothedValue();

  // Adjust motor direction based on output value with hysteresis
  if (pot_value < setpoint - hysteresis) {
    digitalWrite(turnCW_PIN, HIGH);
    digitalWrite(turnCCW_PIN, LOW);
  } else if (pot_value > setpoint + hysteresis) {
    digitalWrite(turnCW_PIN, LOW);
    digitalWrite(turnCCW_PIN, HIGH);
  }

  // Set PWM value based on pot value in relation to pot center
  if (abs(pot_value - setpoint) > 15) {
    pwm = 255;
  } else if (abs(pot_value - setpoint) > 5) {
    pwm = 150;
  } else {
    pwm = 0;
  }

  analogWrite(mainMotorPWM_PIN, pwm);
  updatePotValueCharacteristic();
}

void runMotorPID() {
  // Read input from potentiometer
  input = analogRead(pot_pin);

  // Compute PID terms
  double error = setpoint - input;
  integralTerm += error;
  double derivativeTerm = input - previousInput;

  // Compute output using PID control equation
  output = Kp * error + Ki * integralTerm + Kd * derivativeTerm;

  // Adjust motor direction based on output value
  if (output > 0) {
    digitalWrite(turnCW_PIN, HIGH);
    digitalWrite(turnCCW_PIN, LOW);
  } else if (output < 0) {
    digitalWrite(turnCW_PIN, LOW);
    digitalWrite(turnCCW_PIN, HIGH);
  } else {
    // Stop the motor when output is zero
    digitalWrite(turnCW_PIN, LOW);
    digitalWrite(turnCCW_PIN, LOW);
  }

  output = abs(output);
  output = constrain(output, 0, 255); // Limit output to PWM range

  // Set PWM value based on output
  analogWrite(mainMotorPWM_PIN, static_cast<int>(output));

  // Update previous input and output
  previousInput = input;
  previousOutput = output;
}

void runMotorMaxPWM(int direction) {
  // Drive the motor with maximum PWM until pot value stops changing
  double prevPotValue = 0;
  while (true) {
    input = analogRead(pot_pin);
    if (input != prevPotValue) {
      // Potentiometer value changed, keep driving at max PWM
      if (direction > 0) {
        digitalWrite(turnCW_PIN, HIGH);
        digitalWrite(turnCCW_PIN, LOW);
      } else if (direction < 0) {
        digitalWrite(turnCW_PIN, LOW);
        digitalWrite(turnCCW_PIN, HIGH);
      }
      analogWrite(mainMotorPWM_PIN, 255);
      prevPotValue = input;
    } else {
      // Potentiometer value stopped changing, stop the motor and exit the loop
      digitalWrite(turnCW_PIN, LOW);
      digitalWrite(turnCCW_PIN, LOW);
      analogWrite(mainMotorPWM_PIN, 0);
      break;
    }
    delay(100);
  }
}

void updateMotorSpeed() {
  // Check the setpoint and run the motor accordingly
  if (setpoint == 1) {
    // Drive the motor with maximum PWM in the clockwise direction until pot value stops changing
    runMotorMaxPWM(1);
  } else if (setpoint == 2) {
    // Drive the motor with maximum PWM in the counterclockwise direction until pot value stops changing
    runMotorMaxPWM(-1);
  } else {
    // For all other setpoint values, call runMotor
    runMotor();
  }
}

void updatePotValueCharacteristic() {
  // Update the potValueChar characteristic with the potentiometer value
  potValueChar.writeValue(pot_value);
}

int getSmoothedValue() {
  // Subtract the last reading from the total
  total = total - readings[readIndex];

  // Read from the potentiometer and add the new reading to the total
  readings[readIndex] = analogRead(pot_pin);
  total = total + readings[readIndex];

  // Advance to the next position in the array
  readIndex++;

  // If we've reached the end of the array, wrap around to the beginning
  if (readIndex >= numReadings) {
    readIndex = 0;
  }

  // Calculate the average and return the smoothed value
  return total / numReadings;
}
