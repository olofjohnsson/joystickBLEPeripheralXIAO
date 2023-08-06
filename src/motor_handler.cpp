// motor_handler.cpp
#include "motor_handler.h"
#include "ble_handler.h"
#include "pinout.h"

// PID variables
double input = 0;
double output = 0;
double Kp = 13;  // Proportional constant
double Ki = 0.5;  // Integral constant
double Kd = 0;  // Derivative constant
double integralTerm = 0;
double previousInput = 0;
double previousOutput = 0;

// Potentiometer
double pot_value = 0;
double pot_center = 227;

// Motor PWM signal
int pwm = 255;

// Smoothening function
// Moving average filter
const int numReadings = 10; // Number of readings to average
int readings[numReadings];  // Array to store readings

void runMotor() {
  static bool motor_is_active = false;
  // Define the hysteresis value (adjust this based on your requirements)
  const int hysteresis = 1;
  // Read input from potentiometer
  pot_value = getMedianValue();

  if (control_output > 0)
  {
    setMotorDirection(CW);
    pwm = 255;
    control_output--;
  }
  else if (control_output < 0)
  {
    setMotorDirection(CCW);
    pwm = 255;
    control_output++;
  }
  else
  {
    // Adjust motor direction based on output value with hysteresis
    if (pot_value < setpoint - hysteresis) {
      setMotorDirection(CW);
    } else if (pot_value > setpoint + hysteresis) {
      setMotorDirection(CCW);
    }

    // Set PWM value
    if (abs(pot_value - setpoint) > 15) {
      pwm = 255;
      motor_is_active = true;
    } else if (abs(pot_value - setpoint) > 1) {
      pwm = 150;
      motor_is_active = true;
    } else {
      pwm = 0;
      if (motor_is_active == true)
      {
        Serial.print("Setpoint: ");
        Serial.println(setpoint);
        Serial.print("Pot median value: ");
        Serial.println(pot_value);
        motor_is_active = false;
      }
    }
  }

  analogWrite(mainMotorPWM_PIN, pwm);
  updatePotValueCharacteristic(pot_value);
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
    setMotorDirection(CW);
  } else if (output < 0) {
    setMotorDirection(CCW);
  } else {
    // Stop the motor when output is zero
    setMotorDirection(NONE);
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
        setMotorDirection(CW);
      } else if (direction < 0) {
        setMotorDirection(CCW);
      }
      analogWrite(mainMotorPWM_PIN, 255);
      prevPotValue = input;
    } else {
      // Potentiometer value stopped changing, stop the motor and exit the loop
      setMotorDirection(NONE);
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

int getMedianValue() {
    // Initialize the readings array to 0
    for (int i = 0; i < numReadings; i++) {
        readings[i] = 0;
    }
    int readings[numReadings]; // Array to store readings

    // Collect readings from the potentiometer
    for (int i = 0; i < numReadings; i++) {
        readings[i] = analogRead(pot_pin);
        delay(10); // Add a small delay between readings if needed
    }

    // Sort the readings in ascending order
    for (int i = 0; i < numReadings - 1; i++) {
        for (int j = i + 1; j < numReadings; j++) {
            if (readings[i] > readings[j]) {
            int temp = readings[i];
            readings[i] = readings[j];
            readings[j] = temp;
            }
        }
    }

  // Calculate and return the median value
  int medianIndex = numReadings / 2;
  return readings[medianIndex];
}

void setMotorDirection(uint8_t dir)
{
    switch(dir)
    {
        case CW:
            digitalWrite(turnCW_PIN, HIGH);
            digitalWrite(turnCCW_PIN, LOW);
            break;
        case CCW:
            digitalWrite(turnCW_PIN, LOW);
            digitalWrite(turnCCW_PIN, HIGH);
            break;
        default:
            digitalWrite(turnCW_PIN, LOW);
            digitalWrite(turnCCW_PIN, LOW);
            break;
    }
}