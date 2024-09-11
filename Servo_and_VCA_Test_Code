//CODE TO TEST THE VCA AND SERVO AT THE SAME TIME
//WORKING AS OF 10.09.24


#include <DRV8833.h>
#include <PWMServo.h>  // PWMServo library for servo control

/******* Motor Driver Definition *******/
DRV8833 driver = DRV8833();
const int inputA1 = 9, inputA2 = 10;  // [9,10] for VCA
#define SLP 4  // Sleep pin for DRV8833 driver

PWMServo servo;  // Servo motor for pinching force
#define SERVO_PIN 6
const int minimum_angle = 115;
const int maximum_angle = 165;

int setAngle = 140;  // Initial angle of the servo

/******* VCA Parameters *******/
float updateFrequency = 100.0f; // Frequency of the VCA pattern
float timeBetweenUpdates;
int patternLength = 500;
int hapticPattern[500];  // Pattern array for vibration
int patternIndex = 0;    // Index in the vibration pattern
int amp = 0;             // Amplitude for VCA vibration

float timeSinceLastUpdate = 0.0f;

void setup() {
  Serial.begin(2000000);  // Serial communication for debug
  Serial.setTimeout(1);

  /******* Servo Setup *******/
  servo.attach(SERVO_PIN);  // Attach the servo to the defined pin

  /******* VCA Setup *******/
  driver.attachMotorA(inputA1, inputA2);  // Attach VCA to motor A of DRV8833
  driver.motorAStop();  // Ensure motor is stopped initially

  pinMode(SLP, OUTPUT);
  digitalWrite(SLP, HIGH);  // Enable the motor driver (DRV8833)

  timeBetweenUpdates = 1.0f / updateFrequency;  // Calculate the time between updates based on frequency

  Serial.println("System Initialized. Type 'A' followed by an angle to move the servo (e.g., A140).");
  Serial.println("Type 'P' followed by an amplitude (e.g., P200) for VCA vibration.");
  Serial.println("Type 'F' followed by a frequency (e.g., F100) for VCA vibration frequency.");
}

void loop() {
  // Check if there's serial input available
  if (Serial.available()) {
    String st = Serial.readString();
    char command = st.charAt(0);
    String val = st.substring(1);  // Get the value from the string

    if (command == 'A') {
      // Servo angle command
      setAngle = DoAngleCommand(val);
      servo.write(setAngle);  // Move the servo to the specified angle
      Serial.print("Setting servo to angle: ");
      Serial.println(setAngle);
    } else if (command == 'P') {
      // VCA amplitude command
      DoPatternCommand(val);
      Serial.print("Setting VCA amplitude to: ");
      Serial.println(amp);
    } else if (command == 'F') {
      // VCA frequency command
      DoFreqCommand(val);
      Serial.print("Setting VCA frequency to: ");
      Serial.println(updateFrequency);
    }
  }

  // Update VCA vibration pattern based on frequency and amplitude
  timeSinceLastUpdate += (micros() - timeSinceLastUpdate) / 1000000.0f;
  if (timeSinceLastUpdate >= timeBetweenUpdates) {
    if (amp > 0) {
      if (hapticPattern[patternIndex] >= 0) {
        driver.motorAForward(hapticPattern[patternIndex]);  // Forward direction vibration
      } else {
        driver.motorAReverse(-hapticPattern[patternIndex]);  // Reverse direction vibration
      }
    } else {
      driver.motorAStop();  // Stop vibration if amplitude is 0
    }

    patternIndex += 1;
    if (patternIndex >= patternLength) {
      patternIndex = 0;  // Loop the pattern
    }

    timeSinceLastUpdate = 0.0f;
  }
}

double DoAngleCommand(String val) {
  double angle = val.toFloat();  // Convert the string value to a float

  // Ensure the angle is within the valid range
  if (angle > maximum_angle) {
    angle = maximum_angle;
  } else if (angle < minimum_angle) {
    angle = minimum_angle;
  }

  return angle;
}

void DoPatternCommand(String val) {
  amp = val.toInt();  // Convert the string value to an integer

  // Ensure the amplitude is within the valid range (0 - 500)
  if (amp > 500) {
    amp = 500;
  } else if (amp < 0) {
    amp = 0;
  }

  // Generate a random vibration pattern based on the amplitude
  for (int i = 0; i < patternLength; ++i) {
    hapticPattern[i] = random(-amp, amp);
  }
}

void DoFreqCommand(String val) {
  updateFrequency = val.toFloat();  // Convert the string value to a float

  // Ensure the frequency is within the valid range (1 - 15000 Hz)
  if (updateFrequency > 15000) {
    updateFrequency = 15000;
  } else if (updateFrequency < 1) {
    updateFrequency = 1;
  }

  timeBetweenUpdates = 1.0f / updateFrequency;  // Recalculate time between updates
}
