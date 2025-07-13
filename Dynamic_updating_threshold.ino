#include <Servo.h>  // Include the Servo library to control servos

Servo servoLeft;
Servo servoRight;

const int leftSensorPin = A0;   // Left infrared sensor
const int rightSensorPin = A2;  // Right infrared sensor

// PID parameters
float Kp = 0.7;   // Proportional gain
float Ki = 0.1;   // Integral gain
float Kd = 0.01;  // Derivative gain

float error = 0, previousError = 0, integral = 0;

// Servo control parameters
const int baseSpeed = 1500;     // Neutral PWM signal (servo stop)
const int speedAdjust = 50;     // Base adjustment for forward speed

// Target value will be dynamically calibrated
int targetValue = 75;           // Initial dummy value (will be updated at startup)
const int deadZone = 15;        // Range in which difference is considered zero
const int maxDifference = 200;  // Max PID error difference to avoid overshoot

// Dynamic auto-tuning parameters
const int autoTuneInterval = 500;        // Time interval (ms) to refresh targetValue
unsigned long lastTuneTime = 0;          // Last update timestamp

void setup() {
  Serial.begin(9600);                    // Start Serial Monitor for debugging
  pinMode(LED_BUILTIN, OUTPUT);         // Use built-in LED as a status indicator
  servoLeft.attach(10);                 // Attach left servo to pin 10
  servoRight.attach(11);                // Attach right servo to pin 11

  // ✅ Initial calibration of targetValue at startup (take 10 samples)
  const int sampleCount = 10;
  int totalLeft = 0, totalRight = 0;

  Serial.println("Auto-calibrating targetValue...");
  for (int i = 0; i < sampleCount; i++) {
    int left = analogRead(leftSensorPin);
    int right = analogRead(rightSensorPin);
    totalLeft += left;
    totalRight += right;

    Serial.print("Sample "); Serial.print(i);
    Serial.print(" | L: "); Serial.print(left);
    Serial.print(" | R: "); Serial.println(right);
    delay(100);  // Delay between samples
  }

  // Set initial targetValue to average of readings
  targetValue = (totalLeft + totalRight) / (2 * sampleCount);
  Serial.print(">> Initial targetValue: ");
  Serial.println(targetValue);
}

void loop() {
  // Read sensor values
  int leftValue = analogRead(leftSensorPin);
  int rightValue = analogRead(rightSensorPin);
  int difference = rightValue - leftValue;

  // ✅ Dynamically update targetValue during operation
  if (millis() - lastTuneTime > autoTuneInterval) {
    // Update only if difference is small (i.e. robot is on the line)
    if (abs(difference) <= deadZone) {
      targetValue = (leftValue + rightValue) / 2;
      Serial.print(">> Updated targetValue: ");
      Serial.println(targetValue);
    }
    lastTuneTime = millis();  // Reset timer
  }

  // Calculate sensor error from target value
  int leftError = leftValue - targetValue;
  int rightError = rightValue - targetValue;

  // If within dead zone, treat as centered (no error)
  if (abs(difference) <= deadZone) {
    error = 0;
  } else {
    error = rightError - leftError;
  }

  // Limit error range to avoid extreme correction
  error = constrain(error, -maxDifference, maxDifference);

  // PID control
  integral += error;
  integral = constrain(integral, -1000, 1000);  // Clamp integral term
  float derivative = error - previousError;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;

  // Calculate PWM output for servos
  int leftPWM = baseSpeed + speedAdjust - correction;
  int rightPWM = baseSpeed - speedAdjust - correction;

  // Constrain PWM values to safe range
  leftPWM = constrain(leftPWM, 1300, 1700);
  rightPWM = constrain(rightPWM, 1300, 1700);

  // Send PWM signals to servos
  digitalWrite(LED_BUILTIN, HIGH);  // Turn on LED to show active tracking
  servoLeft.writeMicroseconds(leftPWM);
  servoRight.writeMicroseconds(rightPWM);

  // Debug info for monitoring
  Serial.print("L: "); Serial.print(leftValue);
  Serial.print(" | R: "); Serial.print(rightValue);
  Serial.print(" | T: "); Serial.print(targetValue);
  Serial.print(" | Err: "); Serial.print(error);
  Serial.print(" | Corr: "); Serial.print(correction);
  Serial.print(" | L_PWM: "); Serial.print(leftPWM);
  Serial.print(" | R_PWM: "); Serial.println(rightPWM);

  delay(20); // Small delay for smooth and responsive control
}

