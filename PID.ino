#include <Servo.h>  // Include the Servo library

Servo servoLeft;
Servo servoRight;

const int leftSensorPin = A0;   // Left infrared sensor (connected to A0)
const int rightSensorPin = A2;  // Right infrared sensor (connected to A2)

// PID control parameters
float Kp = 0.2;    // Proportional gain
float Ki = 0.0;    // Integral gain (can be set to 0 initially)
float Kd = 2.0;    // Derivative gain

float error = 0;
float previousError = 0;
float integral = 0;

const int baseSpeed = 1500;          // Neutral signal for servos (stop)
const int speedAdjust = 100;         // Maximum speed offset for forward movement

void setup() {
  Serial.begin(9600);               // Initialize serial communication
  pinMode(LED_BUILTIN, OUTPUT);     // Use built-in LED for status indication

  servoLeft.attach(10);  // Attach left servo to pin 10
  servoRight.attach(11); // Attach right servo to pin 11
}

void loop() {
  int leftValue = analogRead(leftSensorPin);   // Read left sensor
  int rightValue = analogRead(rightSensorPin); // Read right sensor

  // Calculate error: difference between right and left sensor readings
  // A positive error indicates deviation to the left
  error = rightValue - leftValue;
  integral += error;                               // Accumulate integral
  float derivative = error - previousError;        // Calculate derivative

  // PID output used as correction factor
  float correction = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;                           // Update previous error

  // Adjust PWM signals for each servo based on correction
  int leftPWM = baseSpeed + speedAdjust - correction;
  int rightPWM = baseSpeed - speedAdjust - correction;

  // Constrain PWM values to safe servo range
  leftPWM = constrain(leftPWM, 1300, 1700);
  rightPWM = constrain(rightPWM, 1300, 1700);

  // Print debug info to Serial Monitor
  Serial.print("L: "); Serial.print(leftValue);
  Serial.print(" | R: "); Serial.print(rightValue);
  Serial.print(" | Err: "); Serial.print(error);
  Serial.print(" | Corr: "); Serial.print(correction);
  Serial.print(" | L_PWM: "); Serial.print(leftPWM);
  Serial.print(" | R_PWM: "); Serial.println(rightPWM);

  // Send PWM signals to servos
  digitalWrite(LED_BUILTIN, HIGH);        // Turn on LED to show activity
  servoLeft.writeMicroseconds(leftPWM);   // Drive left servo
  servoRight.writeMicroseconds(rightPWM); // Drive right servo

  delay(30); // Small delay for fast, stable response
}
