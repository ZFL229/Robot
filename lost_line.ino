#include <Servo.h>  // Include the Servo library

Servo servoLeft;
Servo servoRight;

const int leftSensorPin = A0;  // Left infrared sensor (analog pin A0)
const int rightSensorPin = A2; // Right infrared sensor (analog pin A2)

const int targetValue = 110;   // Target threshold to distinguish black line
const int baseSpeed = 1500;    // Neutral servo pulse (stop position)
const int speedAdjust = 100;   // Maximum adjustment to PWM pulse width

// PID control parameters
float Kp = 0.9;
float Ki = 0.0;
float Kd = 5.0;

float error = 0;
float previousError = 0;
float integral = 0;

// Line-loss detection and buffering
bool lostLine = false;              // Whether robot is in recovery mode
unsigned long lostTime = 0;         // Time when line was lost
int lostCounter = 0;                // Count of consecutive line-loss detections
const int lostThreshold = 5;        // Threshold for entering recovery mode

void setup() {
  Serial.begin(9600);              // Start serial monitor
  pinMode(LED_BUILTIN, OUTPUT);    // Use built-in LED to indicate state
  servoLeft.attach(10);            // Attach left servo to pin 10
  servoRight.attach(11);           // Attach right servo to pin 11
}

void loop() {
  // Read sensor values
  int leftValue = analogRead(leftSensorPin);
  int rightValue = analogRead(rightSensorPin);

  // Determine if sensors detect black line
  bool leftOnLine = leftValue < targetValue;
  bool rightOnLine = rightValue < targetValue;

  // ✅ Buffered line-loss detection logic
  if (leftOnLine || rightOnLine) {
    lostCounter = 0;        // Reset counter if line is detected
    lostLine = false;       // Exit recovery mode
  } else {
    lostCounter++;          // Increment loss counter
    if (lostCounter >= lostThreshold && !lostLine) {
      lostLine = true;      // Enter recovery mode
      lostTime = millis();  // Record time when line was lost
    }
  }

  // ✅ Normal line-following using PID
  if (!lostLine) {
    digitalWrite(LED_BUILTIN, HIGH); // Turn on LED to indicate normal tracking

    // Calculate PID error based on sensor deviation from target
    int leftError = leftValue - targetValue;
    int rightError = rightValue - targetValue;
    error = rightError - leftError;     // Differential error between sensors

    integral += error;                  // Accumulate integral
    float derivative = error - previousError; // Change in error (derivative)
    float correction = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;             // Save error for next cycle

    // Compute PWM signals for servos based on correction
    int leftPWM = baseSpeed + speedAdjust - correction;
    int rightPWM = baseSpeed - speedAdjust - correction;

    // Clamp PWM values to safe servo range
    leftPWM = constrain(leftPWM, 1300, 1700);
    rightPWM = constrain(rightPWM, 1300, 1700);

    // Apply PWM to servos
    servoLeft.writeMicroseconds(leftPWM);
    servoRight.writeMicroseconds(rightPWM);

    // Debugging output
    Serial.print("L: "); Serial.print(leftValue);
    Serial.print(" | R: "); Serial.print(rightValue);
    Serial.print(" | Err: "); Serial.print(error);
    Serial.print(" | Corr: "); Serial.print(correction);
    Serial.print(" | L_PWM: "); Serial.print(leftPWM);
    Serial.print(" | R_PWM: "); Serial.println(rightPWM);
  }

  // ❌ Recovery mode (when both sensors lost the line)
  else {
    digitalWrite(LED_BUILTIN, LOW); // Turn off LED to indicate recovery

    unsigned long timeSinceLost = millis() - lostTime;

    if (timeSinceLost < 2000) {
      // First 2 seconds: move backward
      servoLeft.writeMicroseconds(1400);
      servoRight.writeMicroseconds(1600);
      Serial.println("Recovery: BACKWARD");
    }
    else if (timeSinceLost < 4000) {
      // 2–4 seconds: spin left to search
      servoLeft.writeMicroseconds(1500); // Stop left wheel
      servoRight.writeMicroseconds(1600); // Right wheel forward
      Serial.println("Recovery: SEARCH LEFT");
    }
    else if (timeSinceLost < 6000) {
      // 4–6 seconds: spin right to search
      servoLeft.writeMicroseconds(1600); // Left wheel forward
      servoRight.writeMicroseconds(1500); // Stop right wheel
      Serial.println("Recovery: SEARCH RIGHT");
    }
    else {
      // After 6 seconds: stop completely
      servoLeft.writeMicroseconds(1500);
      servoRight.writeMicroseconds(1500);
      Serial.println("Recovery: STOPPED - LINE NOT FOUND");
    }
  }

  delay(30); // Sampling interval to ensure smooth operation
}

