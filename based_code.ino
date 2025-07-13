#include <Servo.h>  // Include the Servo library

Servo servoLeft;
Servo servoRight;

const int leftSensorPin = A2;   // Left infrared sensor connected to analog pin A2
const int rightSensorPin = A0;  // Right infrared sensor connected to analog pin A0

const int threshold = 100;      // Threshold value to distinguish between black and white surfaces

void setup() {
  Serial.begin(9600);                // Initialize serial communication for debugging
  pinMode(LED_BUILTIN, OUTPUT);     // Set built-in LED pin as output

  servoLeft.attach(10);   // Attach left servo to digital pin 10
  servoRight.attach(11);  // Attach right servo to digital pin 11
}

void loop() {
  // Read analog values from the sensors
  int leftValue = analogRead(leftSensorPin);
  int rightValue = analogRead(rightSensorPin);

  // Determine whether each sensor is over a black line
  bool leftOnLine = leftValue < threshold;
  bool rightOnLine = rightValue < threshold;

  // Print sensor values and status
  Serial.print("Left: ");
  Serial.print(leftValue);
  Serial.print(leftOnLine ? " [BLACK]" : " [WHITE]");
  Serial.print("  |  Right: ");
  Serial.print(rightValue);
  Serial.print(rightOnLine ? " [BLACK]" : " [WHITE]");
  Serial.print("  => Action: ");

  // Decide movement based on sensor states
  if (leftOnLine && rightOnLine) {
    Serial.println("GO FORWARD");
    goForward();  // Both sensors on black → move forward
  } else if (leftOnLine && !rightOnLine) {
    Serial.println("TURN LEFT");
    turnLeft();   // Left sensor on black, right on white → turn left
  } else if (!leftOnLine && rightOnLine) {
    Serial.println("TURN RIGHT");
    turnRight();  // Right sensor on black, left on white → turn right
  } else {
    Serial.println("STOP");
    stopMotors(); // Both sensors on white → stop
  }

  delay(150);  // Sampling interval delay
}

// Movement control functions

void goForward() {
  digitalWrite(LED_BUILTIN, HIGH);  // Turn on LED to indicate action
  servoLeft.writeMicroseconds(1600);  // Left wheel forward
  servoRight.writeMicroseconds(1400); // Right wheel forward
}

void turnLeft() {
  digitalWrite(LED_BUILTIN, HIGH);  
  servoLeft.writeMicroseconds(1500);   // Stop left wheel
  servoRight.writeMicroseconds(1450);  // Move right wheel forward slowly
}

void turnRight() {
  digitalWrite(LED_BUILTIN, HIGH);
  servoLeft.writeMicroseconds(1550);   // Move left wheel forward slowly
  servoRight.writeMicroseconds(1500);  // Stop right wheel
}

void stopMotors() {
  digitalWrite(LED_BUILTIN, LOW);   // Turn off LED
  servoLeft.writeMicroseconds(1500); // Stop left servo
  servoRight.writeMicroseconds(1500); // Stop right servo
}
