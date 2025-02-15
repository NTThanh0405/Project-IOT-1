#include <Arduino.h>

const int trigPin = 2;
const int echoPin = 3;
const int motorPin = 4;

// Define motor control pins (replace with your actual pins)
const int inR1 = 5;
const int inR2 = 6;
const int inL1 = 7;
const int inL2 = 8;

// Define distance threshold
const int allowDistance = 10; // cm

// Function to control the robot's movement
void robotMover(int inR1, int inR2, int inL1, int inL2, int direction) {
  switch (direction) {
    case 1: // Forward
      digitalWrite(inR1, HIGH);
      digitalWrite(inR2, LOW);
      digitalWrite(inL1, HIGH);
      digitalWrite(inL2, LOW);
      break;
    case 2: // Backward
      digitalWrite(inR1, LOW);
      digitalWrite(inR2, HIGH);
      digitalWrite(inL1, LOW);
      digitalWrite(inL2, HIGH);
      break;
    case 0: // Stop
      digitalWrite(inR1, LOW);
      digitalWrite(inR2, LOW);
      digitalWrite(inL1, LOW);
      digitalWrite(inL2, LOW);
      break;
    default: // Stop (default case)
      digitalWrite(inR1, LOW);
      digitalWrite(inR2, LOW);
      digitalWrite(inL1, LOW);
      digitalWrite(inL2, LOW);
      break;
  }
}

// Function to measure distance using ultrasonic sensor
long objectDistance_cm(int angle) {  // Added angle parameter for potential future use
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  return distance;
}

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(motorPin, OUTPUT);

  // Set motor control pins as outputs
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);
}

void loop() {
  long front_distance = objectDistance_cm(0); // Measure front distance

  // Obstacle avoidance logic using the single front sensor
  if (front_distance < allowDistance) {
    robotMover(inR1, inR2, inL1, inL2, 2); // Move backward
    Serial.println("Lùi");
    delay(300);
    robotMover(inR1, inR2, inL1, inL2, 0); // Stop
    delay(100); // Small delay before turning

    long left_distance = objectDistance_cm(180); // Measure left distance
    long right_distance = objectDistance_cm(0); // Measure right distance

    if (left_distance > right_distance) {
      // Turn left (replace with your turning logic)
      // Example:
      digitalWrite(inR1, LOW);
      digitalWrite(inR2, HIGH);
      digitalWrite(inL1, HIGH);
      digitalWrite(inL2, LOW);
      delay(500); // Adjust turning time
      robotMover(inR1, inR2, inL1, inL2, 0); // Stop
    } else {
      // Turn right (replace with your turning logic)
      // Example:
      digitalWrite(inR1, HIGH);
      digitalWrite(inR2, LOW);
      digitalWrite(inL1, LOW);
      digitalWrite(inL2, HIGH);
      delay(500); // Adjust turning time
      robotMover(inR1, inR2, inL1, inL2, 0); // Stop
    }
  } else {
    robotMover(inR1, inR2, inL1, inL2, 1); // Move forward
    Serial.println("Tiến");
  }

  Serial.print("front: ");
  Serial.println(front_distance);
  //Serial.print("left: ");
  //Serial.println(left_distance);
  //Serial.print("right: ");
  //Serial.println(right_distance);
  delay(10);
}