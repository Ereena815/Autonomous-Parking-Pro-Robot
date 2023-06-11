/****
   SIT310 : Robotics Application Development
   Final Project : Parking Pro
   Name : Ereena Bagga
   Student ID : 221127314
 ****/
 
#include <PID_v1_bc.h>      // Library for PID control for motors
#include <ZumoMotors.h>  // Library to control zumo motors
#include <Wire.h>

// Define PID constants
double Kp = 1.0;  // Proportional gain
double Ki = 0.5;  // Integral gain
double Kd = 0.2;  // Derivative gain

// Define target position and PID variables
double targetPos = 0.0;
double currentPosition = 0.0;
double output = 0.0;

// Define PID objects
PID myPID(&currentPosition, &output, &targetPos, Kp, Ki, Kd, DIRECT);

// Define motor object
ZumoMotors motors;

// Define encoder pins
const int leftEncoderPin = 2;
const int rightEncoderPin = 3;

// Define encoder variables
volatile int leftEncoderCount = 0;
volatile int rightEncoderCount = 0;

// Define the pins for the ultrasonic sensors
const int echoPinLeft = 6;
const int trigPinLeft = 7;
const int echoPinFront = 11;
const int trigPinFront = 13;
const int echoPinRight = 4;
const int trigPinRight = 5;

// Define the pins for reflectance sensors
const int reflectancePin1 = A0;
const int reflectancePin2 = A1;

// Define obstacle avoidance variables
long duration;
int distance;
// Speed for the robot
const int parkingSpeed = 200;
// Distance between each parking slot
const int parkingDelay = 500;
// Thresholds for determining line detection
const int lineThreshold = 500;
// Thresholds for determining obstacle/car detection
const int obstacleDistanceThreshold = 10;

// Define parking variables
enum ParkingState {
  FORWARD_PARKING,
  REVERSE_PARKING,
  PARALLEL_PARKING,
  PARKING_COMPLETE
};

// Interrupt service routine for left encoder
void leftEncoderISR() {
  if (digitalRead(leftEncoderPin) == HIGH)
    leftEncoderCount++;
  else
    leftEncoderCount--;
}

// Interrupt service routine for right encoder
void rightEncoderISR() {
  if (digitalRead(rightEncoderPin) == HIGH)
    rightEncoderCount++;
  else
    rightEncoderCount--;
}

// Function to calculate obstacle distance
int ultrasonic(int echoPin, int trigPin) {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  return distance;
}

// Function to move the robot forward for a specified distance
void moveForward(double distance, double currentPosition) {
  targetPos = currentPosition + distance;
  while (currentPosition < targetPos) {
    // Set the motor speeds to move the robot forward
    int leftSpeed = constrain(int(output), -parkingSpeed, parkingSpeed);
    int rightSpeed = constrain(int(output), -parkingSpeed, parkingSpeed);
    motors.setSpeeds(leftSpeed, rightSpeed);

    // Update the current position based on encoder counts
    currentPosition = (leftEncoderCount + rightEncoderCount) / 2.0;
  }
  stopMoving();
}

// Function to move the robot backward for a specified distance
void moveBackward(double distance, double currentPosition) {
  targetPos = currentPosition - distance;
  while (currentPosition > targetPos) {
    // Set the motor speeds to move the robot backward
    int leftSpeed = -constrain(int(output), -parkingSpeed, parkingSpeed);
    int rightSpeed = -constrain(int(output), -parkingSpeed, parkingSpeed);
    motors.setSpeeds(leftSpeed, rightSpeed);

    // Update the current position based on encoder counts
    currentPosition = (leftEncoderCount + rightEncoderCount) / 2.0;
  }
  stopMoving();
}

// Function to turn the robot left by a specified angle
void turnLeft(double angle, double currentPosition) {
  targetPos = currentPosition - angle;  // Update target position based on angle
  while (currentPosition > targetPos) {
    // Calculate the difference between the target position and the current position
    double positionDifference = targetPos - currentPosition;

    // Set the motor speeds to make the robot turn left
    int leftSpeed = -constrain(int(output), -parkingSpeed, parkingSpeed);
    int rightSpeed = constrain(int(output), -parkingSpeed, parkingSpeed);
    motors.setSpeeds(leftSpeed, rightSpeed);

    // Update the current position based on encoder counts
    currentPosition = (leftEncoderCount + rightEncoderCount) / 2.0;
  }
  stopMoving();
}

// Function to turn the robot right by a specified angle
void turnRight(double angle, double currentPosition) {
  targetPos = currentPosition + angle;  // Update target position based on angle
  while (currentPosition < targetPos) {
    // Calculate the difference between the target position and the current position
    double positionDifference = targetPos - currentPosition;

    // Set the motor speeds to make the robot turn right
    int leftSpeed = constrain(int(output), -parkingSpeed, parkingSpeed);
    int rightSpeed = -constrain(int(output), -parkingSpeed, parkingSpeed);
    motors.setSpeeds(leftSpeed, rightSpeed);

    // Update the current position based on encoder counts
    currentPosition = (leftEncoderCount + rightEncoderCount) / 2.0;
  }
  stopMoving();
}

// Function to stop the robot
void stopMoving() {
  motors.setSpeeds(0, 0);  // Set motor speeds to zero
}

// Function to check if there is an empty space on the left
bool isLeftSpaceEmpty() {
  int leftDistance = ultrasonic(echoPinLeft, trigPinLeft);
  return leftDistance > obstacleDistanceThreshold;
}

// Function to check if there is an empty space on the right
bool isRightSpaceEmpty() {
  int rightDistance = ultrasonic(echoPinRight, trigPinRight);
  return rightDistance > obstacleDistanceThreshold;
}

// Function to check if there is an obstacle in the front
bool isFrontObstacle() {
  int front = ultrasonic(echoPinFront, trigPinFront);
  return front > obstacleDistanceThreshold;
}

// Function to check if parking done correctly
bool isParkedCorrectly() {
  // Read reflectance sensor values
  int sensor1 = analogRead(reflectancePin1);
  int sensor2 = analogRead(reflectancePin2);

  // Check if the robot is parked correctly
  if (sensor1 < lineThreshold && sensor2 < lineThreshold) {
    Serial.println("Parked correctly!");
    return true;
  } else {
    Serial.println("Not parked correctly!");
    return false;
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  pinMode(trigPinLeft, OUTPUT);   // Sets the trigPin as an Output
  pinMode(echoPinLeft, INPUT);    // Sets the echoPin as an Input
  pinMode(trigPinFront, OUTPUT);  // Sets the trigPin as an Output
  pinMode(echoPinFront, INPUT);   // Sets the echoPin as an Input
  pinMode(trigPinRight, OUTPUT);  // Sets the trigPin as an Output
  pinMode(echoPinRight, INPUT);   // Sets the echoPin as an Input

  // Set reflectance sensor pins as inputs
  pinMode(reflectancePin1, INPUT);
  pinMode(reflectancePin2, INPUT);

  // Attach encoder interrupt pins
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), rightEncoderISR, CHANGE);

  // Set PID tuning parameters
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-parkingSpeed, parkingSpeed);
  myPID.SetSampleTime(10);  // PID update interval in milliseconds

  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);
  motors.setSpeeds(0, 0);
  delay(2000);  // Allow time to place the robot on the ground
}


void loop() {
  // Update current position based on encoder counts
  currentPosition = (leftEncoderCount + rightEncoderCount) / 2.0;

  // Compute PID control
  myPID.Compute();

  moveForward(parkingDelay, currentPosition);

  // Look for obstacles in the front
  if (isFrontObstacle()) {
    moveBackward(parkingDelay, currentPosition);
  }

  // Move forward and look for empty spaces
  ParkingState currentState = FORWARD_PARKING;

  // Perform the parking maneuver
  switch (currentState) {
    case FORWARD_PARKING:
      // Check for available parking spaces
      if (isLeftSpaceEmpty() || isRightSpaceEmpty()) {
        if (isLeftSpaceEmpty() && !isRightSpaceEmpty()) {
          turnLeft(90, currentPosition);
          moveForward(parkingDelay, currentPosition);
        } else if (!isLeftSpaceEmpty() && isRightSpaceEmpty()) {
          turnRight(90, currentPosition);
          moveForward(parkingDelay, currentPosition);
        } else {
          turnLeft(90, currentPosition);
          moveForward(parkingDelay, currentPosition);
        }
        currentState = PARKING_COMPLETE;
      } else {
        moveForward(parkingDelay, currentPosition);
      }
      break;

    case REVERSE_PARKING:
      // Check for available parking spaces
      if (isLeftSpaceEmpty() || isRightSpaceEmpty()) {
        if (isLeftSpaceEmpty() && !isRightSpaceEmpty()) {
          turnRight(90, currentPosition);
          moveForward(parkingDelay, currentPosition);
          moveBackward(parkingDelay, currentPosition);
        } else if (!isLeftSpaceEmpty() && isRightSpaceEmpty()) {
          turnLeft(90, currentPosition);
          moveForward(parkingDelay, currentPosition);
          moveBackward(parkingDelay, currentPosition);
        } else {
          turnRight(90, currentPosition);
          moveForward(parkingDelay, currentPosition);
          moveBackward(parkingDelay, currentPosition);
        }
        currentState = PARKING_COMPLETE;
      } else {
        moveForward(parkingDelay, currentPosition);
      }
      break;

    case PARALLEL_PARKING:
      // Check for available parking spaces
      if (isLeftSpaceEmpty() || isRightSpaceEmpty()) {
        if (isLeftSpaceEmpty() && !isRightSpaceEmpty()) {
          turnRight(90, currentPosition);
          moveForward(parkingDelay, currentPosition);
          moveBackward(parkingDelay, currentPosition);
          turnLeft(90, currentPosition);
        } else if (!isLeftSpaceEmpty() && isRightSpaceEmpty()) {
          turnLeft(90, currentPosition);
          moveForward(parkingDelay, currentPosition);
          moveBackward(parkingDelay, currentPosition);
          turnRight(90, currentPosition);
        } else {
          turnRight(90, currentPosition);
          moveForward(parkingDelay, currentPosition);
          moveBackward(parkingDelay, currentPosition);
          turnLeft(90, currentPosition);
        }
        currentState = PARKING_COMPLETE;
      } else {
        moveForward(parkingDelay, currentPosition);
      }
      break;

    case PARKING_COMPLETE:
      if (isParkedCorrectly())
      {
        stopMoving();
      }
      else
      {
        moveForward(20, currentPosition);
        moveBackward(20, currentPosition);
      }
      break;
  }
}
