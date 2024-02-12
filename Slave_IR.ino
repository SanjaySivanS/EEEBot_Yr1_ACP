#include <Wire.h>

int motor1Pin1 = 26;
int motor1Pin2 = 27;
int enable1Pin = 33;

int motor2Pin1 = 14;
int motor2Pin2 = 12;
int enable2Pin = 25;

const int freq = 2000;
const int servoFrequency = 50;
const int ledChannela = 0;
const int ledChannelb = 1;
const int servoChannel = 2;
const int resolution = 8;
const int servoResolution = 12;

int servoPin = 13;
float steeringAngle;

const int numBytes = 6;

void setup() {
  Wire.begin(8); // Initialize I2C communication with address 8
  Wire.onReceive(receiveEvent); // Register event for receiving data
  Serial.begin(115200); // Start serial communication for output

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);

  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

  digitalWrite(enable1Pin, HIGH);
  digitalWrite(enable2Pin, HIGH);

  ledcSetup(servoChannel, servoFrequency, servoResolution);
  ledcAttachPin(servoPin, servoChannel);
}

void loop() {
  // Motor control logic based on steering angle
  if (steeringAngle < 300) {
    analogWrite(motor2Pin1, 30 * 255 / 100); // Right motor speed set to 30%
    analogWrite(motor2Pin2, 0); // Right motor direction
    analogWrite(motor1Pin1, 55 * 255 / 100); // Left motor speed set to 55%
    analogWrite(motor1Pin2, 0); // Left motor direction
  } else if (steeringAngle > 350) {
    analogWrite(motor1Pin1, 30 * 255 / 100); // Left motor speed set to 30%
    analogWrite(motor1Pin2, 0); // Left motor direction
    analogWrite(motor2Pin1, 55 * 255 / 100); // Right motor speed set to 55%
    analogWrite(motor2Pin2, 0); // Right motor direction
  } else {
    // Both motors go forward at 55% speed
    analogWrite(motor1Pin1, 55 * 255 / 100);
    analogWrite(motor1Pin2, 0);
    analogWrite(motor2Pin1, 55 * 255 / 100);
    analogWrite(motor2Pin2, 0);
  }
}

void receiveEvent(int numBytes) {
  while (Wire.available() < numBytes); // Wait until data is available
  Wire.readBytes((char*)&steeringAngle, sizeof(steeringAngle)); // Read steeringAngle from master
  // Map angle to servo range (center is 325, left is 240, right is 370)
  int servoValue = map(steeringAngle, -90, 90, 240, 370);
  ledcWrite(servoChannel, servoValue); // Write mapped value to servo
}

