#include <Wire.h>
#include <math.h>

const int freq = 2000;
const int servoFrequency = 50;
const int ledChannela = 0;
const int ledChannelb = 1;
const int servoChannel = 2;
const int resolution = 8;
const int servoResolution = 12;
bool reverse;
int servoPin = 13;
int sensors;
float Xpk;
char lineIs;
byte speedSetting = 0;   // Initial speed = 0
byte speedRampFlag = 1;  // Define a direction controller for the loop
byte changeDirection = 0;

#define enA 33  // EnableA command line
#define enB 25  // EnableA command line
#define INa 26  // Channel A Direction
#define INb 27  // Channel A Direction
#define INc 14  // Channel B Direction
#define INd 12  // Channel B Direction

float steeringAngle;
int rightSpeed = 120;
int leftSpeed = 120;
bool stop = false;

void goFowards() {
  digitalWrite(INa, LOW);
  digitalWrite(INb, HIGH);
  digitalWrite(INc, HIGH);
  digitalWrite(INd, LOW);
}

void goBackwards() {
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW);
  digitalWrite(INd, HIGH);
}

void setup() {
  Serial.begin(9600);
  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);

  ledcSetup(ledChannela, freq, resolution);
  ledcSetup(ledChannelb, freq, resolution);
  ledcSetup(servoChannel, servoFrequency, servoResolution);

  ledcAttachPin(enA, ledChannela);
  ledcAttachPin(enB, ledChannelb);
  ledcAttachPin(servoPin, servoChannel);

  ledcWrite(servoChannel, 282);

  Wire.begin(0x08);
  Wire.onReceive(receiveEvent);

  goFowards();
}

void motors(int leftSpeed, int rightSpeed) {
  ledcWrite(ledChannela, leftSpeed);
  ledcWrite(ledChannelb, rightSpeed);
  delay(25);
}

void loop() {
  if (!reverse) {
    steeringAngle = round(282 - Xpk / 1.25);
    digitalWrite(INa, LOW);
    digitalWrite(INb, HIGH);
    digitalWrite(INc, HIGH);
    digitalWrite(INd, LOW);
    rightSpeed = 115 + Xpk / 10;
    leftSpeed = 115 - Xpk / 10;
  } else {
    if (lineIs == 'r') {
      steeringAngle = 380;
    }
    if (lineIs == 'l') {
      steeringAngle = 184;
    }
    digitalWrite(INa, HIGH);
    digitalWrite(INb, LOW);
    digitalWrite(INc, LOW);
    digitalWrite(INd, HIGH);
  }
  ledcWrite(servoChannel, steeringAngle);
  motors(leftSpeed, rightSpeed);
}

void receiveEvent(int x) {
  while (Wire.available()) {
    Wire.readBytes((uint8_t *)&Xpk, sizeof(float));
    if (isnan(Xpk)) {
      reverse = true;
    } else {
      reverse = false;
      if (Xpk > 100) {
        Xpk = 100;
      } else if (Xpk < -100) {
        Xpk = -100;
      }
      if (Xpk < 0) {
        lineIs = 'l';
      } else if (Xpk > 0) {
        lineIs = 'r';
      }
    }
    Serial.println(Xpk);
  }
}
