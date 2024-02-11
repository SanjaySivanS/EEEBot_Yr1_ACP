#include <Wire.h>
#include <Math.h>
#include <PID_v1.h>

#define FLS_IR 35
#define NLS_IR 26
#define M_IR 25
#define NRS_IR 33
#define FRS_IR 32

int IR_Value[] = {0, 0, 0, 0, 0};
const float mapFLS[] = {1000, 2250};
const float mapNLS[] = {1000, 2200};
const float mapM[] = {1000, 2250};
const float mapNRS[] = {1000, 2400};
const float mapFRS[] = {1800, 2600};

const int numBytes = 5;
float invertedValues[numBytes];
float valSi[] = {-25.5, -12.5, 0, 12.5, 25.5};
float valSX[numBytes];
float sumSX = 0;
float sumS = 0;
float xPk;

double Setpoint, Input, Output;
double Kp = 0.5;
double Ki = 5;
double Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

float steerAngle;

void setup() {
  Wire.begin(); 
  Input = xPk;
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  //-------------------------------------------------------
  IR_Value[4] = analogRead(FLS_IR);
  IR_Value[3] = analogRead(NLS_IR);
  IR_Value[2] = analogRead(M_IR);
  IR_Value[1] = analogRead(NRS_IR);
  IR_Value[0] = analogRead(FRS_IR);
  //-------------------------------------------------------

  invertedValues[numBytes];
  for (int i = 0; i < numBytes; i++) {
    invertedValues[i] = 4095 - IR_Value[i];
  }

  WeightedAverage();
  sumSX = 0;
  sumS = 0;

  PIDController();

  delay(50);
}

void WeightedAverage() {
  for (int i = 0; i < numBytes; i++) {
    valSX[i] = invertedValues[i] * valSi[i];
    sumSX = sumSX + valSX[i];
  }

  for (int i = 0; i < numBytes; i++) {
    sumS = sumS + invertedValues[i];
  }

  if (sumS != 0) { // Avoid division by zero
    xPk = sumSX / sumS;
  } else {
    // Handle error condition
  }
}

void PIDController() {
  Input = Setpoint - xPk;
  myPID.Compute();
  if (xPk < 0) {
    steerAngle = 325 - Output;
  } else {
    steerAngle = 325 + Output;
  }
  
  sendSteerAngle(steerAngle); // Send steerAngle to slave ESP32 via I2C
}

void sendSteerAngle(float angle) {
  Wire.beginTransmission(8);  // Address of the slave ESP32
  Wire.write((uint8_t*)&angle, sizeof(angle)); // Write the steerAngle to the I2C bus
  Wire.endTransmission();  // End the transmission
}
