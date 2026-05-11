#include <AFMotor.h>

/*
 * SCOUT Rover - simplified combined Mega sketch
 *
 * Goals:
 * - keep motor behavior as close as possible to the original MOTOR.cpp
 * - stream raw gas data only
 * - leave all conversion / threshold / topic logic for the PC node
 *
 * Motor input:
 *   v <linear> <angular>
 *
 * Gas output:
 *   g <mq2_raw> <mq3_raw>
 *
 * Gas lines are sent slowly and only when the motor command path is idle,
 * so driving stays the priority.
 */

AF_DCMotor motorLeft(1, MOTOR12_64KHZ);
AF_DCMotor motorRight(2, MOTOR12_64KHZ);

const uint8_t MQ2_PIN = A11;
const uint8_t MQ3_PIN = A9;

float leftTrim = 1.0f;
float rightTrim = 1.0f;

unsigned long lastCommandMillis = 0;
unsigned long lastGasMillis = 0;

const unsigned long TIMEOUT_MS = 500;
const unsigned long GAS_INTERVAL_MS = 1000;
const unsigned long GAS_IDLE_WINDOW_MS = 250;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);
  Stop();
}

void loop() {
  if (Serial.available() > 0) {
    char startChar = Serial.read();

    if (startChar == 'v') {
      float currentLin = Serial.parseFloat();
      float currentAng = Serial.parseFloat();

      float leftPower = (currentLin - currentAng) * 255.0f * leftTrim;
      float rightPower = (currentLin + currentAng) * 255.0f * rightTrim;

      updateMotors(leftPower, rightPower);
      lastCommandMillis = millis();
    }
  }

  if (millis() - lastCommandMillis > TIMEOUT_MS) {
    Stop();
  }

  if (shouldSendGasSample()) {
    sendGasSample();
  }
}

bool shouldSendGasSample() {
  unsigned long now = millis();

  if (now - lastGasMillis < GAS_INTERVAL_MS) {
    return false;
  }

  if (now - lastCommandMillis < GAS_IDLE_WINDOW_MS) {
    return false;
  }

  return true;
}

int readAveragedAnalog(uint8_t pin) {
  const uint8_t samples = 4;
  long sum = 0;

  for (uint8_t i = 0; i < samples; ++i) {
    sum += analogRead(pin);
  }

  return (int)(sum / samples);
}

void sendGasSample() {
  int mq2Raw = readAveragedAnalog(MQ2_PIN);
  int mq3Raw = readAveragedAnalog(MQ3_PIN);

  Serial.print(F("g "));
  Serial.print(mq2Raw);
  Serial.print(' ');
  Serial.println(mq3Raw);

  lastGasMillis = millis();
}

void updateMotors(float left, float right) {
  motorLeft.setSpeed(constrain(abs(left), 0, 255));
  motorRight.setSpeed(constrain(abs(right), 0, 255));

  if (left > 5) {
    motorLeft.run(BACKWARD);
  } else if (left < -5) {
    motorLeft.run(FORWARD);
  } else {
    motorLeft.run(RELEASE);
  }

  if (right > 5) {
    motorRight.run(BACKWARD);
  } else if (right < -5) {
    motorRight.run(FORWARD);
  } else {
    motorRight.run(RELEASE);
  }
}

void Stop() {
  motorLeft.run(RELEASE);
  motorRight.run(RELEASE);
}
