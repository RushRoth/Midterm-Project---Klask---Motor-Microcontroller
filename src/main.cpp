#include <Arduino.h>
#include <AccelStepper.h>
#include <math.h>
#include <stdio.h>

const int pinTx = 5;
const int pinRx = 6;

// Y motor
const int pinStepY = 9;
const int pinDirY = 10;

// X motor
const int pinStepX = 11;
const int pinDirX = 12;

AccelStepper xAxis(
  AccelStepper::DRIVER,
  pinStepX,
  pinDirX
);

AccelStepper yAxis(
  AccelStepper::DRIVER,
  pinStepY,
  pinDirY
);

// STEP pulses per second
const float maxSpeedX = 700.0;
const float maxSpeedY = 700.0;

// Speed change in steps/second per second
const float acceleration = 2000.0;

// Change to -1 to reverse an axis
const int directionX = 1;
const int directionY = 1;

const unsigned long signalTimeout = 300;

int joystickX = 0;
int joystickY = 0;

float currentSpeedX = 0;
float currentSpeedY = 0;

char boardbuffer[24];
int bufferPosition = 0;
bool discardLine = false;

bool receivedMessage = false;

unsigned long lastReceiveTime = 0;
unsigned long lastSpeedUpdate = 0;

float approachSpeed(float current, float target, float change) {
  if (current < target) {
    return fminf(current + change, target);
  }

  if (current > target) {
    return fmaxf(current - change, target);
  }

  return target;
}

void readJoystick() {
  int bytesRead = 0;

  while (Serial1.available() && bytesRead < 32) {
    char c = Serial1.read();
    bytesRead++;

    if (c == '\n') {
      if (!discardLine) {
        boardbuffer[bufferPosition] = '\0';

        int receivedX;
        int receivedY;
        char extra;

        int values = sscanf(
          boardbuffer,
          "%4d,%4d%c",
          &receivedX,
          &receivedY,
          &extra
        );

        if (values == 2 &&
            receivedX >= -127 && receivedX <= 127 &&
            receivedY >= -127 && receivedY <= 127) {
          joystickX = receivedX;
          joystickY = receivedY;

          receivedMessage = true;
          lastReceiveTime = millis();
        }
      }

      bufferPosition = 0;
      discardLine = false;

    } else if (c != '\r' && !discardLine) {
      if (bufferPosition < (int)sizeof(boardbuffer) - 1) {
        boardbuffer[bufferPosition++] = c;
      } else {
        discardLine = true;
      }
    }

    xAxis.runSpeed();
    yAxis.runSpeed();
  }
}

void setup() {
  Serial1.begin(115200, SERIAL_8N1, pinRx, pinTx);

  xAxis.setMaxSpeed(maxSpeedX);
  yAxis.setMaxSpeed(maxSpeedY);

  xAxis.setMinPulseWidth(5);
  yAxis.setMinPulseWidth(5);

  xAxis.setSpeed(0);
  yAxis.setSpeed(0);

  lastSpeedUpdate = micros();
}

void loop() {
  readJoystick();

  bool signalOK =
    receivedMessage &&
    millis() - lastReceiveTime <= signalTimeout;

  if (!signalOK) {
    joystickX = 0;
    joystickY = 0;

    currentSpeedX = 0;
    currentSpeedY = 0;

    xAxis.setSpeed(0);
    yAxis.setSpeed(0);
  }

  unsigned long now = micros();
  unsigned long elapsed = now - lastSpeedUpdate;

  if (elapsed >= 1000) {
    lastSpeedUpdate = now;

    float seconds = fminf(elapsed / 1000000.0f, 0.02f);
    float speedChange = acceleration * seconds;

    float targetSpeedX =
      directionX * joystickX / 127.0f * maxSpeedX;

    float targetSpeedY =
      directionY * joystickY / 127.0f * maxSpeedY;

    currentSpeedX = approachSpeed(
      currentSpeedX,
      targetSpeedX,
      speedChange
    );

    currentSpeedY = approachSpeed(
      currentSpeedY,
      targetSpeedY,
      speedChange
    );

    xAxis.setSpeed(currentSpeedX);
    yAxis.setSpeed(currentSpeedY);
  }

  xAxis.runSpeed();
  yAxis.runSpeed();
}