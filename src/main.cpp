#include <Arduino.h>
#include <AccelStepper.h>
#include <math.h>
#include <stdio.h>

const int pinTx = 5;
const int pinRx = 6;

// X motor
const int pinStepX = 11;
const int pinDirX = 12;

// Y motor
const int pinStepY = 9;
const int pinDirY = 10;

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

// Match the direction settings used in your jog tester.
// Positive movement must go AWAY from the 0,0 corner.
const bool reverseX = false;
const bool reverseY = false;

const long maxPositionX = 950;
const long maxPositionY = 920;

const float maxSpeedX = 700.0;
const float maxSpeedY = 700.0;
const float acceleration = 2000.0;

const unsigned long signalTimeout = 300;

int joystickX = 0;
int joystickY = 0;

float currentSpeedX = 0;
float currentSpeedY = 0;

char boardbuffer[24];
int bufferPosition = 0;
bool discardLine = false;

bool receivedMessage = false;
bool movementEnabled = false;

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

void stopMotors() {
  currentSpeedX = 0;
  currentSpeedY = 0;

  xAxis.setSpeed(0);
  yAxis.setSpeed(0);
}

// Every step passes through these travel checks
void runMotors() {
  long positionX = xAxis.currentPosition();
  long positionY = yAxis.currentPosition();

  if ((positionX <= 0 && xAxis.speed() < 0) ||
      (positionX >= maxPositionX && xAxis.speed() > 0)) {
    currentSpeedX = 0;
    xAxis.setSpeed(0);
  }

  if ((positionY <= 0 && yAxis.speed() < 0) ||
      (positionY >= maxPositionY && yAxis.speed() > 0)) {
    currentSpeedY = 0;
    yAxis.setSpeed(0);
  }

  xAxis.runSpeed();
  yAxis.runSpeed();
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

          // A long gap requires centering again
          if (!receivedMessage ||
              millis() - lastReceiveTime > signalTimeout) {
            movementEnabled = false;
            stopMotors();
          }

          joystickX = receivedX;
          joystickY = receivedY;

          receivedMessage = true;
          lastReceiveTime = millis();

          if (!movementEnabled &&
              joystickX == 0 && joystickY == 0) {
            movementEnabled = true;
          }

          // No coasting when an axis enters its dead zone
          if (joystickX == 0) {
            currentSpeedX = 0;
            xAxis.setSpeed(0);
          }

          if (joystickY == 0) {
            currentSpeedY = 0;
            yAxis.setSpeed(0);
          }
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

    runMotors();
  }
}

void setup() {
  Serial1.begin(115200, SERIAL_8N1, pinRx, pinTx);

  xAxis.setPinsInverted(reverseX, false, false);
  yAxis.setPinsInverted(reverseY, false, false);

  xAxis.setMaxSpeed(maxSpeedX);
  yAxis.setMaxSpeed(maxSpeedY);

  xAxis.setMinPulseWidth(5);
  yAxis.setMinPulseWidth(5);

  // The head MUST physically start at your 0,0 corner
  xAxis.setCurrentPosition(0);
  yAxis.setCurrentPosition(0);

  stopMotors();
  lastSpeedUpdate = micros();
}

void loop() {
  // Stop before reading more data if communication was lost
  if (!receivedMessage ||
      millis() - lastReceiveTime > signalTimeout) {
    movementEnabled = false;
    stopMotors();
  }

  readJoystick();

  if (!movementEnabled) {
    stopMotors();
    return;
  }

  unsigned long now = micros();
  unsigned long elapsed = now - lastSpeedUpdate;

  if (elapsed >= 1000) {
    lastSpeedUpdate = now;

    float seconds = fminf(elapsed / 1000000.0f, 0.02f);
    float speedChange = acceleration * seconds;

    float targetSpeedX = joystickX / 127.0f * maxSpeedX;
    float targetSpeedY = joystickY / 127.0f * maxSpeedY;

    // Block outward commands at each limit
    if ((xAxis.currentPosition() <= 0 && targetSpeedX < 0) ||
        (xAxis.currentPosition() >= maxPositionX &&
         targetSpeedX > 0)) {
      targetSpeedX = 0;
    }

    if ((yAxis.currentPosition() <= 0 && targetSpeedY < 0) ||
        (yAxis.currentPosition() >= maxPositionY &&
         targetSpeedY > 0)) {
      targetSpeedY = 0;
    }

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

  runMotors();
}