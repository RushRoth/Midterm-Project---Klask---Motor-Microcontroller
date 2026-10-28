#include <Arduino.h>
#include <AccelStepper.h>
#include <stdio.h>

int pinTx = 5;
int pinRx = 6;

// Y motor
int pinStepY = 9;
int pinDirY = 10;

// X motor
int pinStepX = 11;
int pinDirX = 12;

AccelStepper yAxis(1, pinStepY, pinDirY);
AccelStepper xAxis(1, pinStepX, pinDirX);

String boardbuffer = "";

int joystickX = 0;
int joystickY = 0;

// Maximum commanded speed in steps per second
float motorSpeed = 200.0;

// Change to -1 if an axis moves backward
int directionX = 1;
int directionY = 1;

unsigned long lastReceiveTime = 0;
const unsigned long signalTimeout = 300;

void setup() {
  Serial1.setPins(pinRx, pinTx);
  Serial1.begin(9600);

  pinMode(pinStepY, OUTPUT);
  pinMode(pinDirY, OUTPUT);

  pinMode(pinStepX, OUTPUT);
  pinMode(pinDirX, OUTPUT);

  xAxis.setMaxSpeed(700);
  yAxis.setMaxSpeed(700);

  xAxis.setSpeed(0);
  yAxis.setSpeed(0);
}

void loop() {
  while (Serial1.available()) {
    char c = Serial1.read();

    if (c == '\n') {
      int receivedX;
      int receivedY;
      char extra;

      int values = sscanf(
        boardbuffer.c_str(),
        "%d,%d%c",
        &receivedX,
        &receivedY,
        &extra
      );

      if (values == 2 &&
          receivedX >= -127 && receivedX <= 127 &&
          receivedY >= -127 && receivedY <= 127) {
        joystickX = receivedX;
        joystickY = receivedY;

        lastReceiveTime = millis();
      }

      boardbuffer = "";

    } else if (c != '\r') {
      boardbuffer += c;

      if (boardbuffer.length() > 24) {
        boardbuffer = "";
      }
    }
  }

  // Stop if valid messages stop arriving
  if (millis() - lastReceiveTime > signalTimeout) {
    joystickX = 0;
    joystickY = 0;
  }

  float speedX = directionX * joystickX / 127.0 * motorSpeed;
  float speedY = directionY * joystickY / 127.0 * motorSpeed;

  xAxis.setSpeed(speedX);
  yAxis.setSpeed(speedY);

  xAxis.runSpeed();
  yAxis.runSpeed();
}