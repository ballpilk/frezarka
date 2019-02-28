#pragma once
#include <Arduino.h>
class Motor
{
public:
  Motor(int stepPin, int dirPin)
  :pin(stepPin), dir(dirPin)
  {
    pinMode(pin, OUTPUT);
    pinMode(dir, OUTPUT);
  }
  void setDir(int direction)
  {
    digitalWrite(dir, direction);
  }
  void step()
  {
    digitalWrite(pin, pinStates[++count%2]);
  }
private:
  unsigned int count = 0;
  int pinStates[2] = {HIGH, LOW};
  int pin = 0;
  int dir = 0;
};
