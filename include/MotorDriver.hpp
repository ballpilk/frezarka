#pragma once
#include "Motor.hpp"

class MotorDriver{
public:
  MotorDriver(int stepPin, int dirPin)
  :mot(stepPin, dirPin)
  {}
  void manualForward()
  {
      if(numberOfStepsToTake==0)
        linearForward(400, 64, micros());
  }
  void manualBackWard()
  {
      if(numberOfStepsToTake==0)
        linearBackward(400, 64, micros());
  }
  void linearForward(long unsigned int steps, unsigned int separation, long unsigned int startTime)
  {
    numberOfStepsToTake = steps;
    lastStepTime = startTime;
    stepTimeSeparation = separation;
    direction = HIGH;
    mot.setDir(HIGH);
  }
  void linearBackward(long unsigned int steps, unsigned int separation, long unsigned int startTime)
  {
    numberOfStepsToTake = steps;
    lastStepTime = startTime;
    stepTimeSeparation = separation;
    direction = LOW;
    mot.setDir(LOW);
  }
  bool onTime(long unsigned int microsecond)
  {
    if(numberOfStepsToTake>0)
    {
      int distance = microsecond - lastStepTime;
      if(distance >= stepTimeSeparation)
      {
        mot.step();
        --numberOfStepsToTake;
        lastStepTime = microsecond;
      }
    }
    return numberOfStepsToTake == 0;
  }
private:
  long int maxError = 0;
  long unsigned int lastStepTime=0;
  long unsigned int numberOfStepsToTake=0;
  unsigned int stepTimeSeparation=0;
  int direction = LOW;
  Motor mot;
};
