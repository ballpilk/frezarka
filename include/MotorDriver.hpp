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
        linearForward(400, 16, micros());
  }
  void manualBackWard()
  {
      if(numberOfStepsToTake==0)
        linearBackward(400, 16, micros());
  }
  void linearForward(long unsigned int steps, long unsigned int separation, long unsigned int startTime)
  {
    numberOfStepsToTake = steps;
    lastStepTime = startTime;
    stepTimeSeparation = separation;
    direction = HIGH;
  }
  void linearBackward(long unsigned int steps, long unsigned int separation, long unsigned int startTime)
  {
    numberOfStepsToTake = steps;
    lastStepTime = startTime;
    stepTimeSeparation = separation;
    direction = LOW;
  }
  bool onTime(long unsigned int microsecond)
  {
    if(numberOfStepsToTake>0)
    {
      if(microsecond - lastStepTime >= stepTimeSeparation)
      {
        mot.step(direction);
        --numberOfStepsToTake;
        lastStepTime = microsecond;
      }
    }
    return numberOfStepsToTake == 0;
  }
private:
  long unsigned int lastStepTime=0;
  long unsigned int numberOfStepsToTake=0;
  long unsigned int stepTimeSeparation=0;
  int direction = LOW;
  Motor mot;
};
