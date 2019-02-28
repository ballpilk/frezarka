#include "MotorDriver.hpp"
#include "MotorDriver.hpp"

class ManualControl{
public:
  ManualControl(MotorDriver& x1, MotorDriver& x2, MotorDriver& y,  MotorDriver& z)
  :X1(x1), X2(x2), Y(y), Z(z)
  {
    pinMode(A0, INPUT_PULLUP);
    pinMode(A1, INPUT_PULLUP);
    pinMode(A2, INPUT_PULLUP);
    pinMode(A3, INPUT_PULLUP);
    pinMode(A4, INPUT_PULLUP);
    pinMode(A5, INPUT_PULLUP);
  }
  void check()
  {//A0-lewy A3-prawy A1-tyl A2-przod A4-góra A5-dół
    bool go = false;
    if(digitalRead(A3)==LOW)
    {
        Y.manualBackWard();
        go = true;
    }
    if(digitalRead(A0)==LOW)
    {
        Y.manualForward();
        go = true;
    }
    if(digitalRead(A4)==LOW)
    {
        Z.manualForward();
        go = true;
    }
    if(digitalRead(A5)==LOW)
    {
        Z.manualBackWard();
        go = true;
    }
    if(digitalRead(A2)==LOW)
    {
        X1.manualForward();
        X2.manualForward();
        go = true;
    }
    if(digitalRead(A1)==LOW)
    {
      X1.manualBackWard();
      X2.manualBackWard();
      go = true;
    }

    bool finished = false;
    while(go && !finished)
    {
      long unsigned int micro = micros();
      finished = X1.onTime(micro);
      finished = X2.onTime(micro) && finished;
      finished = Y.onTime(micro) && finished;
      finished = Z.onTime(micro) && finished;
    }

  }
private:
  MotorDriver& X1;
  MotorDriver& X2;
  MotorDriver& Y;
  MotorDriver& Z;
};
