#include "ManualControl.hpp"
#include "MotorCoordinator.hpp"
#include "SerialInterpreter.hpp"


unsigned long int tim = 0;
MotorDriver X1(8,9);
MotorDriver X2(6,7);
MotorDriver Y(4,5);
MotorDriver Z(2,3);
ManualControl manual(X1,X2,Y,Z);
MotorCoordinator coordinator(X1,X2,Y,Z);
SerialInterpretter serialInterpret(coordinator);


int count = 0;

void setup() {
  Serial.begin(9600);
  Serial.write(">\n");
}

void loop() {
  if(Serial.available())
  {
    char letter = Serial.read();
    serialInterpret.putChar(letter);
  }
  if (millis() > tim)
  {
    digitalWrite(13, ++count%2==0?HIGH:LOW);
    tim+=1000;
  }
  manual.check();
}
