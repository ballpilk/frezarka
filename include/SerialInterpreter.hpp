#pragma once

#include "MotorCoordinator.hpp"

struct SerialBuffer{
  char buffer[128];
  int pos=0;
  void append(char letter)
  {
    buffer[pos] = letter;
    pos = (pos+1)%128;
  }
  void reset()
  {
    pos = 0;
  }
  char& operator[](unsigned int i)
  {
    return buffer[i];
  }
};

class SerialInterpretter
{
public:
  SerialInterpretter(MotorCoordinator& coord)
  :coordinator(coord)
  {}
  void processCommand(const SerialBuffer& buff)
  {
    int commandNumber = getNumber( 'G');
    switch(commandNumber)
    {
        case 1: //line
        {
          coordinator.line(getNumber('X')*stepsPerMM, getNumber('Y')*stepsPerMM, getNumber('Z')*stepsPerMM, getNumber('F')*stepsPerMM);
          break;
        }
        case 3: //arc right
        {
          coordinator.archR(getNumber('X')*stepsPerMM,
                          getNumber('Y')*stepsPerMM,
                          getNumber('Z')*stepsPerMM,
                          getNumber('I')*stepsPerMM,
                          getNumber('J')*stepsPerMM,
                          getNumber('F')*stepsPerMM/10);
          break;
        }
        case 4: //arc left
        {
          coordinator.archL(getNumber('X')*stepsPerMM,
                          getNumber('Y')*stepsPerMM,
                          getNumber('Z')*stepsPerMM,
                          getNumber('I')*stepsPerMM,
                          getNumber('J')*stepsPerMM,
                          getNumber('F')*stepsPerMM/10);
          break;
        }
    }
  }
  void putChar(char letter)
  {
    buff.append(letter);
    Serial.write(letter);
    if(letter == '\n' or letter == '\r')
    {
      buff.append('\0');
      processCommand(buff);
      buff.reset();
      Serial.write(">\n");
    }
  }
private:
  const unsigned int stepsPerMM = 1600;
  double getNumber(char code)
  {
    for(int i=0; i < buff.pos;++i)
    {
      if(buff[i] == code)
        return atof(&buff.buffer[i+1]);
    }
    return 0.;
  }
  SerialBuffer buff;
  MotorCoordinator& coordinator;
};
