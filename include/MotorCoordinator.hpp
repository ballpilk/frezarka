#pragma once

#include "MotorDriver.hpp"
#include <math.h>
const double microsInSec = 1000000;
class MotorCoordinator
{
public:
MotorCoordinator(MotorDriver& x1, MotorDriver& x2, MotorDriver& y,  MotorDriver& z)
:X1(x1), X2(x2), Y(y), Z(z)
{}
  void line(long int x, long int y, long int z, unsigned int feed)//G01 X11585 Y04676 Z10000 F100.
  //all values in steps and steps/sec
  {
      if(feed==0)
        return;
      double dx = x-currentX;
      double dy = y-currentY;
      double dz = z-currentZ;
      double dxyz = sqrt(dx*dx+dy*dy+dz*dz);
      double roundsPerSecTimesDistance = microsInSec*dxyz/feed;
      Serial.println(roundsPerSecTimesDistance);
      unsigned int separationX = 0;
      unsigned int separationY = 0;
      unsigned int separationZ = 0;
      if(dxyz != 0)
      {
        if(dx != 0)
        {
          separationX = abs(roundsPerSecTimesDistance/dx);
        }
        if(dy != 0)
        {
          separationY = abs(roundsPerSecTimesDistance/dy);
        }
        if(dz != 0)
        {
          separationZ = abs(roundsPerSecTimesDistance/dz);
        }
        Serial.println(separationX);
        Serial.println(separationY);
        Serial.println(separationZ);
      }
      else
      {
        Serial.println("No Move");
        return;
      }
      unsigned long int starttime = micros();
      if (dx > 0)
      {
        X1.linearForward(dx, separationX, starttime);
        X2.linearForward(dx, separationX, starttime);
      }
      else
      {
        X1.linearBackward(-dx, separationX, starttime);
        X2.linearBackward(-dx, separationX, starttime);
      }
      if(dy > 0)
        Y.linearBackward(dy, separationY, starttime);//change in direction
      else
        Y.linearForward(-dy, separationY, starttime);
      if(dz > 0)
        Z.linearForward(dz, separationZ, starttime);
      else
        Z.linearBackward(-dz, separationZ, starttime);

      go();
//      Serial.println(X1.getMaxErr());
//      Serial.println(X2.getMaxErr());
//      Serial.println(Y.getMaxErr());
//      Serial.println(Z.getMaxErr());
      currentX = x;
      currentY = y;
      currentZ = z;
  }
  void archR(long int x, long int y, long int z, long int centerRelativeX, long int centerRelativeY, unsigned int feed)
  {//Z is linear!
    long int absoluteCenterX = currentX+centerRelativeX;
    long int absoluteCenterY = currentY+centerRelativeY;
    double radius = sqrt(double(centerRelativeX)*double(centerRelativeX)+double(centerRelativeY)*double(centerRelativeY));//converted to double because of overflow problem on large arches
//przejscie do ukladu biegunowego
    double fi1 = getFi(-centerRelativeX, -centerRelativeY);
    double fi2 = getFi(x-currentX-centerRelativeX, y-currentY-centerRelativeY);
    if (fi2 >= fi1)
      fi1 += 2*M_PI;
    double arcAngle = fi1-fi2;
    double arclength = arcAngle*radius;
    unsigned int sections = abs(arclength/20.);//20 steps = ~0,016mm
    double sectionAngle = arcAngle/sections;
    double dz = z-currentZ;
    long unsigned int startZ = currentZ;
    for(unsigned int s = 0; s < sections; ++s)
    {
      double currentfi = fi1 - s*sectionAngle;
      line(absoluteCenterX + getx(radius, currentfi), absoluteCenterY + gety(radius, currentfi), startZ+s*dz, feed);
    }
    line(x, y, z, feed);//just in case to finish off mitigating all numerical errors
  }

  void archL(long int x, long int y, long int z, long int centerRelativeX, long int centerRelativeY, unsigned int feed)
  {//Z is linear!
    long int absoluteCenterX = currentX+centerRelativeX;
    long int absoluteCenterY = currentY+centerRelativeY;
    double radius = sqrt(double(centerRelativeX)*double(centerRelativeX)+double(centerRelativeY)*double(centerRelativeY));//converted to double because of overflow problem on large arches
//przejscie do ukladu biegunowego
    double fi1 = getFi(-centerRelativeX,-centerRelativeY);
    double fi2 = getFi(x-currentX-centerRelativeX,y-currentY-centerRelativeY);
    if (fi2 <= fi1) //<= so we can do a full circle at one command
      fi2 +=2*M_PI;
    double arcAngle = fi2-fi1;
    double arclength = arcAngle*radius;
    unsigned int sections = abs(arclength/20);//20 steps = ~0,1mm
    double sectionAngle = arcAngle/double(sections);
    double dz = z-currentZ;
    long unsigned int startZ = currentZ;
    for(unsigned int s = 0; s < sections; ++s)
    {
      double currentfi = fi1 + s*sectionAngle;
      line(absoluteCenterX + getx(radius, currentfi), absoluteCenterY + gety(radius, currentfi), startZ+s*dz, feed);
    }
    line(x, y, z, feed);//just in case to finish off mitigating all numerical errors
  }


private:
  void go()
  {
    bool finished = false;
    while(!finished)
    {
      long unsigned int micro = micros();
      finished = X1.onTime(micro);
      finished = X2.onTime(micro) && finished;
      finished = Y.onTime(micro) && finished;
      finished = Z.onTime(micro) && finished;
    }
  }
  long int getx(double radius, double fi)
  {
    return radius*cos(fi);
  }
  long int gety(double radius, double fi)
  {
    return radius*sin(fi);
  }
  double getFi(double x, double y)
  {
    if (x > 0. && y>=0.)
      return atan(y/x);
    if(x > 0. && y < 0.)
      return atan(y/x)+2.*M_PI;
    if( x < 0.)
      return atan(y/x)+M_PI;
    if( x == 0. && y > 0.)
      return M_PI/2.;
    else
      return 3.*M_PI/2.;
  }

  long int currentX=0;
  long int currentY=0;
  long int currentZ=0;
  MotorDriver& X1;
  MotorDriver& X2;
  MotorDriver& Y;
  MotorDriver& Z;
};
