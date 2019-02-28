#pragma once

#include "MotorDriver.hpp"
#include <math.h>
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
      long int dx = x-currentX;
      long int dy = y-currentY;
      long int dz = z-currentZ;

      double dxyz = sqrt(double(dx*dx+dy*dy+dz*dz));
      unsigned long int starttime = micros();

      if(dxyz != 0)
      {
        if(dx != 0)
        {
          long unsigned int separationX = abs(dxyz/((double)feed*dx));
          if (dx > 0)
          {
            X1.linearForward(dx, separationX, starttime);
            X2.linearForward(dx, separationX, starttime);
          }
          else if (dx < 0)
          {
            X1.linearBackward(-dx, separationX, starttime);
            X2.linearBackward(-dx, separationX, starttime);
          }
        }
        if(dy != 0)
        {
          long unsigned int separationY = abs(dxyz/((double)feed*dy));
          if(dy > 0)
            Y.linearBackward(dy, separationY, starttime);//change in direction
          else if (dy < 0)
            Y.linearForward(-dy, separationY, starttime);
        }
        if(dz != 0)
        {
          long unsigned int separationZ = abs(dxyz/((double)feed*dz));
          if(dz > 0)
            Z.linearForward(dz, separationZ, starttime);
          else if (dz < 0)
            Z.linearBackward(-dz, separationZ, starttime);
        }
      }
      bool finished = false;
      while(!finished)
      {
        long unsigned int micro = micros();
        finished = X1.onTime(micro);
        finished = X2.onTime(micro) && finished;
        finished = Y.onTime(micro) && finished;
        finished = Z.onTime(micro) && finished;
      }
      currentX = x;
      currentY = y;
      currentZ = z;
  }
  void archR(long int x, long int y, long int z, long int centerRelativeX, long int centerRelativeY, unsigned int feed)
  {//Z is linear!
    long int absoluteCenterX = currentX+centerRelativeX;
    long int absoluteCenterY = currentY+centerRelativeY;
    double radius = sqrt((currentX-absoluteCenterX)*(currentX-absoluteCenterX)+(currentY-absoluteCenterY)*(currentY-absoluteCenterY));
    double radius2 = sqrt((x-absoluteCenterX)*(x-absoluteCenterX)+(y-absoluteCenterY)*(y-absoluteCenterY));
    if (abs(radius - radius2) > 2)
    {
      Serial.print("Variable radius arcs are not supporrted yet.");
      return;
    }
//przejscie do ukladu biegunowego
    double fi1 = getFi(-centerRelativeX,-centerRelativeY);
    double fi2 = getFi(x-currentX-centerRelativeX,y-currentY-centerRelativeY);
    if (fi2 >= fi1)
      fi1 +=2*M_PI;
    double arcAngle = fi1-fi2;
    double arclength = arcAngle*radius;
    unsigned int sections = abs(arclength/20);//20 steps = ~0,016mm
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
    double radius = sqrt((currentX-absoluteCenterX)*(currentX-absoluteCenterX)+(currentY-absoluteCenterY)*(currentY-absoluteCenterY));
    double radius2 = sqrt((x-absoluteCenterX)*(x-absoluteCenterX)+(y-absoluteCenterY)*(y-absoluteCenterY));
    if (abs(radius - radius2) > 2)
    {
      Serial.print("Variable radius arcs are not supporrted yet.");
      return;
    }
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
