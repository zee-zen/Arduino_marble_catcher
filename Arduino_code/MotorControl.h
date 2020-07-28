#ifndef MotorControl_h
#define MotorControl_h

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
  #endif
#include <Wire.h>

class MotorControl
{
public:
 MotorControl(int pinFwd,int pinRev,int pinPwm);

 void SpeedWrite(int fwdrev, int Speed);
 void SpeedStepUp(int PwmStepUp);
 void SpeedStepDown(int PwmStepDown);

 char Direction();
 char _Direction;

 int SpeedRead();

private:
 int _pinFwd;
 int _pinRev;
 int _pinPwm;
 int _Pwm;
 int _d;
 int _PwmStepUp;
 int _PwmStepDown;
 int _ConstrainPwm;
};
#endif
