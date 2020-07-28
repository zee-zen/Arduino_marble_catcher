
#include "MotorControl.h"

MotorControl::MotorControl(int pinFwd,int pinRev, int pinPwm)
{
 pinMode(pinFwd, OUTPUT);
 pinMode(pinRev, OUTPUT);
 _pinFwd = pinFwd;
 _pinRev = pinRev;
 _pinPwm = pinPwm;
}

void MotorControl::SpeedWrite(int d,int Pwm)
{
 _Pwm = Pwm;
 _d=d;

 // Check if PWM value do not exceed the boundaries (0 to 255).
 _Pwm = constrain(_Pwm, 0, 255);

 // Is the PWM value positive, if yes go forward.
 if (_Pwm >= 64 && _Pwm <= 255)
 {
   if (_d==1)
   {
   digitalWrite(_pinFwd, HIGH );
   digitalWrite(_pinRev, LOW);
   analogWrite (_pinFwd, _Pwm);
   _Direction = 'F';
   }
   else if (_d==0)
   {
   digitalWrite(_pinFwd, LOW );
   digitalWrite(_pinRev, HIGH);
   analogWrite (_pinFwd, _Pwm);
   _Direction = 'R';
   }
 }

 // Is the PWM value is lower limit, if yes the motor is braked.
 else if (_Pwm ==64)
 {
   digitalWrite(_pinFwd, HIGH);
   digitalWrite(_pinRev, HIGH);
   analogWrite (_pinFwd, _Pwm);
   _Direction = 'B';
 }
// Is the PWM value is 0, if yes the motor is spooling or idle.
 else if (_Pwm ==0)
 {
   digitalWrite(_pinFwd, HIGH);
   digitalWrite(_pinRev, LOW);
   analogWrite (_pinFwd, _Pwm);
   _Direction = 'I';
 }
}

void MotorControl::SpeedStepUp(int PwmStepUp)
{
 _PwmStepUp = PwmStepUp;

 _Pwm = _Pwm + _PwmStepUp;

 this->SpeedWrite(_d,_Pwm);
}

void MotorControl::SpeedStepDown(int PwmStepDown)
{
 _PwmStepDown = PwmStepDown;

 _Pwm = _Pwm - _PwmStepDown;

 this->SpeedWrite(_d,_Pwm);
}

int MotorControl::SpeedRead()
{
 return _Pwm;
}

char MotorControl::Direction()
{
 return _Direction;
}
