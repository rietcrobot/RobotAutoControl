#include <Arduino.h>
#include <config/parameter.h>

class Wheel
{
private:
    int _pinIN1 = -1;
    int _pinIN2 = -1;
    int _pinPWM = -1;
    int invert_dir = 0;
    int _ErrorSpeed = 0;

    //HardwareTimer *MyTim1 = new HardwareTimer(TIM2);
    //HardwareTimer *MyTim2 = new HardwareTimer(TIM3);

public:
    Wheel(int pinIN1, int pinIN2, int pinPWM, int ErrorSpeed)
    {
        _pinIN1 = pinIN1;
        _pinIN2 = pinIN2;
        _pinPWM = pinPWM;
        _ErrorSpeed = ErrorSpeed;
    }

    void Init()
    {
        pinMode(_pinIN1, OUTPUT);
        pinMode(_pinIN2, OUTPUT);
        pinMode(_pinPWM, OUTPUT);
    }

    void ControlMotorSpeed(int Speed)
    {
        if (Speed < 0)
        {
            digitalWrite(_pinIN1, 1);
            digitalWrite(_pinIN2, 0);
        }
        else if (Speed > 0)
        {
            digitalWrite(_pinIN1, 0);
            digitalWrite(_pinIN2, 1);
        }
        else
        {
            digitalWrite(_pinIN1, 0);
            digitalWrite(_pinIN2, 0);
        }

        int Speed_Motor = abs(Speed);
        

        //int Speed_Motor_ = map(Speed_Motor,0,255,SpeedMin,SpeedMax);

        if (Speed_Motor > SpeedMax)
        {
            Speed_Motor = SpeedMax;
        }
        if (Speed_Motor < SpeedMin)
        {
            Speed_Motor = SpeedMin;
        }

        Speed_Motor = Speed_Motor + _ErrorSpeed;

        if (Speed_Motor > 255)
        {
            Speed_Motor = 255;
        }
        if (Speed_Motor < 0)
        {
            Speed_Motor = 0;
        }

         analogWrite(_pinPWM, Speed_Motor);
    }
};