#include "motor.hpp"
#include <Arduino.h>
#include <cmath>

namespace diff
{
    void MotorDriver::begin()
    {
        pinMode(this->enable_pin_, OUTPUT);
        pinMode(this->forw_pin_, OUTPUT);
        pinMode(this->back_pin_, OUTPUT);
    }

    void MotorDriver::set_speed(const int& speed)
    {
        unsigned int abs_speed = abs(speed);
        if(abs_speed > this->MAX_SPEED)
        {
            abs_speed = this->MAX_SPEED;
        }

        if(speed < 0)
        {
            digitalWrite(this->forw_pin_, HIGH);
            digitalWrite(this->back_pin_, LOW);
        }
        else
        {
            digitalWrite(this->forw_pin_, LOW);
            digitalWrite(this->back_pin_, HIGH);
        }

        analogWrite(this->enable_pin_, abs_speed);
    }
}