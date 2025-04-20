#include "pid.hpp"

namespace diff
{
    void ControlPID::compute(int enc_count, int& computed_output)
    {
        if(!this->enabled_)
        {
            if(this->last_input_ != 0)
            {
                reset(enc_count);
            }
            return;
        }

        int input = enc_count - this->last_enc_count_;
        long err = this->setpoint_ - input;

        long output = (this->kp_ * err - this->kd_ * (input - this->last_input_)
            + this->integral_term_) / this->ko_;
        
        output += this->last_output_;

        if(output > this->pwm_max_)
        {
            output = this->pwm_max_;
        }
        else if (output < this->pwm_min_)
        {
            output = this->pwm_min_;
        }
        else
        {
            this->integral_term_ += this->ki_ * err;
        }

        computed_output = output;

        this->last_enc_count_ = enc_count;
        this->last_input_ = input;
        this->last_output_ = output;
    }
        
    void ControlPID::disable()
    {
        this->enabled_ = false;
    }
    
    void ControlPID::enable()
    {
        this->enabled_ = true;
    }
    
    bool ControlPID::enabled()
    {
        return this->enabled_;
    }
    
    void ControlPID::reset(int enc_count)
    {
        this->setpoint_ = 0;
        this->integral_term_ = 0;
        this->last_enc_count_ = enc_count;
        this->last_input_ = 0;
        this->last_output_ = 0;
    }

    void ControlPID::setSetpoint(int setpoint)
    {
        this->setpoint_ = setpoint;
    }

    void ControlPID::setTunings(int kp, int kd, int ki, int ko)
    {
        this->kp_ = kp;
        this->kd_ = kd;
        this->ki_ = ki;
        this->ko_ = ko;
    }
}
