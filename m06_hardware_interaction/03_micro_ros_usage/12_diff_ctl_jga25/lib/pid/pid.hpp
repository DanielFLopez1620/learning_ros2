#ifndef PID_HPP
#define PID_HPP

namespace diff
{
    class ControlPID
    {
    private:
        int kp_{0};
        int kd_{0};
        int ki_{0};
        int ko_{0};
        int pwm_max_{0};
        int pwm_min_{0};
        bool enabled_{false};
        int setpoint_{0};
        int integral_term_{0};
        long last_enc_count_{0};
        int last_input_{0};
        long last_output_{0};
    public:
        ControlPID(int kp, int kd, int ki, int ko, 
            int pwm_max, int pwm_min)
            : kp_{kp}, kd_ {kd}, ko_{ko}, pwm_min_{pwm_min}, pwm_max_(pwm_max)
            {}
        
        void compute(int enc_count, int& computed_output);
        
        void disable();
        
        void enable();
        
        bool enabled();
        
        void reset(int enc_count);

        void setSetpoint(int setpoint);

        void setTunings(int kp, int kd, int ki, int ko);
    };
}

#endif