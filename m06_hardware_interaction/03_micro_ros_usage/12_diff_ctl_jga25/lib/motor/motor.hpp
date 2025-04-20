#ifndef MOTOR_HPP
#define MOTOR_HPP

namespace diff
{
    class MotorDriver
    {
    private:
        static constexpr int MAX_SPEED{255};
        unsigned int enable_pin_{0};
        unsigned int forw_pin_{0};
        unsigned int back_pin_{0};
    public:
        MotorDriver(const unsigned int& enable, const unsigned int& forward, const unsigned int& backward)
            : enable_pin_{enable}, forw_pin_{forward}, back_pin_{backward}
        {}

        void begin();
        void set_speed(int speed);
    };
}

#endif
