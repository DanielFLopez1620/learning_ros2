#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

namespace diff
{
    struct ROBOT_CONST
    {
        static const int PID_RATE {30};
        static const int PID_T {1000 / PID_RATE};
        static const int PID_KP {20};
        static const int PID_KD {12};
        static const int PID_KI {1};
        static const int PID_KO {50};
        static const int PWM_MAX {255};
        static const int PWM_MIN {-255};
    };
}

#endif