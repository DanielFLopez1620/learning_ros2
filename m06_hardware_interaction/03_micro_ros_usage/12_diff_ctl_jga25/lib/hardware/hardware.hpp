#ifndef HARDWARE_HPP
#define HARDWARE_HPP

namespace diff
{
    struct HARDWARE
    {
        // ------------------- Motor Left --------------------------
        
        // Encoder Channel A
        static const unsigned int ML_ENCA = 32;
        // Encoder Channel B
        static const unsigned int ML_ENCB = 33;
        // Driver Forward Pin
        static const unsigned int ML_FORW = 22;
        // Driver Backward Pin
        static const unsigned int ML_BACW = 21;
        // Driver Enable Pin
        static const unsigned int ML_EN = 17;

        // ------------------- Motor RIGHT --------------------------
        
        // Check if it is valid, or exchange enc ports
        // Encoder Channel A
        static const unsigned int MR_ENCA = 35;
        // Encoder Channel B
        static const unsigned int MR_ENCB = 34;
        // Driver Forward Pin
        static const unsigned int MR_FORW = 19;
        // Driver Backward Pin
        static const unsigned int MR_BACW = 18;
        // Driver Enable Pin
        static const unsigned int MR_EN = 16;
    };
}

#endif