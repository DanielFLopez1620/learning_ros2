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
        static const unsigned int ML_FORW = 21;
        // Driver Backward Pin
        static const unsigned int ML_BACW = 22;
        // Driver Enable Pin
        static const unsigned int ML_EN = 17;

        // ------------------- Motor RIGHT --------------------------
        
        // Encoder Channel A
        static const unsigned int MR_ENCA = 34;
        // Encoder Channel B
        static const unsigned int MR_ENCB = 35;
        // Driver Forward Pin
        static const unsigned int MR_FORW = 18;
        // Driver Backward Pin
        static const unsigned int MR_BACW = 19;
        // Driver Enable Pin
        static const unsigned int MR_EN = 16;
    };
}

#endif