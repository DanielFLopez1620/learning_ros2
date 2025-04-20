#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>

namespace diff
{
    class EncoderDriver
    {
    private:
        int enc_a_{0};
        int enc_b_{0};

        volatile int pos_i_{0};

    public:
        EncoderDriver(const int& enc_a, const int& enc_b)
            : enc_a_{enc_a}, enc_b_ {enc_b}
        {}

        void begin();
        int read();
        void IRAM_ATTR readEnc();
    };
}

#endif