#pragma once
#include "c620_can.hpp"

extern C620CAN c620;

namespace run
{
    inline void setup()
    {
        c620.setCurrent(1, 1);
        c620.sendCurrents();
    }

    inline void loop()
    {
    
    }
}