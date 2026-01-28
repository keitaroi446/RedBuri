#pragma once

#include "usart.h"
#include <string.h>

namespace log
{
    inline void write(const char* s)
    {
        if(!s) return;
        HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t*>(const_cast<char*>(s)),
                          strlen(s), HAL_MAX_DELAY);
    }
}
