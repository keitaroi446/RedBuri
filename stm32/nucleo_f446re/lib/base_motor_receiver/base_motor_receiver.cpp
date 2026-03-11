#include "base_motor_receiver.hpp"
#include "usart.h"
#include <cstdio>

void BaseMotorReceiver::init()
{
    HAL_UART_Receive_IT(&huart2, &rx_byte_, 1);
}

void BaseMotorReceiver::callback()
{
    const char c = static_cast<char>(rx_byte_);

    if(c == '\r')
    {
        HAL_UART_Receive_IT(&huart2, &rx_byte_, 1);
        return;
    }

    if(c == '\n')
    {
        if(len_ > 0)
        {
            buf_[len_] = '\0';

            float rpm = 0.0f;
            float steer = 0.0f;
            if(std::sscanf(buf_, "B,%f,%f", &rpm, &steer) == 2)
            {
                motor_rpm_ = rpm;
                target_steer_deg_ = steer;
            }
        }

        len_ = 0;
        HAL_UART_Receive_IT(&huart2, &rx_byte_, 1);
        return;
    }

    if(len_ < BUF_SIZE - 1)
    {
        buf_[len_++] = c;
    }
    else
    {
        len_ = 0;
    }

    HAL_UART_Receive_IT(&huart2, &rx_byte_, 1);
}
