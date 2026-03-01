#include "base_msg_receiver.hpp"

#include <cstdio>
#include <cstring>

BaseMsgReceiver::BaseMsgReceiver(UART_HandleTypeDef &huart)
    : huart_(huart)
{}

void BaseMsgReceiver::init()
{
    HAL_UART_Receive_IT(&huart_, &msg_, 1);
}

void BaseMsgReceiver::readMsg()
{
    if (msg_ == '\n')
    {
        rx_buf_[rx_len_] = '\0';

        char msg_type{};
        float steer{};
        float motor_f{};
        float motor_r{};
        float motor_l{};

        const int parsed = std::sscanf(
            rx_buf_,
            "%c,%f,%f,%f,%f",
            &msg_type,
            &steer,
            &motor_f,
            &motor_r,
            &motor_l
        );

        if (parsed == 5 && msg_type == 'B')
        {
            steer_deg_ = steer;
            motor_f_rpm_ = motor_f;
            motor_r_rpm_ = motor_r;
            motor_l_rpm_ = motor_l;
        }

        rx_len_ = 0;
        std::memset(rx_buf_, 0, sizeof(rx_buf_));
    }
    else
    {
        if (rx_len_ < RX_BUF_SIZE - 1)
        {
            rx_buf_[rx_len_++] = static_cast<char>(msg_);
        }
        else
        {
            rx_len_ = 0;
            std::memset(rx_buf_, 0, sizeof(rx_buf_));
        }
    }

    HAL_UART_Receive_IT(&huart_, &msg_, 1);
}

float BaseMsgReceiver::getSteerDeg() const
{
    return steer_deg_;
}

float BaseMsgReceiver::getMotorFRpm() const
{
    return motor_f_rpm_;
}

float BaseMsgReceiver::getMotorRRpm() const
{
    return motor_r_rpm_;
}

float BaseMsgReceiver::getMotorLRpm() const
{
    return motor_l_rpm_;
}
