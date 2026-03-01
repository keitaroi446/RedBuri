#pragma once

#include <cstdint>
#include <cstddef>
#include "usart.h"

class BaseMsgReceiver
{
public:
    explicit BaseMsgReceiver(UART_HandleTypeDef &huart);
    void init();
    void readMsg();
    float getSteerDeg() const;
    float getMotorFRpm() const;
    float getMotorRRpm() const;
    float getMotorLRpm() const;

private:
    static constexpr size_t RX_BUF_SIZE = 64;

    UART_HandleTypeDef &huart_;
    uint8_t msg_{};
    char rx_buf_[RX_BUF_SIZE]{};
    size_t rx_len_{};
    float steer_deg_{};
    float motor_f_rpm_{};
    float motor_r_rpm_{};
    float motor_l_rpm_{};
};
