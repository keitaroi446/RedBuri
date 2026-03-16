#pragma once

#include <cstddef>
#include <cstdint>
#include "tim.h"
#include "usart.h"

class UartSender
{
public:
    UartSender(UART_HandleTypeDef& huart, TIM_HandleTypeDef& htim);
    void init();
    void onTimerTick();
    void sendJointDeg(float joint_1_rad,
                      float joint_2_rad,
                      float joint_3_rad,
                      float joint_4_rad,
                      float joint_5_rad,
                      float joint_6_rad,
                      float gripper_rad);

private:
    static constexpr size_t TX_BUF_SIZE = 128U;

    UART_HandleTypeDef& huart_;
    TIM_HandleTypeDef& htim_;
    volatile bool tick_pending_{false};
    char tx_buf_[TX_BUF_SIZE]{};
};
