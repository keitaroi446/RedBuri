#include <algorithm>
#include <cstdio>
#include "uart_sender.hpp"

namespace
{
constexpr float kRadScale = 1000.0f;
}

UartSender::UartSender(UART_HandleTypeDef& huart, TIM_HandleTypeDef& htim)
    : huart_(huart), htim_(htim)
{}

void UartSender::init()
{
    HAL_TIM_Base_Start_IT(&htim_);
}

void UartSender::onTimerTick()
{
    tick_pending_ = true;
}

void UartSender::sendJointDeg(float joint_1_rad,
                              float joint_2_rad,
                              float joint_3_rad,
                              float joint_4_rad,
                              float joint_5_rad,
                              float joint_6_rad,
                              float gripper_rad)
{
    if(!tick_pending_)
    {
        return;
    }

    tick_pending_ = false;

    const int32_t joint_1_scaled = static_cast<int32_t>(joint_1_rad * kRadScale);
    const int32_t joint_2_scaled = static_cast<int32_t>(joint_2_rad * kRadScale);
    const int32_t joint_3_scaled = static_cast<int32_t>(joint_3_rad * kRadScale);
    const int32_t joint_4_scaled = static_cast<int32_t>(joint_4_rad * kRadScale);
    const int32_t joint_5_scaled = static_cast<int32_t>(joint_5_rad * kRadScale);
    const int32_t joint_6_scaled = static_cast<int32_t>(joint_6_rad * kRadScale);
    const int32_t gripper_scaled = static_cast<int32_t>(gripper_rad * kRadScale);

    const int written = std::snprintf(tx_buf_,
                                      TX_BUF_SIZE,
                                      "J,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n",
                                      static_cast<long>(joint_1_scaled),
                                      static_cast<long>(joint_2_scaled),
                                      static_cast<long>(joint_3_scaled),
                                      static_cast<long>(joint_4_scaled),
                                      static_cast<long>(joint_5_scaled),
                                      static_cast<long>(joint_6_scaled),
                                      static_cast<long>(gripper_scaled));
    if(written <= 0)
    {
        return;
    }

    const size_t tx_len = std::min(static_cast<size_t>(written), TX_BUF_SIZE - 1U);
    HAL_UART_Transmit(&huart_, reinterpret_cast<uint8_t*>(tx_buf_), tx_len, HAL_MAX_DELAY);
}
