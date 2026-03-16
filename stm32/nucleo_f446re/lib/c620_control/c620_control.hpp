#pragma once

#include <cstdint>
#include "tim.h"
#include "c620_can.hpp"

class C620Control
{
public:
    explicit C620Control(C620CAN& can, TIM_HandleTypeDef& htim);
    void init();
    void setTargetSpeed(uint8_t motor_id, float target_rpm);
    void onTimerTick();
    void updateMotorControl();
    void updateCurrentAngleDeg();
    bool getCurrentAngleDeg(uint8_t motor_id, float& current_angle_deg);

private:
    static constexpr uint8_t MOTOR_COUNT = 6;
    static constexpr uint16_t MAX_ANGLE_RAW = 8191;
    static constexpr uint16_t HOLD_DEADBAND_RAW = 5;
    static constexpr float MOTOR_GEAR_RATIO = 3591.0f / 187.0f;
    static constexpr float KP_SPEED = 0.02f;
    static constexpr float KP_HOLD = 0.8f;

    C620CAN& can_;
    TIM_HandleTypeDef& htim_;
    float target_speed_rpm_[MOTOR_COUNT]{};
    uint16_t target_angle_raw_[MOTOR_COUNT]{};
    bool hold_enabled[MOTOR_COUNT]{};
    uint16_t prev_angle_raw_[MOTOR_COUNT]{};
    bool angle_initialized_[MOTOR_COUNT]{};
    float current_angle_deg_[MOTOR_COUNT]{};
    volatile bool tick_pending_{false};

    void updateSpeedControl();
    void updateHoldControl();
};
