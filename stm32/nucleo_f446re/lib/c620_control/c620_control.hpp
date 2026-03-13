#pragma once

#include <cstdint>
#include "c620_can.hpp"

class C620Control
{
public:
    explicit C620Control(C620CAN& can);
    void setTargetSpeed(uint8_t motor_id, float target_rpm);
    void updateSpeedControl();
    void updateHoldControl();

private:
    static constexpr uint8_t MOTOR_COUNT = 6;
    static constexpr uint16_t MAX_ANGLE_RAW = 8191;
    static constexpr float MOTOR_GEAR_RATIO = 3591.0f / 187.0f;
    static constexpr float KP_SPEED = 0.015f;
    static constexpr float KP_HOLD = 0.1f;
    static constexpr float MAX_TARGET_RPM = 1000.0f;

    C620CAN& can_;
    float target_speed_rpm_[MOTOR_COUNT]{};
};