#pragma once

#include <cstdint>
#include "c620_can.hpp"

class C620Control
{
public:
    explicit C620Control(C620CAN& can);
    void setSpeedTarget(uint8_t motor_id, float target_rpm);
    void update();

private:
    static constexpr float KP_SPEED = 0.1f;
    static constexpr float KI_SPEED = 0.1f;
    static constexpr float DT_SEC = 0.001f;
    
    C620CAN& can;
    float target_speed_rpm[8] = {};
    float speed_i[8] = {};
};
