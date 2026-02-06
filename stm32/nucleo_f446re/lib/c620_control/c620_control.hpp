#pragma once

#include <cstdint>
#include "c620_can.hpp"

class C620Control
{
public:
    explicit C620Control(C620CAN& can);
    void setSpeedTarget(uint8_t motor_id, float target_rpm);
    void update();
    void rotate(uint8_t motor_id, float delta_deg);
    void setPositionTarget(uint8_t motor_id, float target_deg);

private:
    static constexpr float GEAR_RATIO = 3591.0f / 187.0f;   // 減速比
    static constexpr float KP_SPEED = 0.03f;
    static constexpr float KP_POS = 2.0f;                  // 位置Pゲイン[ rpm/deg ]
    static constexpr float POS_TOL_DEG = 1.0f;             // 位置到達判定[deg]
    static constexpr float MAX_TARGET_RPM = 100.0f;        // 速度指令上限[rpm]
    static constexpr float DT_SEC = 0.001f;
    
    C620CAN& can;
    float target_speed_rpm[8] = {};
    float speed_i[8] = {};
    float prev_angle_deg[8] = {};
    float multi_angle_deg[8] = {};
    bool angle_inited[8] = {};
    float target_pos_deg[8] = {};
    bool position_mode[8] = {};
};
