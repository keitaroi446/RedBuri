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
    void holdPosition(uint8_t motor_id);

private:
    static constexpr float MOTOR_GEAR_RATIO = 3591.0f / 187.0f;     // モーター減速比
    static constexpr float PLANETARY_GEAR_RATIO = 1.5f;             // 遊星ギヤ減速比
    static constexpr float KP_SPEED = 0.03f;
    static constexpr float KP_POS = 1.0f;                           // 位置Pゲイン[ rpm/deg
    static constexpr float KP_HOLD = 10.0f;
    static constexpr float POS_TOL_DEG = 1.0f;                      // 位置到達判定[deg]
    static constexpr float MAX_TARGET_RPM = 100.0f;                 // 速度指令上限[rpm]
    static constexpr float DT_SEC = 0.001f;
    
    C620CAN& can;
    float target_speed_rpm[8] = {};
    float speed_i[8] = {};
    float prev_angle_deg[8] = {};
    float multi_angle_deg[8] = {};
    bool angle_inited[8] = {};
    float target_pos_deg[8] = {};
    bool position_mode[8] = {};
    float hold_pos_deg[8] = {};
    bool hold_mode[8] = {};
};
