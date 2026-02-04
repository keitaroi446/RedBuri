#pragma once
#include <cstdint>

constexpr float LIMIT_CURRENT_AMP = 6.0f;       // ユーザー定義の電流制限[A]
constexpr float MAX_CURRENT_AMP = 20.0f;        // 最大電流[A]
constexpr int16_t MAX_CURRENT_RAW = 16384;      // 最大電流(16bit)
constexpr uint16_t MAX_ANGLE_RAW = 8191;        // 最大角度(16bit)

class C620CAN
{
public:
    void init();
    void setCurrent(uint8_t motor_id, float current_amp);
    void sendCurrents();
    void readMotorStatus();
    float getAngle(uint8_t motor_id);
    float getSpeed(uint8_t motor_id);
    float getCurrent(uint8_t motor_id);
    float getTemp(uint8_t motor_id);
    uint32_t getRxCount() const;

private:
    int16_t target_currents_raw[8] = {};
    uint16_t angles_raw[8] = {};
    int16_t speeds_rpm[8] = {};
    int16_t currents_raw[8] = {};
    uint8_t temps_degc[8] = {};    
};

/*
コールバック関数をmain.cppに書く

C620CAN c620;

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    c620.readMotorStatus();
}
*/
