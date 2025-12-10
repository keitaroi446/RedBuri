#ifndef C620_CAN_HPP
#define C620_CAN_HPP

#include "can.h"

class C620CAN
{
public:
    C620CAN();
    void setCurrent(uint8_t motor_id, float current_amp);
    void sendCurrents();
    void readMotorStatus();
    float getAngle(uint8_t motor_id);
    float getSpeed(uint8_t motor_id);
    float getCurrent(uint8_t motor_id);
    float getTemp(uint8_t motor_id);

private:
    int16_t target_currents_raw[8];
    uint16_t angles_raw[8];
    int16_t speeds_rpm[8];
    int16_t currents_raw[8];
    uint8_t temps_degc[8];
    
};

/*
コールバック関数をmain.cppに書く

C620CAN c620;

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    c620.readMotorStatus();
}
*/

#endif // C620_CAN_HPP