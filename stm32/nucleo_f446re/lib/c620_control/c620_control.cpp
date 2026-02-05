#include "c620_control.hpp"

C620Control::C620Control(C620CAN& c620_can)
    : can(c620_can)
{
    for (uint8_t i = 0; i < 8; ++i)
    {
        target_speed_rpm[i] = 0.0f;
        speed_i[i] = 0.0f;
    }
}

void C620Control::setSpeedTarget(uint8_t motor_id, float target_rpm)
{
    target_speed_rpm[motor_id - 1] = target_rpm;
}

void C620Control::update()
{
    for (uint8_t id = 1; id <= 8; ++id)
    {
        const uint8_t idx = id - 1;
        const float actual_rpm = can.getSpeed(id);
        const float target_rpm = target_speed_rpm[idx];
        const float error = target_rpm - actual_rpm;

        speed_i[idx] += error * DT_SEC;

        const float current_cmd = KP_SPEED * error + KI_SPEED * speed_i[idx];
        can.setCurrent(id, current_cmd);
    }

    can.sendCurrents();
}
