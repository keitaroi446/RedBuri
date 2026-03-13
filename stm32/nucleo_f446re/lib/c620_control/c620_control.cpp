#include "c620_control.hpp"

C620Control::C620Control(C620CAN& c620_can)
    : can_(c620_can)
{}

void C620Control::setTargetSpeed(uint8_t motor_id, float target_rpm)
{
    if(motor_id < 1 || motor_id > MOTOR_COUNT) return;

    const uint8_t idx = motor_id - 1;
    target_speed_rpm_[idx] = target_rpm * MOTOR_GEAR_RATIO;
}

void C620Control::updateSpeedControl()
{
    for(uint8_t motor_id = 1; motor_id <= MOTOR_COUNT; motor_id++)
    {
        const uint8_t idx = motor_id - 1;
        int16_t current_speed_rpm{};
        can_.getSpeedRpm(motor_id, current_speed_rpm);
        const float target_current_amp = KP_SPEED * (target_speed_rpm_[idx] - current_speed_rpm);
        can_.setCurrent(motor_id, target_current_amp);
    }
}