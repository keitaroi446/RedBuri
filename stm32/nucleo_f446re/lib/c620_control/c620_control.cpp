#include "tim.h"
#include "c620_control.hpp"

C620Control::C620Control(C620CAN& c620_can, TIM_HandleTypeDef& htim)
    : can_(c620_can), htim_(htim)
{}

void C620Control::init()
{
    HAL_TIM_Base_Start_IT(&htim_);

    for(uint8_t motor_id = 1; motor_id <= MOTOR_COUNT; motor_id++)
    {
        const uint8_t idx = motor_id - 1;
        can_.getAngleRaw(motor_id, target_angle_raw_[idx]);
        hold_enabled[idx] = true;
    }
}

void C620Control::setTargetSpeed(uint8_t motor_id, float target_rpm)
{
    if(motor_id < 1 || motor_id > MOTOR_COUNT) return;

    const uint8_t idx = motor_id - 1;
    target_speed_rpm_[idx] = target_rpm * MOTOR_GEAR_RATIO;
    if(target_rpm == 0.0f && !hold_enabled[idx])
    {
        can_.getAngleRaw(motor_id, target_angle_raw_[idx]);
        hold_enabled[idx] = true;
    }

    if(target_rpm != 0.0f && hold_enabled[idx])
    {
        hold_enabled[idx] =false;
    } 
}

void C620Control::onTimerTick()
{
    tick_pending_ = true;
}

void C620Control::updateMotorControl()
{
    if(!tick_pending_)
    {
        return;
    }

    tick_pending_ = false;
    updateHoldControl();
    updateSpeedControl();
    can_.sendCurrents();
}

void C620Control::updateCurrentAngleDeg()
{
    for(uint8_t motor_id = 1; motor_id <= MOTOR_COUNT; motor_id++)
    {
        const uint8_t idx = motor_id - 1;
        uint16_t raw{};

        can_.getAngleRaw(motor_id, raw);

        if(!angle_initialized_[idx])
        {
            prev_angle_raw_[idx] = raw;
            angle_initialized_[idx] = true;
            continue;
        }

        int32_t diff = static_cast<int32_t>(raw) - static_cast<int32_t>(prev_angle_raw_[idx]);
        if(diff > 4096) diff -= 8192;
        if(diff < -4096) diff += 8192;

        current_angle_deg_[idx] += (static_cast<float>(diff) * 360.0f / 8192.0f) / MOTOR_GEAR_RATIO;
        prev_angle_raw_[idx] = raw;
    }
}

bool C620Control::getCurrentAngleDeg(uint8_t motor_id, float& current_angle_deg)
{
    if(motor_id < 1 || motor_id > MOTOR_COUNT) return false;
    current_angle_deg = current_angle_deg_[motor_id - 1];
    return true;
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

void C620Control::updateHoldControl()
{
    for(uint8_t motor_id = 1; motor_id <= MOTOR_COUNT; motor_id++)
    {
        const uint8_t idx = motor_id - 1;

        if(!hold_enabled[idx]) continue;

        uint16_t current_angle_raw{};
        can_.getAngleRaw(motor_id, current_angle_raw);
        int16_t error_raw = target_angle_raw_[idx] - current_angle_raw;

        if(error_raw > MAX_ANGLE_RAW / 2)
        {
            error_raw -= MAX_ANGLE_RAW + 1;
        }
        else if(error_raw < -(MAX_ANGLE_RAW / 2))
        {
            error_raw += MAX_ANGLE_RAW + 1;
        }

        if(error_raw > HOLD_DEADBAND_RAW || error_raw < -HOLD_DEADBAND_RAW)
        {
            target_speed_rpm_[idx] = KP_HOLD * error_raw;
        }
        else
        {
            target_speed_rpm_[idx] = 0.0f;
        }
    }
}
