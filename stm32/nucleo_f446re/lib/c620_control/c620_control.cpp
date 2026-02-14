#include "c620_control.hpp"

C620Control::C620Control(C620CAN& c620_can)
    : can(c620_can)
{
    for (uint8_t i = 0; i < 8; ++i)
    {
        target_speed_rpm[i] = 0.0f;
        speed_i[i] = 0.0f;
        prev_angle_deg[i] = 0.0f;
        multi_angle_deg[i] = 0.0f;
        angle_inited[i] = false;
        target_pos_deg[i] = 0.0f;
        position_mode[i] = false;
        hold_pos_deg[i] = 0.0f;
        hold_mode[i] = false;
    }
}

void C620Control::setSpeedTarget(uint8_t motor_id, float target_rpm)
{
    target_speed_rpm[motor_id - 1] = target_rpm * MOTOR_GEAR_RATIO;
    position_mode[motor_id - 1] = false;
    hold_mode[motor_id - 1] = false;
}

void C620Control::setPositionTarget(uint8_t motor_id, float target_deg)
{
    if (motor_id == 0 || motor_id > 8) return;
    target_pos_deg[motor_id - 1] = target_deg;
    position_mode[motor_id - 1] = true;
    hold_mode[motor_id - 1] = false;
}

void C620Control::update()
{
    for (uint8_t id = 1; id <= 8; ++id)
    {
        const uint8_t idx = id - 1;
        const float angle = can.getAngle(id);
        if (!angle_inited[idx])
        {
            prev_angle_deg[idx] = angle;
            multi_angle_deg[idx] = angle;
            angle_inited[idx] = true;
        }
        else
        {
            float d = angle - prev_angle_deg[idx];
            if (d > 180.0f)  d -= 360.0f;
            if (d < -180.0f) d += 360.0f;
            multi_angle_deg[idx] += d;
            prev_angle_deg[idx] = angle;
        }

        const float actual_rpm = can.getSpeed(id);
        float target_rpm = target_speed_rpm[idx];

        if (position_mode[idx])
        {
            const float current_out_deg = multi_angle_deg[idx] / MOTOR_GEAR_RATIO;
            const float error_deg = target_pos_deg[idx] - current_out_deg;

            if (error_deg > -POS_TOL_DEG && error_deg < POS_TOL_DEG)
            {
                // 目標到達後は保持へ。切り替え時の段差を減らすためKP_HOLDで継続制御
                position_mode[idx] = false;
                hold_pos_deg[idx] = target_pos_deg[idx];
                hold_mode[idx] = true;

                float target_out_rpm = KP_HOLD * error_deg;
                if (target_out_rpm > MAX_TARGET_RPM) target_out_rpm = MAX_TARGET_RPM;
                if (target_out_rpm < -MAX_TARGET_RPM) target_out_rpm = -MAX_TARGET_RPM;
                target_rpm = target_out_rpm * MOTOR_GEAR_RATIO;
                target_speed_rpm[idx] = target_rpm;
            }
            else
            {
                // 位置P → 速度指令（出力軸rpm）
                float target_out_rpm = KP_POS * error_deg;
                if (target_out_rpm > MAX_TARGET_RPM) target_out_rpm = MAX_TARGET_RPM;
                if (target_out_rpm < -MAX_TARGET_RPM) target_out_rpm = -MAX_TARGET_RPM;

                target_rpm = target_out_rpm * MOTOR_GEAR_RATIO;
                target_speed_rpm[idx] = target_rpm;
            }
        }
        else if (hold_mode[idx])
        {
            const float current_out_deg = multi_angle_deg[idx] / MOTOR_GEAR_RATIO;
            const float error_deg = hold_pos_deg[idx] - current_out_deg;
            float target_out_rpm = KP_HOLD * error_deg;
            if (target_out_rpm > MAX_TARGET_RPM) target_out_rpm = MAX_TARGET_RPM;
            if (target_out_rpm < -MAX_TARGET_RPM) target_out_rpm = -MAX_TARGET_RPM;
            target_rpm = target_out_rpm * MOTOR_GEAR_RATIO;
            target_speed_rpm[idx] = target_rpm;
        }
        else if (target_speed_rpm[idx] != 0.0f)
        {
            // 速度指令があるときは速度制御を優先
            target_rpm = target_speed_rpm[idx];
        }
        else
        {
            // 速度指令が無いときは現在位置を保持
            const float current_out_deg = multi_angle_deg[idx] / MOTOR_GEAR_RATIO;
            hold_pos_deg[idx] = current_out_deg;
            hold_mode[idx] = true;
            target_rpm = 0.0f;
            target_speed_rpm[idx] = 0.0f;
        }
        const float error = target_rpm - actual_rpm;

        const float current_cmd = KP_SPEED * error;
        can.setCurrent(id, current_cmd);
    }

    can.sendCurrents();
}

void C620Control::rotate(uint8_t motor_id, float delta_deg)
{
    if (motor_id == 0 || motor_id > 8) return;
    const uint8_t idx = motor_id - 1;
    if (!angle_inited[idx])
    {
        // 次のupdateで初期化されるのでここでは何もしない
        return;
    }

    const float current_out_deg = multi_angle_deg[idx] / MOTOR_GEAR_RATIO;
    setPositionTarget(motor_id, current_out_deg + delta_deg);
}

void C620Control::holdPosition(uint8_t motor_id)
{
    if (motor_id == 0 || motor_id > 8) return;
    const uint8_t idx = motor_id - 1;
    if (!angle_inited[idx])
    {
        return;
    }
    const float current_out_deg = multi_angle_deg[idx] / MOTOR_GEAR_RATIO;
    hold_pos_deg[idx] = current_out_deg;
    hold_mode[idx] = true;
    position_mode[idx] = false;
    target_speed_rpm[idx] = 0.0f;
}
