#pragma once

#include <cstdint>

class BaseMsgReceiver
{
public:
    bool readMsg(const char *line);

    float getSteerDeg() const;
    float getMotorFRpm() const;
    float getMotorRRpm() const;
    float getMotorLRpm() const;

private:
    float steer_deg{};
    float motor_f_rpm{};
    float motor_r_rpm{};
    float motor_l_rpm{};
};
