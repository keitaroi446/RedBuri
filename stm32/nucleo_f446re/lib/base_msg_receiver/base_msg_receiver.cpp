#include <cstdio>

#include "base_msg_receiver.hpp"

bool BaseMsgReceiver::readMsg(const char *line)
{
    char msg_type{};
    float steer{};
    float motor_f{};
    float motor_r{};
    float motor_l{};
}

float BaseMsgReceiver::getSteerDeg() const
{
    return steer_deg;
}

float BaseMsgReceiver::getMotorFRpm() const
{
    return motor_f_rpm;
}

float BaseMsgReceiver::getMotorRRpm() const
{
    return motor_r_rpm;
}

float BaseMsgReceiver::getMotorLRpm() const
{
    return motor_l_rpm;
}
