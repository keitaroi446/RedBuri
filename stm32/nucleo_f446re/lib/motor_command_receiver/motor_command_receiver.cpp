#include <cmath>
#include "motor_command_receiver.hpp"

MotorCommandReceiver::MotorCommandReceiver(UART_HandleTypeDef& huart)
    : huart_(huart)
{}

void MotorCommandReceiver::init()
{
    len_ = 0;
    HAL_UART_Receive_IT(&huart_, &rx_byte_, 1);
}

void MotorCommandReceiver::callback()
{
    const char c = static_cast<char>(rx_byte_);

    if(c == '\r')
    {
        HAL_UART_Receive_IT(&huart_, &rx_byte_, 1);
        return;
    }

    if(c == '\n')
    {
        buf_[len_] = '\0';

        if(len_ > 0)
        {
            if(buf_[0] == 'B')
            {
                parseBaseCommand();
            }
            else if(buf_[0] == 'A')
            {
                parseArmCommand();
            }
        }

        len_ = 0;
        HAL_UART_Receive_IT(&huart_, &rx_byte_, 1);
        return;
    }

    if(len_ < BUF_SIZE - 1)
    {
        buf_[len_++] = c;
    }
    else
    {
        len_ = 0;
    }

    HAL_UART_Receive_IT(&huart_, &rx_byte_, 1);
}

void MotorCommandReceiver::setCurrentSteerDeg(float steer_deg)
{
    current_steer_deg_ = steer_deg;
    computeBaseMotorRpm();
}

float MotorCommandReceiver::getFrontRpm() const
{
    return front_rpm_;
}

float MotorCommandReceiver::getRearRightRpm() const
{
    return rear_right_rpm_;
}

float MotorCommandReceiver::getRearLeftRpm() const
{
    return rear_left_rpm_;
}

int16_t MotorCommandReceiver::getTargetSteerDeg() const
{
    return target_steer_deg_;
}

int16_t MotorCommandReceiver::getArmMotorRpm(uint8_t joint_num) const
{
    return arm_motor_rpm_[joint_num - 1];
}

bool MotorCommandReceiver::parseBaseCommand()
{
    const char* p = buf_;

    if(*p++ != 'B') return false;
    if(*p++ != ',') return false;
    if(!parseInt(p, base_motor_rpm_)) return false;
    if(*p++ != ',') return false;
    if(!parseInt(p, target_steer_deg_)) return false;
    if(*p != '\0') return false;

    computeBaseMotorRpm();
    return true;
}

bool MotorCommandReceiver::parseArmCommand()
{
    const char* p = buf_;

    if(*p++ != 'A') return false;
    if(*p++ != ',') return false;

    for(int i = 0; i < 7; ++i)
    {
        if(!parseInt(p, arm_motor_rpm_[i])) return false;
        if(i < 6 && *p++ != ',') return false;
    }

    if(*p != '\0') return false;
    return true;
}

bool MotorCommandReceiver::parseInt(const char*& p, int16_t& value)
{
    bool negative = false;
    int32_t result = 0;

    if(*p == '-')
    {
        negative = true;
        p++;
    }

    if(*p < '0' || *p > '9')
    {
        return false;
    }

    while(*p >= '0' && *p <= '9')
    {
        result = result * 10 + (*p - '0');
        p++;
    }

    if(negative)
    {
        result = -result;
    }

    value = static_cast<int16_t>(result);
    return true;
}

void MotorCommandReceiver::computeBaseMotorRpm()
{
    if(target_steer_deg_ == 90.0f)
    {
        if(std::fabs(current_steer_deg_ - 90.0f) < 2.0f)
        {
            front_rpm_ = base_motor_rpm_;
            rear_right_rpm_ = base_motor_rpm_ * (TREAD_M / 2.0f) / WHEELBASE_M;
            rear_left_rpm_ = -rear_right_rpm_;
            return;
        }

        front_rpm_ = 0.0f;
        rear_right_rpm_ = 0.0f;
        rear_left_rpm_ = 0.0f;
        return;
    }

    if(std::fabs(current_steer_deg_) > 30.0f)
    {
        front_rpm_ = 0.0f;
        rear_right_rpm_ = 0.0f;
        rear_left_rpm_ = 0.0f;
        return;
    }

    if(std::fabs(current_steer_deg_) < 1.0f)
    {
        front_rpm_ = base_motor_rpm_;
        rear_right_rpm_ = front_rpm_;
        rear_left_rpm_ = front_rpm_;
        return;
    }

    const float steer_rad = current_steer_deg_ * 3.1415926535f / 180.0f;
    const float tan_steer = std::tan(steer_rad);
    const float R = std::fabs(WHEELBASE_M / tan_steer);
    const float half_tread = TREAD_M / 2.0f;
    const float outer = base_motor_rpm_ * ((R + half_tread) / R);
    const float inner = base_motor_rpm_ * ((R - half_tread) / R);

    if(current_steer_deg_ > 0.0f)
    {
        front_rpm_ = base_motor_rpm_;
        rear_left_rpm_ = inner;
        rear_right_rpm_ = outer;
    }
    else
    {
        front_rpm_ = base_motor_rpm_;
        rear_left_rpm_ = outer;
        rear_right_rpm_ = inner;
    }
}
