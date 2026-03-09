#include "motor_command_receiver.hpp"
#include "usart.h"
#include <cstddef>
#include <cstdio>
#include <cstring>

void MotorCommandReceiver::reset()
{
    line_length_ = 0;
    overflow_ = false;
    has_base_ = false;
    has_arm_ = false;
    latest_line_[0] = '\0';
    latest_line_length_ = 0;
    has_line_ = false;
}

void MotorCommandReceiver::pushByte(uint8_t byte)
{
    if(byte == '\r')
    {
        return;
    }

    if(byte == '\n')
    {
        if(line_length_ > 0 && !overflow_)
        {
            line_buffer_[line_length_] = '\0';
            std::strncpy(latest_line_, line_buffer_, MAX_LINE_LENGTH);
            latest_line_[MAX_LINE_LENGTH] = '\0';
            latest_line_length_ = line_length_;
            has_line_ = true;
            parseLine(line_buffer_);
        }
        line_length_ = 0;
        overflow_ = false;
        return;
    }

    if(line_length_ < MAX_LINE_LENGTH)
    {
        line_buffer_[line_length_] = static_cast<char>(byte);
        line_length_++;
    }
    else
    {
        overflow_ = true;
    }
}

void MotorCommandReceiver::startUart2Rx()
{
    HAL_UART_Receive_IT(&huart2, &rx_byte_, 1);
}

void MotorCommandReceiver::onUart2RxComplete()
{
    pushByte(rx_byte_);
    HAL_UART_Receive_IT(&huart2, &rx_byte_, 1);
}

bool MotorCommandReceiver::hasBaseCommand() const
{
    return has_base_;
}

bool MotorCommandReceiver::hasArmCommand() const
{
    return has_arm_;
}

bool MotorCommandReceiver::popBaseCommand(BaseMotorCommand& out)
{
    if(!has_base_)
    {
        return false;
    }

    out = latest_base_;
    has_base_ = false;
    return true;
}

bool MotorCommandReceiver::popArmCommand(ArmMotorCommand& out)
{
    if(!has_arm_)
    {
        return false;
    }

    out = latest_arm_;
    has_arm_ = false;
    return true;
}

bool MotorCommandReceiver::popReceivedLine(char* out, size_t out_size)
{
    if(!has_line_ || out == nullptr || out_size == 0)
    {
        return false;
    }

    size_t copy_size = latest_line_length_;
    if(copy_size >= out_size)
    {
        copy_size = out_size - 1;
    }
    std::memcpy(out, latest_line_, copy_size);
    out[copy_size] = '\0';
    has_line_ = false;
    return true;
}

void MotorCommandReceiver::parseLine(const char* line)
{
    if(line == nullptr || line[0] == '\0')
    {
        return;
    }

    if(line[0] == 'B' && line[1] == ',')
    {
        BaseMotorCommand parsed{};
        int count = std::sscanf(
            line,
            "B,%f,%f,%f,%f",
            &parsed.motor_f_rpm,
            &parsed.motor_r_rpm,
            &parsed.motor_l_rpm,
            &parsed.steer_deg);

        if(count == 4)
        {
            latest_base_ = parsed;
            has_base_ = true;
        }
        return;
    }

    if(line[0] == 'A' && line[1] == ',')
    {
        ArmMotorCommand parsed{};
        int count = std::sscanf(
            line,
            "A,%f,%f,%f,%f,%f,%f,%f",
            &parsed.joint_1_rpm,
            &parsed.joint_2_rpm,
            &parsed.joint_3_rpm,
            &parsed.joint_4_rpm,
            &parsed.joint_5_rpm,
            &parsed.joint_6_rpm,
            &parsed.gripper_rpm);

        if(count == 7)
        {
            latest_arm_ = parsed;
            has_arm_ = true;
        }
    }
}
