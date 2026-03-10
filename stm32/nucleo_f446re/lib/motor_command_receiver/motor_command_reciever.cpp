#include "motor_command_receiver.hpp"
#include "usart.h"
#include <cstddef>
#include <cstdlib>
#include <cstdio>
#include <cstring>

namespace
{
bool parseCsvFloats(const char* line, char head, float* out_values, size_t value_count)
{
    if(line == nullptr || out_values == nullptr || value_count == 0)
    {
        return false;
    }

    if(line[0] != head || line[1] != ',')
    {
        return false;
    }

    const char* cursor = line + 2;

    for(size_t i = 0; i < value_count; i++)
    {
        char* end_ptr = nullptr;
        const float value = std::strtof(cursor, &end_ptr);

        if(end_ptr == cursor)
        {
            return false;
        }

        out_values[i] = value;

        if(i + 1 < value_count)
        {
            if(*end_ptr != ',')
            {
                return false;
            }
            cursor = end_ptr + 1;
        }
        else
        {
            if(*end_ptr != '\0')
            {
                return false;
            }
        }
    }

    return true;
}
}  // namespace

void MotorCommandReceiver::reset()
{
    line_length_ = 0;
    overflow_ = false;
    has_base_ = false;
    has_arm_ = false;
    latest_line_[0] = '\0';
    latest_line_length_ = 0;
    has_line_ = false;
    latest_parse_error_[0] = '\0';
    has_parse_error_ = false;
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

bool MotorCommandReceiver::popParseError(char* out, size_t out_size)
{
    if(!has_parse_error_ || out == nullptr || out_size == 0)
    {
        return false;
    }

    size_t len = std::strlen(latest_parse_error_);
    if(len >= out_size)
    {
        len = out_size - 1;
    }
    std::memcpy(out, latest_parse_error_, len);
    out[len] = '\0';
    has_parse_error_ = false;
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
        float values[2]{};

        if(parseCsvFloats(line, 'B', values, 2))
        {
            parsed.motor_rpm = values[0];
            parsed.target_steer_deg = values[1];
            latest_base_ = parsed;
            has_base_ = true;
        }
        else
        {
            std::snprintf(
                latest_parse_error_,
                sizeof(latest_parse_error_),
                "parse fail B line='%s'",
                line);
            has_parse_error_ = true;
        }
        return;
    }

    if(line[0] == 'A' && line[1] == ',')
    {
        ArmMotorCommand parsed{};
        float values[7]{};

        if(parseCsvFloats(line, 'A', values, 7))
        {
            parsed.joint_1_rpm = values[0];
            parsed.joint_2_rpm = values[1];
            parsed.joint_3_rpm = values[2];
            parsed.joint_4_rpm = values[3];
            parsed.joint_5_rpm = values[4];
            parsed.joint_6_rpm = values[5];
            parsed.gripper_rpm = values[6];
            latest_arm_ = parsed;
            has_arm_ = true;
        }
        else
        {
            std::snprintf(
                latest_parse_error_,
                sizeof(latest_parse_error_),
                "parse fail A line='%s'",
                line);
            has_parse_error_ = true;
        }
        return;
    }

    std::snprintf(
        latest_parse_error_,
        sizeof(latest_parse_error_),
        "unknown head line='%s'",
        line);
    has_parse_error_ = true;
}
