#pragma once

#include <cstddef>
#include <cstdint>

struct BaseMotorCommand
{
    float motor_rpm = 0.0f;
    float target_steer_deg = 0.0f;
};

struct ArmMotorCommand
{
    float joint_1_rpm = 0.0f;
    float joint_2_rpm = 0.0f;
    float joint_3_rpm = 0.0f;
    float joint_4_rpm = 0.0f;
    float joint_5_rpm = 0.0f;
    float joint_6_rpm = 0.0f;
    float gripper_rpm = 0.0f;
};

class MotorCommandReceiver
{
public:
    static constexpr uint16_t MAX_LINE_LENGTH = 127;

    void reset();
    void pushByte(uint8_t byte);
    void startUart2Rx();
    void onUart2RxComplete();

    bool hasBaseCommand() const;
    bool hasArmCommand() const;
    bool popBaseCommand(BaseMotorCommand& out);
    bool popArmCommand(ArmMotorCommand& out);
    bool popReceivedLine(char* out, size_t out_size);
    bool popParseError(char* out, size_t out_size);

private:
    void parseLine(const char* line);

    char line_buffer_[MAX_LINE_LENGTH + 1]{};
    uint16_t line_length_ = 0;
    bool overflow_ = false;

    BaseMotorCommand latest_base_{};
    ArmMotorCommand latest_arm_{};
    bool has_base_ = false;
    bool has_arm_ = false;
    char latest_line_[MAX_LINE_LENGTH + 1]{};
    uint16_t latest_line_length_ = 0;
    bool has_line_ = false;
    char latest_parse_error_[MAX_LINE_LENGTH + 1]{};
    bool has_parse_error_ = false;
    uint8_t rx_byte_ = 0;
};
