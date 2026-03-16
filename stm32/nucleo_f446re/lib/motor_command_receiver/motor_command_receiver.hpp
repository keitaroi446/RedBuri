#pragma once

#include <cstdint>
#include "usart.h"

class MotorCommandReceiver
{
public:
    explicit MotorCommandReceiver(UART_HandleTypeDef& huart);
    void init();
    void callback();
    void setCurrentSteerDeg(float steer_deg);
    float getFrontRpm() const;
    float getRearRightRpm() const;
    float getRearLeftRpm() const;
    int16_t getTargetSteerDeg() const;
    int16_t getArmMotorRpm(uint8_t joint_num) const;

private:
    static constexpr float WHEELBASE_M = 0.93053f;
    static constexpr float TREAD_M = 0.675026f;
    static constexpr uint8_t BUF_SIZE{64};
    UART_HandleTypeDef& huart_;
    uint8_t rx_byte_{};
    char buf_[BUF_SIZE]{};
    uint8_t len_{};
    int16_t base_motor_rpm_{};
    int16_t target_steer_deg_{};
    float current_steer_deg_{};
    float front_rpm_{};
    float rear_right_rpm_{};
    float rear_left_rpm_{};
    int16_t arm_motor_rpm_[7]{};

    bool parseInt(const char*& p, int16_t& value);
    bool parseBaseCommand();
    bool parseArmCommand();
    void computeBaseMotorRpm();
};
