#pragma once

#include <cstdint>

class BaseMotorReceiver
{
public:
    void init();
    void callback();
    float motor_rpm_{0.0f};
    float target_steer_deg_{0.0f};

private:
    static constexpr int WHEELBASE_M{};
    static constexpr int TREAD_M{};
    static constexpr uint8_t BUF_SIZE{64};

    uint8_t rx_byte_{};
    char buf_[BUF_SIZE]{};
    uint8_t len_{0};
};
