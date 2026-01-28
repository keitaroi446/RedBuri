#pragma once

#include "usart.h"
#include <stddef.h>
#include <stdint.h>

class MotorCmdUart
{
public:
    explicit MotorCmdUart(UART_HandleTypeDef* huart);
    void startRx();
    bool popLine(char* out, size_t max_len);
    bool popParsed(char id[3], float* value);
    void handleRxComplete(UART_HandleTypeDef* huart);

private:
    void onRxByte(uint8_t byte);

    UART_HandleTypeDef* huart_{nullptr};
    uint8_t rx_byte_{0};
    static constexpr size_t kLineBufSize = 64;
    char line_buf_[kLineBufSize]{};
    volatile uint16_t line_len_{0};
    volatile bool line_ready_{false};
};
