#include "motor_cmd_uart.hpp"
#include <string.h>
#include <stdlib.h>

MotorCmdUart::MotorCmdUart(UART_HandleTypeDef* huart) : huart_(huart) {}

void MotorCmdUart::startRx()
{
    if (!huart_) return;
    HAL_UART_Receive_IT(huart_, &rx_byte_, 1);
}

bool MotorCmdUart::popLine(char* out, size_t max_len)
{
    if (!out || max_len == 0) {
        return false;
    }

    __disable_irq();
    if (!line_ready_) {
        __enable_irq();
        return false;
    }
    size_t n = line_len_;
    if (n >= max_len) {
        n = max_len - 1;
    }
    memcpy(out, line_buf_, n);
    out[n] = '\0';
    line_ready_ = false;
    line_len_ = 0;
    __enable_irq();
    return true;
}

bool MotorCmdUart::popParsed(char id[3], float* value)
{
    if (!id || !value) {
        return false;
    }

    char line[kLineBufSize]{};
    if (!popLine(line, sizeof(line))) {
        return false;
    }

    if (strlen(line) < 3) {
        return false;
    }

    id[0] = line[0];
    id[1] = line[1];
    id[2] = '\0';

    char* endptr = nullptr;
    const float v = strtof(&line[2], &endptr);
    if (endptr == &line[2]) {
        return false;
    }
    *value = v;
    return true;
}

void MotorCmdUart::onRxByte(uint8_t byte)
{
    const char c = static_cast<char>(byte);
    if (c == '\n') {
        if (!line_ready_ && line_len_ > 0) {
            if (line_len_ >= kLineBufSize) {
                line_len_ = kLineBufSize - 1;
            }
            line_buf_[line_len_] = '\0';
            line_ready_ = true;
        }
        line_len_ = 0;
    } else if (c != '\r') {
        if (!line_ready_) {
            if (line_len_ < kLineBufSize - 1) {
                line_buf_[line_len_++] = c;
            } else {
                line_len_ = 0;
            }
        }
    }
}

void MotorCmdUart::handleRxComplete(UART_HandleTypeDef* huart)
{
    if (huart != huart_) {
        return;
    }
    onRxByte(rx_byte_);
    HAL_UART_Receive_IT(huart_, &rx_byte_, 1);
}
