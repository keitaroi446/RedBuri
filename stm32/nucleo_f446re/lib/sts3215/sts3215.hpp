#pragma once
#include "main.h"
#include <stdint.h>
#include "usart.h"
#include "gpio.h"

class STS3215{
public:
    STS3215(UART_HandleTypeDef* huart, uint8_t id); //uartの指定とID（出荷時は1）
    
    void setMode(uint8_t mode);
    void setPosition(uint16_t position);
    int16_t getPosition();

    int16_t AdjustPosition();
    void moveRelative(int16_t targetAngle);

private:
    UART_HandleTypeDef* _huart;
    uint8_t _id;
    int16_t _startPos;

    uint8_t calcChecksum(uint8_t *msg, uint8_t len);
    int16_t deg_to_ticks(float deg);
};