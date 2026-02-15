#include "sts3215.hpp"
#include <cmath>


STS3215::STS3215(UART_HandleTypeDef* huart, uint8_t id)
 : _huart(huart), _id(id), _startPos(-1) {}

uint8_t STS3215::calcChecksum(uint8_t *msg, uint8_t len){
    uint8_t checksum = 0;
    for(int i=2; i < len - 1; i++){
        checksum += msg[i];
    };
    return ~checksum; // ビット反転.
}

// デフォルトは0なので通常はいらない.
void STS3215::setMode(uint8_t mode){
    uint8_t msg[8] = {0xFF, 0xFF, _id, 4, 3, 33, mode, 0};
    msg[7] = calcChecksum(msg, 8);

    HAL_HalfDuplex_EnableTransmitter(_huart);
    HAL_UART_Transmit(_huart, msg, 8, 10);
    while(__HAL_UART_GET_FLAG(_huart, UART_FLAG_TC) == RESET);
    HAL_HalfDuplex_EnableReceiver(_huart);
}

void STS3215::setPosition(uint16_t position) {
    uint8_t msg[13] = {0xFF, 0xFF, _id, 9, 3, 42}; // 指定.
    msg[6] = position & 0xFF;
    msg[7] = (position >> 8) & 0xFF;
    msg[8] = 0x00; msg[9] = 0x00; // 時間、デフォルトのままで.
    msg[10] = 0x00; msg[11] = 0x00; // 速度、でふぉるとのままで.
    msg[12] = calcChecksum(msg, 13);

    HAL_HalfDuplex_EnableTransmitter(_huart);
    HAL_UART_Transmit(_huart, msg, 13, 10);
    while(__HAL_UART_GET_FLAG(_huart, UART_FLAG_TC) == RESET);
    HAL_HalfDuplex_EnableReceiver(_huart);
}

int16_t STS3215::getPosition() {
    uint8_t tx_msg[8] = {0xFF, 0xFF, _id, 4, 2, 56, 2, 8}; // チェックサムは8にしておかないとなぜか送れない(一敗).
    uint8_t rx_msg[8] = {0};
    tx_msg[7] = calcChecksum(tx_msg, 8);

    __HAL_UART_CLEAR_OREFLAG(_huart);
    HAL_HalfDuplex_EnableTransmitter(_huart);
    HAL_UART_Transmit(_huart, tx_msg, 8, 10);
    while(__HAL_UART_GET_FLAG(_huart, UART_FLAG_TC) == RESET);

    HAL_HalfDuplex_EnableReceiver(_huart);
    if (HAL_UART_Receive(_huart, rx_msg, 8, 20) == HAL_OK) {
        if (rx_msg[0] == 0xFF && rx_msg[1] == 0xFF) {
            return (int16_t)((rx_msg[6] << 8) | rx_msg[5]);
        }
    }
    return -1; // -1で受信判断(LED).
}

int16_t STS3215::AdjustPosition() {
    HAL_Delay(50); // 猶予.

    // ここの繰り返しは100回50秒だと確実にうまくいく.
    for (int i = 0; i < 100; i++) {
        _startPos = getPosition();
        if (_startPos != -1) break;
        HAL_Delay(50);
    }
    return (_startPos != -1) ? _startPos : 2048; // デバッグ用
}

void STS3215::moveRelative(int16_t targetAngle) {
    if (_startPos == -1) return; // 受信できなかった場合戻す.
    int16_t change = deg_to_ticks(targetAngle);
    int target = _startPos + change;

    // 制限付き注意.
    if (target > 4095) target = 4095;
    if (target < 0) target = 0;
    setPosition((uint16_t)target);

    _startPos = (uint16_t)target;
}

int16_t STS3215::deg_to_ticks(float deg) {
    float ticks = deg * (4096.0f / 360.0f);
    return (int16_t)lroundf(ticks);
}

