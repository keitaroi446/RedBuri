#include "sts3215.hpp"
#include <cmath>
#include <cstdio>
#include "usart.h"
#include <cstring>

STS3215::STS3215(UART_HandleTypeDef* huart, uint8_t id,
                 GPIO_TypeDef* ledPort, uint16_t ledPin)
: huart_(huart), id_(id), startPos_(-1), zeroPos_(0), zeroCaptured_(false),
  ledPort_(ledPort), ledPin_(ledPin)
{}

uint8_t STS3215::calcChecksum(const uint8_t* msg, size_t len) {
    // 仕様：msg[2] ～ msg[len-2] の加算を反転 → msg[len-1]
    uint8_t sum = 0;
    for (size_t i = 2; i + 1 < len; ++i) sum += msg[i];
    return static_cast<uint8_t>(~sum);
}

HAL_StatusTypeDef STS3215::setMode(uint8_t mode) {
    uint8_t msg[8] = {0xFF, 0xFF, id_, 4, 3, 33, mode, 0};
    msg[7] = calcChecksum(msg, sizeof(msg));

    toTx();
    auto st = HAL_UART_Transmit(huart_, msg, sizeof(msg), 30);
    if (st != HAL_OK) return st;
    while (__HAL_UART_GET_FLAG(huart_, UART_FLAG_TC) == RESET) { /*wait*/ }
    toRx();
    return HAL_OK;
}

HAL_StatusTypeDef STS3215::setPosition(uint16_t position, uint16_t time_ms, uint16_t speed) {
    uint8_t msg[13] = {0xFF, 0xFF, id_, 9, 3, 42};
    msg[6]  = static_cast<uint8_t>(position & 0xFF);
    msg[7]  = static_cast<uint8_t>((position >> 8) & 0xFF);
    msg[8]  = static_cast<uint8_t>(time_ms & 0xFF);
    msg[9]  = static_cast<uint8_t>((time_ms >> 8) & 0xFF);
    msg[10] = static_cast<uint8_t>(speed & 0xFF);
    msg[11] = static_cast<uint8_t>((speed >> 8) & 0xFF);
    msg[12] = calcChecksum(msg, sizeof(msg));

    toTx();
    auto st = HAL_UART_Transmit(huart_, msg, sizeof(msg), 30);
    if (st != HAL_OK) return st;
    while (__HAL_UART_GET_FLAG(huart_, UART_FLAG_TC) == RESET) { /*wait*/ }
    toRx();
    return HAL_OK;
}

int16_t STS3215::getPosition(uint32_t timeout_ms) {
    // Read 命令: FF FF ID 04 02 38 02 CHK
    uint8_t tx_msg[8] = {0xFF, 0xFF, id_, 4, 2, 56, 2, 0};
    tx_msg[7] = calcChecksum(tx_msg, sizeof(tx_msg));

    // 送信前に受信側の残留を掃除
    __HAL_UART_CLEAR_OREFLAG(huart_);
#if defined(__HAL_UART_CLEAR_IDLEFLAG)
    __HAL_UART_CLEAR_IDLEFLAG(huart_);
#endif
#if defined(__HAL_UART_FLUSH_DRREGISTER)
    __HAL_UART_FLUSH_DRREGISTER(huart_);
#endif

    toTx();
    if (HAL_UART_Transmit(huart_, tx_msg, sizeof(tx_msg), timeout_ms) != HAL_OK) return -1;
    while (__HAL_UART_GET_FLAG(huart_, UART_FLAG_TC) == RESET) { /*wait*/ }

    toRx();

    // Fast path: status packet is typically 8 bytes for 2-byte read.
    uint8_t rx[8] = {0};
    if (HAL_UART_Receive(huart_, rx, sizeof(rx), timeout_ms) == HAL_OK) {
        if (rx[0] == 0xFF && rx[1] == 0xFF && rx[2] == id_ && rx[3] == 4) {
            uint8_t chk = calcChecksum(rx, sizeof(rx));
            if (chk == rx[7]) {
                const int16_t pos = static_cast<int16_t>((rx[6] << 8) | rx[5]);
                startPos_ = pos;
                return pos;
            }
        }
    }

    // --- ヘッダ同期受信 ---
    uint32_t t0 = HAL_GetTick();
    uint8_t b = 0;
    uint8_t ff_count = 0;
    uint8_t len = 0;

    while ((HAL_GetTick() - t0) < timeout_ms) {
        if (HAL_UART_Receive(huart_, &b, 1, 2) != HAL_OK) continue;

        if (ff_count < 2) {
            ff_count = (b == 0xFF) ? static_cast<uint8_t>(ff_count + 1) : 0;
            continue;
        }

        // ID
        if (b != id_) {
            ff_count = 0;
            continue;
        }

        // LEN
        if (HAL_UART_Receive(huart_, &len, 1, 3) != HAL_OK) return -1;
        if (len < 4 || len > 20) return -1; // ERR + 2byte param + CHK
        break;
    }
    if (len == 0) return -1;

    // --- ボディ受信（ERR + PARAM + CHK）---
    uint8_t body[32] = {0};
    if (HAL_UART_Receive(huart_, body, len, timeout_ms) != HAL_OK) return -1;

    // --- チェックサム検証 ---
    uint8_t frame[4 + 32] = {0xFF, 0xFF, id_, len};
    std::memcpy(frame + 4, body, len);
    uint8_t chk = calcChecksum(frame, 4 + len);
    if (chk != body[len - 1]) return -1;

    // 位置抽出（PARAM=2byte想定: Low=body[1], High=body[2]）
    // body[0] は ERR、body[1..] が PARAM...
    if (len < 4) return -1;
    int16_t pos = static_cast<int16_t>((body[2] << 8) | body[1]);
    startPos_ = pos;
    return pos;
}

float STS3215::getAngleDeg(uint32_t timeout_ms) {
    const int16_t pos = getPosition(timeout_ms);
    if (pos < 0) return -1.0f;
    return ticksToDeg(static_cast<uint16_t>(pos));
}

int16_t STS3215::syncCenter(uint32_t retries, uint32_t interval_ms) {
    HAL_Delay(50); // 安定待ち
    int16_t pos = -1;
    for (uint32_t i = 0; i < retries; ++i) {
        pos = getPosition();
        if (pos != -1) {
            startPos_ = pos;
            ledOff(); // 成功＝LED消灯
            return pos;
        }
        HAL_Delay(interval_ms);
    }
    // 失敗時のフォールバック：2048 & LED点灯
    startPos_ = 2048;
    ledOn();
    return startPos_;
}

void STS3215::moveRelativeTicks(int16_t offset_ticks) {
    if (startPos_ == -1) {
        // まだ基準なし → 一度だけ取得を試みる
        if (syncCenter(50, 10) == -1) return;
    }
    int32_t target = static_cast<int32_t>(startPos_) + static_cast<int32_t>(offset_ticks);
    if (target > 4095) target = 4095;
    if (target < 0)    target = 0;
    (void)setPosition(static_cast<uint16_t>(target), /*time*/0, /*speed*/0);
}

void STS3215::moveRelativeDeg(float offset_deg) {
    moveRelativeTicks(degToTicks(offset_deg));
}

HAL_StatusTypeDef STS3215::setAngleDeg(float angle_deg, uint16_t time_ms, uint16_t speed) {
    const uint16_t target = degToPos(angle_deg);
    const HAL_StatusTypeDef st = setPosition(target, time_ms, speed);
    if (st == HAL_OK) {
        startPos_ = static_cast<int16_t>(target);
    }
    return st;
}

HAL_StatusTypeDef STS3215::moveByTicks(int16_t delta_ticks, uint32_t timeout_ms, uint16_t time_ms, uint16_t speed) {
    int16_t current = -1;
    for (int i = 0; i < 3; ++i) {
        current = getPosition(timeout_ms);
        if (current >= 0) break;
        HAL_Delay(2);
    }
    if (current < 0) {
        if (startPos_ >= 0) {
            current = startPos_;
        } else {
            return HAL_ERROR;
        }
    }

    int32_t target = static_cast<int32_t>(current) + static_cast<int32_t>(delta_ticks);
    target %= 4096;
    if (target < 0) target += 4096;
    const HAL_StatusTypeDef st = setPosition(static_cast<uint16_t>(target), time_ms, speed);
    if (st == HAL_OK) {
        startPos_ = static_cast<int16_t>(target);
    }
    return st;
}

HAL_StatusTypeDef STS3215::moveByDeg(float delta_deg, uint32_t timeout_ms, uint16_t time_ms, uint16_t speed) {
    return moveByTicks(degToTicks(delta_deg), timeout_ms, time_ms, speed);
}

HAL_StatusTypeDef STS3215::captureZero(uint32_t timeout_ms) {
    const int16_t current = getPosition(timeout_ms);
    if (current < 0) return HAL_ERROR;
    zeroPos_ = static_cast<uint16_t>(current);
    zeroCaptured_ = true;
    return HAL_OK;
}

float STS3215::getAngleFromZeroDeg(uint32_t timeout_ms) {
    const int16_t current = getPosition(timeout_ms);
    if (current < 0) return -1.0f;
    if (!zeroCaptured_) {
        zeroPos_ = static_cast<uint16_t>(current);
        zeroCaptured_ = true;
    }

    int32_t rel = static_cast<int32_t>(current) - static_cast<int32_t>(zeroPos_);
    rel %= 4096;
    if (rel < 0) rel += 4096;
    return ticksToDeg(static_cast<uint16_t>(rel));
}

HAL_StatusTypeDef STS3215::setAngleFromZeroDeg(float angle_deg, uint16_t time_ms, uint16_t speed) {
    if (!zeroCaptured_) {
        if (captureZero(40) != HAL_OK) return HAL_ERROR;
    }

    const uint16_t rel = degToPos(angle_deg);
    uint16_t target = static_cast<uint16_t>((static_cast<uint32_t>(zeroPos_) + rel) % 4096U);
    const HAL_StatusTypeDef st = setPosition(target, time_ms, speed);
    if (st == HAL_OK) {
        startPos_ = static_cast<int16_t>(target);
    }
    return st;
}

int16_t STS3215::degToTicks(float deg) {
    // 4096/360 ≒ 11.377... 1tick ≈ 0.08789°
    const double ticks = static_cast<double>(deg) * (4096.0 / 360.0);
    long it = std::lround(ticks);
    if (it < -4096) it = -4096;
    if (it >  4096) it =  4096;
    return static_cast<int16_t>(it);
}

uint16_t STS3215::degToPos(float deg) {
    float wrapped = std::fmod(deg, 360.0f);
    if (wrapped < 0.0f) wrapped += 360.0f;
    const double ticks = static_cast<double>(wrapped) * (4096.0 / 360.0);
    long pos = std::lround(ticks);
    pos %= 4096;
    if (pos < 0) pos += 4096;
    return static_cast<uint16_t>(pos);
}

float STS3215::ticksToDeg(uint16_t ticks) {
    const uint16_t t = static_cast<uint16_t>(ticks % 4096U);
    return (static_cast<float>(t) * 360.0f) / 4096.0f;
}
