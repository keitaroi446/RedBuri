#pragma once
#include "main.h"
#include <stdint.h>
#include "usart.h"
#include "gpio.h"

class STS3215{
public:
    STS3215(UART_HandleTypeDef* huart, uint8_t id,
            GPIO_TypeDef* ledPort = nullptr, uint16_t ledPin = 0);

    // ランニングモード設定（Reg 33）
    HAL_StatusTypeDef setMode(uint8_t mode);

    // 位置指令（Reg 42）: 0-4095、time/speed は必要なら
    HAL_StatusTypeDef setPosition(uint16_t position,
                                  uint16_t time_ms = 0,
                                  uint16_t speed   = 0);

    // 現在位置（Reg 56）: 正常 0-4095 / 失敗 -1
    int16_t getPosition(uint32_t timeout_ms = 40);
    // 現在角度[deg]（0.0～360.0未満）/ 失敗時 -1.0f
    float getAngleDeg(uint32_t timeout_ms = 40);

    // 現在位置が読めるまでリトライして基準化（失敗時 2048）
    int16_t syncCenter(uint32_t retries = 1000, uint32_t interval_ms = 50);

    // 相対移動（基準+相対ティック）
    void moveRelativeTicks(int16_t offset_ticks);

    // 相対移動（角度deg指定）
    void moveRelativeDeg(float offset_deg);

    // 絶対角度移動（0～360deg、範囲外はwrap）
    HAL_StatusTypeDef setAngleDeg(float angle_deg,
                                  uint16_t time_ms = 0,
                                  uint16_t speed   = 0);
    // 現在位置基準の相対移動
    HAL_StatusTypeDef moveByTicks(int16_t delta_ticks,
                                  uint32_t timeout_ms = 40,
                                  uint16_t time_ms = 0,
                                  uint16_t speed = 0);
    HAL_StatusTypeDef moveByDeg(float delta_deg,
                                uint32_t timeout_ms = 40,
                                uint16_t time_ms = 0,
                                uint16_t speed = 0);
    // 現在位置をゼロ基準として記録
    HAL_StatusTypeDef captureZero(uint32_t timeout_ms = 40);
    // ゼロ基準からの現在角度[deg]（0.0～360.0未満）/ 失敗時 -1.0f
    float getAngleFromZeroDeg(uint32_t timeout_ms = 40);
    // ゼロ基準からの絶対角度移動（0～360deg、範囲外はwrap）
    HAL_StatusTypeDef setAngleFromZeroDeg(float angle_deg,
                                          uint16_t time_ms = 0,
                                          uint16_t speed = 0);

    // 角度→ティック（4096/360）
    static int16_t degToTicks(float deg);
    static uint16_t degToPos(float deg);   // 0～360deg -> 0～4095
    static float ticksToDeg(uint16_t ticks); // 0～4095 -> 0～360deg

private:
    UART_HandleTypeDef* huart_;
    uint8_t             id_;
    int16_t             startPos_;      // -1 = 未確定
    uint16_t            zeroPos_;       // captureZeroで記録する基準tick
    bool                zeroCaptured_;
    GPIO_TypeDef*       ledPort_;       // 任意
    uint16_t            ledPin_;        // 任意

    // 低レベル補助
    static uint8_t calcChecksum(const uint8_t* msg, size_t len);
    inline void toTx() { HAL_HalfDuplex_EnableTransmitter(huart_); }
    inline void toRx() { HAL_HalfDuplex_EnableReceiver(huart_);   }
    inline void ledOn()  { if (ledPort_) HAL_GPIO_WritePin(ledPort_, ledPin_, GPIO_PIN_SET); }
    inline void ledOff() { if (ledPort_) HAL_GPIO_WritePin(ledPort_, ledPin_, GPIO_PIN_RESET); }
};
