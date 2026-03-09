#pragma once

#include <cstdint>
#include "main.h"

class StepAxis
{
public:
    StepAxis(TIM_HandleTypeDef* timer,
             uint32_t timer_channel,
             GPIO_TypeDef* dir_port,
             uint16_t dir_pin,
             bool dir_positive_high,
             float step_deg,
             uint32_t microstep,
             float gear_ratio);

    TIM_HandleTypeDef* timer() const;
    bool isRunning() const;

    void setStepFrequencyHz(uint32_t step_hz);
    void moveToDeg(float target_deg);
    void onPulseFinished();

private:
    uint32_t angleToSteps(float delta_deg) const;
    void startMoveRelativeDeg(float delta_deg);

    TIM_HandleTypeDef* timer_;
    uint32_t channel_;
    GPIO_TypeDef* dir_port_;
    uint16_t dir_pin_;
    bool dir_positive_high_;
    float step_deg_;
    uint32_t microstep_;
    float gear_ratio_;
    volatile uint32_t remaining_steps_;
    volatile bool running_;
    float current_deg_;
};
