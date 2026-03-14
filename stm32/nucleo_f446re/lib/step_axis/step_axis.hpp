#pragma once

#include <cstddef>
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
    float getCurrentDeg() const;

    void setStepFrequencyHz(uint32_t step_hz);
    void moveToDeg(float target_deg);
    // Non-blocking start (returns false if already running)
    bool startMoveToDeg(float target_deg);
    // Reset "current angle" (open-loop). Use when motor power is applied or re-homed.
    void resetCurrentDeg(float deg = 0.0f);
    void onPulseFinished();
    static void moveAxisToDeg(StepAxis* const* axes,
                              size_t axis_count,
                              size_t axis_index,
                              float target_deg);
    static bool startAxisToDeg(StepAxis* const* axes,
                               size_t axis_count,
                               size_t axis_index,
                               float target_deg);
    static void onPulseFinishedForTimer(TIM_HandleTypeDef* htim);

private:
    static void registerInstance(StepAxis* axis);
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
    float target_deg_;
};
