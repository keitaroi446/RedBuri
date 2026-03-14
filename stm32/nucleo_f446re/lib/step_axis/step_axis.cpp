#include "step_axis.hpp"
#include <cmath>

namespace {
constexpr size_t kMaxRegisteredAxes = 8U;
StepAxis* s_registeredAxes[kMaxRegisteredAxes] = {nullptr};
size_t s_registeredAxisCount = 0U;
}

StepAxis::StepAxis(TIM_HandleTypeDef* timer,
                   uint32_t timer_channel,
                   GPIO_TypeDef* dir_port,
                   uint16_t dir_pin,
                   bool dir_positive_high,
                   float step_deg,
                   uint32_t microstep,
                   float gear_ratio)
    : timer_(timer),
      channel_(timer_channel),
      dir_port_(dir_port),
      dir_pin_(dir_pin),
      dir_positive_high_(dir_positive_high),
      step_deg_(step_deg),
      microstep_(microstep),
      gear_ratio_(gear_ratio),
      remaining_steps_(0U),
      running_(false),
      current_deg_(0.0f),
      target_deg_(0.0f)
{
    registerInstance(this);
}

TIM_HandleTypeDef* StepAxis::timer() const
{
    return timer_;
}

bool StepAxis::isRunning() const
{
    return running_;
}

float StepAxis::getCurrentDeg() const
{
    return current_deg_;
}

void StepAxis::setStepFrequencyHz(uint32_t step_hz)
{
    if (step_hz < 10U) step_hz = 10U;
    if (step_hz > 20000U) step_hz = 20000U;

    uint32_t arr = (1000000U / step_hz);
    if (arr < 2U) arr = 2U;
    arr -= 1U;

    __HAL_TIM_SET_AUTORELOAD(timer_, arr);
    __HAL_TIM_SET_COMPARE(timer_, channel_, (arr + 1U) / 2U);
    __HAL_TIM_SET_COUNTER(timer_, 0U);
}

void StepAxis::moveToDeg(float target_deg)
{
    if (!startMoveToDeg(target_deg)) return;
    while (running_) {
    }
}

bool StepAxis::startMoveToDeg(float target_deg)
{
    if (running_) return false;
    const float delta = target_deg - current_deg_;
    const uint32_t steps = angleToSteps(delta);
    if (steps == 0U) {
        current_deg_ = target_deg;
        target_deg_ = target_deg;
        return true;
    }

    const bool positive = (delta >= 0.0f);
    const GPIO_PinState dir_state =
        ((positive && dir_positive_high_) || (!positive && !dir_positive_high_))
            ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(dir_port_, dir_pin_, dir_state);

    remaining_steps_ = steps;
    running_ = true;
    target_deg_ = target_deg;
    if (HAL_TIM_PWM_Start_IT(timer_, channel_) != HAL_OK) {
        running_ = false;
        remaining_steps_ = 0U;
        return false;
    }
    return true;
}

void StepAxis::resetCurrentDeg(float deg)
{
    // If a move is in progress, stop it and reset state.
    if (running_) {
        HAL_TIM_PWM_Stop_IT(timer_, channel_);
        running_ = false;
        remaining_steps_ = 0U;
    }
    current_deg_ = deg;
    target_deg_ = deg;
}

void StepAxis::onPulseFinished()
{
    if (!running_) return;
    if (remaining_steps_ > 0U) {
        --remaining_steps_;
    }
    if (remaining_steps_ == 0U) {
        HAL_TIM_PWM_Stop_IT(timer_, channel_);
        running_ = false;
        current_deg_ = target_deg_;
    }
}

void StepAxis::moveAxisToDeg(StepAxis* const* axes,
                             size_t axis_count,
                             size_t axis_index,
                             float target_deg)
{
    if (axes == nullptr) return;
    if (axis_index >= axis_count) return;
    if (axes[axis_index] == nullptr) return;
    axes[axis_index]->moveToDeg(target_deg);
}

bool StepAxis::startAxisToDeg(StepAxis* const* axes,
                              size_t axis_count,
                              size_t axis_index,
                              float target_deg)
{
    if (axes == nullptr) return false;
    if (axis_index >= axis_count) return false;
    if (axes[axis_index] == nullptr) return false;
    return axes[axis_index]->startMoveToDeg(target_deg);
}

void StepAxis::onPulseFinishedForTimer(TIM_HandleTypeDef* htim)
{
    if (htim == nullptr) return;
    for (size_t i = 0; i < s_registeredAxisCount; ++i) {
        StepAxis* const axis = s_registeredAxes[i];
        if (axis == nullptr) {
            continue;
        }
        if (axis->timer() != htim) continue;
        axis->onPulseFinished();
        return;
    }
}

void StepAxis::registerInstance(StepAxis* axis)
{
    if (axis == nullptr) return;
    for (size_t i = 0; i < s_registeredAxisCount; ++i) {
        if (s_registeredAxes[i] == axis) return;
    }
    if (s_registeredAxisCount < kMaxRegisteredAxes) {
        s_registeredAxes[s_registeredAxisCount] = axis;
        ++s_registeredAxisCount;
    }
}

uint32_t StepAxis::angleToSteps(float delta_deg) const
{
    const float abs_deg = (delta_deg >= 0.0f) ? delta_deg : -delta_deg;
    const float steps_per_rev =
        (360.0f / step_deg_) * static_cast<float>(microstep_) * gear_ratio_;
    return static_cast<uint32_t>(lroundf(abs_deg * steps_per_rev / 360.0f));
}

void StepAxis::startMoveRelativeDeg(float delta_deg)
{
    const uint32_t steps = angleToSteps(delta_deg);
    if (steps == 0U) return;

    const bool positive = (delta_deg >= 0.0f);
    const GPIO_PinState dir_state =
        ((positive && dir_positive_high_) || (!positive && !dir_positive_high_))
            ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(dir_port_, dir_pin_, dir_state);

    remaining_steps_ = steps;
    running_ = true;
    if (HAL_TIM_PWM_Start_IT(timer_, channel_) != HAL_OK) {
        running_ = false;
        remaining_steps_ = 0U;
    }
}
