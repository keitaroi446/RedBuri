#include "tim.h"
#include "step_axis.hpp"
#include <cstdint>

volatile uint8_t g_tim6_tick = 0;

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim6)
    {
        g_tim6_tick = 1;
    }
}

extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    StepAxis::onPulseFinishedForTimer(htim);
}
