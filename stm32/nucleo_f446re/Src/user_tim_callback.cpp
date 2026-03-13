#include <cstdint>
#include "tim.h"
#include "c620_control.hpp"
#include "step_axis.hpp"

extern C620Control c620_control;

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim6)
    {
        c620_control.updateSpeedControl();
    }
}

extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    StepAxis::onPulseFinishedForTimer(htim);
}
