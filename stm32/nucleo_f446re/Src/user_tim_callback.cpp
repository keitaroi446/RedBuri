#include "tim.h"
#include "c620_control.hpp"

extern C620Control c620_control;
namespace run { void on_step_pulse_finished(TIM_HandleTypeDef* htim); }

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6)
    {
        c620_control.update();
    }
}

extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    run::on_step_pulse_finished(htim);
}
