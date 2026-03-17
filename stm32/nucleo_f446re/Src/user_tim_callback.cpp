#include <cstdint>
#include "tim.h"
#include "c620_can.hpp"
#include "c620_control.hpp"
#include "step_axis.hpp"
#include "uart_sender.hpp"

extern C620CAN c620_can;
extern C620Control c620_control;
extern UartSender uart_sender;

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim6)
    {
        c620_control.onTimerTick();
    }
    else if(htim == &htim7)
    {
        uart_sender.onTimerTick();
    }
}

extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    StepAxis::onPulseFinishedForTimer(htim);
}
