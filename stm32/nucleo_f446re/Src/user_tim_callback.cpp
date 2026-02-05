#include "tim.h"
#include "c620_control.hpp"

extern C620Control c620_control;

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6)
    {
        c620_control.update();
    }
}
