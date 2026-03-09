#include "main.h"
#include "usart.h"
#include "motor_command_receiver.hpp"

extern MotorCommandReceiver motor_command_receiver;

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart != &huart2)
    {
        return;
    }

    motor_command_receiver.onUart2RxComplete();
}
