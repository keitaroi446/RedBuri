#include "main.h"
#include "c620_can.hpp"
#include "c620_control.hpp"

extern C620CAN c620_can;
extern C620Control c620_control;

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    c620_can.updateMotorStatus();
    c620_control.updateCurrentAngleDeg();
}