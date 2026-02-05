#include "main.h"
#include "c620_can.hpp"

extern C620CAN c620;

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    c620.readMotorStatus();
}