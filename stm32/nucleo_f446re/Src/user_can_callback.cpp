#include "main.h"
#include "c620_can.hpp"

extern C620CAN c620_can;

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    c620_can.readMotorStatus();
}