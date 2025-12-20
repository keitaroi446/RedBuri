#include "c620_can.hpp"
#include <cmath>

C620CAN::C620CAN() 
: target_currents_raw{0},
  angles_raw{0},
  speeds_rpm{0},
  currents_raw{0},
  temps_degc{0}
{
    // C620のCAN_ID: 0x201~0x208
    CAN_FilterTypeDef filter;
    filter.FilterIdHigh         = 0x200 << 5;
    filter.FilterIdLow          = 0;
    filter.FilterMaskIdHigh     = 0x7F0 << 5;
    filter.FilterMaskIdLow      = 0;
    filter.FilterScale          = CAN_FILTERSCALE_16BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.SlaveStartFilterBank = 14;
    filter.FilterActivation     = ENABLE;

    HAL_CAN_ConfigFilter(&hcan1, &filter);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void C620CAN::setCurrent(uint8_t motor_id, float target_current_amp) 
{
    constexpr float LIMIT_CURRENT_AMP = 6.0f;      // ユーザー定義の電流制限[A]
    constexpr float MAX_CURRENT_AMP = 20.0f;       // 最大電流[A]
    constexpr int16_t MAX_CURRENT_RAW = 16384;     // 最大電流(16bit)

    if(LIMIT_CURRENT_AMP > MAX_CURRENT_AMP) return;
    if(motor_id == 0 || motor_id > 8) return;
    if(std::fabs(target_current_amp) >= LIMIT_CURRENT_AMP)
    {
        target_current_amp = std::copysign(LIMIT_CURRENT_AMP, target_current_amp);
    }

    float current_raw = (target_current_amp / MAX_CURRENT_AMP) * MAX_CURRENT_RAW;
    target_currents_raw[motor_id - 1] = static_cast<int16_t>(current_raw);
}

void C620CAN::sendCurrents()
{
    CAN_TxHeaderTypeDef tx_deader;
    uint32_t tx_mailbox;
    uint8_t tx_data[8];

    if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) >= 2)
    {
        // モーターID: 1~4
        tx_deader.StdId = 0x200;
        tx_deader.IDE = CAN_ID_STD;
        tx_deader.RTR = CAN_RTR_DATA;
        tx_deader.DLC = 8;
        tx_deader.TransmitGlobalTime = DISABLE;
        for (int i = 0; i < 4; i++)
        {
            int16_t current = target_currents_raw[i];
            tx_data[2*i] = (current >> 8) & 0xFF;
            tx_data[2*i + 1] = current & 0xFF;
        }

        HAL_CAN_AddTxMessage(&hcan1, &tx_deader, tx_data, &tx_mailbox);

        // モーターID: 5~6
        tx_deader.StdId = 0x1FF;
        for (int i = 0; i < 4; i++)
        {
            int16_t current = target_currents_raw[i + 4];
            tx_data[2*i]     = (current >> 8) & 0xFF;
            tx_data[2*i + 1] = current & 0xFF;
        }

        HAL_CAN_AddTxMessage(&hcan1, &tx_deader, tx_data, &tx_mailbox);
    }
}

void C620CAN::readMotorStatus() 
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
    {
        if(rx_header.StdId < 0x201 || rx_header.StdId > 0x208) return;
        const uint8_t idx = rx_header.StdId - 0x201;

        angles_raw[idx]   = static_cast<uint16_t>(rx_data[0] << 8 | rx_data[1]);
        speeds_rpm[idx]   = static_cast<int16_t>(rx_data[2] << 8 | rx_data[3]);
        currents_raw[idx] = static_cast<int16_t>(rx_data[4] << 8 | rx_data[5]);
        temps_degc[idx]    = rx_data[6];
    }
}

float C620CAN::getAngle(uint8_t motor_id)
{
    constexpr uint16_t MAX_ANGLE_RAW = 8191.0f; // 最大角度(16bit)

    if(motor_id == 0 || motor_id > 8) return -1.0f;
    return 360 * angles_raw[motor_id - 1] / MAX_ANGLE_RAW; // モーター角度範囲は0~360°
}

float C620CAN::getSpeed(uint8_t motor_id)
{
    if(motor_id == 0 || motor_id > 8) return -1.0f;
    return speeds_rpm[motor_id - 1];
}

float C620CAN::getCurrent(uint8_t motor_id)
{
    constexpr float MAX_CURRENT_AMP = 20.0f;   // 最大電流[A]
    constexpr int16_t MAX_CURRENT_RAW = 16384; // 最大電流(16bit)

    if(motor_id == 0 || motor_id > 8) return -1.0f;
    return MAX_CURRENT_AMP * currents_raw[motor_id - 1] / MAX_CURRENT_RAW;
}

float C620CAN::getTemp(uint8_t motor_id)
{
    if(motor_id == 0 || motor_id > 8) return -1.0f;
    return temps_degc[motor_id - 1];
}