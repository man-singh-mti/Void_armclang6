#include "can.h"
// #include "mti_can.h"
#include "vmt_uart.h"
#include "mti_system.h"
#include <string.h>

CAN_RxHeaderTypeDef rxHeader;                              // CAN Bus Transmit Header
CAN_TxHeaderTypeDef txHeader;                              // CAN Bus Receive Header
uint8_t             canRX[8] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // CAN Bus Receive Buffer
CAN_FilterTypeDef   canfil;                                // CAN Bus Filter
uint32_t            canMailbox;                            // CAN Bus Mail box variable

bool can_setup(void)
{
#ifdef PCB_CANBUS
    canfil.FilterBank           = 0;
    canfil.FilterMode           = CAN_FILTERMODE_IDMASK;
    canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
    canfil.FilterIdHigh         = 0xA2 << 5;
    canfil.FilterIdLow          = 0;
    canfil.FilterMaskIdHigh     = 0;
    canfil.FilterMaskIdLow      = 0;
    canfil.FilterScale          = CAN_FILTERSCALE_32BIT;
    canfil.FilterActivation     = ENABLE;
    canfil.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan1, &canfil); // Initialize CAN Filter
    HAL_CAN_Start(&hcan1);                 // Initialize CAN Bus
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK)
        return true; // Initialize CAN Bus Rx Interrupt
#endif
    return false;
}

bool can_send(uint32_t ID, uint8_t message)
{
#ifdef PCB_CANBUS
    txHeader.DLC                = 1; // Number of bites to be transmitted max- 8
    txHeader.IDE                = CAN_ID_EXT;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.StdId              = ID;
    txHeader.ExtId              = ID;
    txHeader.TransmitGlobalTime = DISABLE;
    uart_tx_channel_set(UART_DEBUG);
    printf("CAN TX (0x%02X) 0x%02X\n", ID, message);
    uart_tx_channel_undo();
    uint8_t csend[1];
    csend[0] = message;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, csend, &canMailbox) == HAL_OK)
        return true; // Send Message
#endif
    return false;
}

bool can_send_array(uint32_t ID, uint8_t *message, size_t length)
{
#ifdef PCB_CANBUS
    txHeader.DLC                = length; // Number of bites to be transmitted max- 8
    txHeader.IDE                = CAN_ID_EXT;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.StdId              = ID;
    txHeader.ExtId              = ID;
    txHeader.TransmitGlobalTime = DISABLE;

    char data_hex[17];
    memset(data_hex, 0, 17);
    for (int i = 0; i < length; i++)
    {
        sprintf(data_hex, "%s%02X", data_hex, message[i]);
    }
    uart_tx_channel_set(UART_DEBUG);
    printf("CAN TX (0x%02X) 0x%s\n", ID, data_hex);
    uart_tx_channel_undo();
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, message, &canMailbox) == HAL_OK)
        return true; // Send Message
#endif
    return false;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
    if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, canRX) == HAL_OK)
    { // Receive CAN bus message to canRX buffer
        char data_hex[17];
        memset(data_hex, 0, 17);
        for (int i = 0; i < rxHeader.DLC; i++)
        {
            sprintf(data_hex, "%s%02X", data_hex, canRX[i]);
        }
        uart_tx_channel_set(UART_DEBUG);
        printf("CAN RX: (0x%02X) 0x%s\n", rxHeader.ExtId, data_hex);


        if (rxHeader.ExtId == CAN_CMD && canRX[0] == CAN_STATUS)
        { // For PCB testing
            can_send(CAN_ID_STATUS, RADAR_READY);
        }
        else if (rxHeader.ExtId == CAN_ID_STATUS)
        { // Sensor status
            radar_status_set(canRX[0]);
        }
    }
    else
    {
        printf("CAN RX ERROR\n");
    }
    uart_tx_channel_undo();
}
