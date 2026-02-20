#include "stm32f7xx.h"
#include <stdbool.h>

#define CAN_CMD        0x80
#define CAN_START      0x00
#define CAN_STOP       0x01
#define CAN_CAL        0x02
#define CAN_POWER      0x03
#define CAN_STATUS     0x04
#define CAN_ID_HEADER  0xA0
#define CAN_ID_OBJECT  0xA1
#define CAN_ID_PROFILE 0xA2
#define CAN_ID_STATUS  0xA3

typedef enum
{
    RADAR_INITIALISING,
    RADAR_READY,
    RADAR_CHIRPING,
    RADAR_STOPPED,
} radar_status_t;

bool can_setup(void);
bool can_send(uint32_t ID, uint8_t message);
bool can_send_array(uint32_t ID, uint8_t *message, size_t length);
