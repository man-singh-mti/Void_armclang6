#include "main.h"
#include "vmt_device.h"
#include "mti_can.h"

#define HW_VER 3
#if HW_VER >= 3
#define PCB_CANBUS
// #define SENSOR_RADAR //define if radar is present
#endif

enum state_enum
{
    initialising_state,
    stopped_state,
    measure_state,
    saving_state,
    debug_log_state
};

typedef enum
{
    STATUS_SYNC,
    STATUS_OK,
    STATUS_IMU_ERROR,
    STATUS_VOID_ERROR,
    STATUS_TEMP_ERROR
} status_t;

typedef enum step_
{
    STEP_START,      // 0
    STEP_VER_SYNC,   // 1
    STEP_WATER_SYNC, // 2
    STEP_IMU_SYNC,   // 3
    STEP_IMU_TEST,   // 4
    STEP_VOID,       // 5
    STEP_TEMP,       // 6
    STEP_FINISH,     // 7
} init_step_t;

void     imu_validate(h_imu_t *h_imu);
bool     debug_get(void);
bool     initialised_get(void);
bool     water_synced_get(void);
uint8_t  state_get(void);
void     state_set(uint8_t);
status_t module_status_get(void);
void     radar_status_set(radar_status_t status);
void     module_status_set(status_t status);
bool     module_init(void);
void     version_ack(bool received);
bool     keepalive_check(void);
void     keepalive_reset(void);
