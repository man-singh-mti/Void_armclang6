#include "mti_system.h"
#include "vmt_uart.h"
#include "mti_imu.h"
#include "mti_water.h"

bool           debug = true;
bool           initialised;
bool           water_synced;
radar_status_t radar_status;
uint8_t        module_status;
bool           initialised;
bool           version_sent;
init_step_t    init_step     = STEP_START;
uint8_t        retries_ver   = 0;
uint8_t        retries_water = 0;
uint8_t        retries_imu   = 0;
uint8_t        retries_radar = 0;
uint8_t        state;
uint32_t       keepalive_timer;

static const char *str_radar_status[4] = { "Not Initialised", "Ready", "Chirping", "Stopped" };

bool debug_get(void)
{
    return debug;
}

bool initialised_get(void)
{
    return initialised;
}

bool water_synced_get(void)
{
    return water_synced;
}

void radar_status_set(radar_status_t status)
{
    radar_status = status;
    printf("Radar status: %s\n", str_radar_status[radar_status]);
}

uint8_t state_get(void)
{
    return state;
}

void state_set(uint8_t set)
{
    state = set;
}

status_t module_status_get(void)
{
    return module_status;
}

void module_status_set(status_t status)
{
    module_status = status;
}

bool module_init(void)
{
    if (initialised)
        return true;
    static uint32_t timestamp;
    if (HAL_GetTick() < timestamp + 1000)
        return false;
    timestamp = HAL_GetTick();

    uart_tx_channel_set(UART_UPHOLE);

    switch (init_step)
    {
    case STEP_START:
        printf("Initialising modules\n");
        init_step = STEP_VER_SYNC;
    case STEP_VER_SYNC:
        if (version_sent)
            init_step = STEP_WATER_SYNC;
        else if (retries_ver < 3)
        {
            printf("@status,down,0,ver,%d,%d,%d\n", FW_VER_MAJOR, FW_VER_MINOR, FW_VER_SUB);
            retries_ver++;
        }
        else
        {
            printf("@db, Downhole version not acknowledged by uphole\n");
            init_step = STEP_WATER_SYNC;
        }
        break;

    case STEP_WATER_SYNC:
        if (reserve_get())
            init_step = STEP_IMU_SYNC;
        else if (retries_water < 3 || HAL_GetTick() < 8000)
        {
            printf("@status,down,0,water\n");
            retries_water++;
            break;
        }
        else
        {
            reserve_set(1280);
            printf("@db,Sync error. Setting water to %d\n", reserve_get());
            init_step = STEP_IMU_SYNC;
        }
    case STEP_IMU_SYNC:
        if (imu_profile_get() != 0)
            init_step = STEP_IMU_TEST;
        else if (retries_imu < 3)
        {
            printf("@status,down,0,imu\n");
            retries_imu++;
            break;
        }
        else
        {
            imu_profile_set(1);
            printf("@db,Sync error. Setting imu profile to %d\n", imu_profile_get());
            init_step = STEP_IMU_TEST;
        }
    case STEP_IMU_TEST:
        if (imu_active_get() == 10)
            break;
        if (imu_active_get() == 30)
        {
            printf("@status,down,2\n"); // IMU error
        }
        init_step = STEP_VOID;
    case STEP_VOID:
#ifdef SENSOR_RADAR
        if (radar_status == 0)
        {
            if (retries_radar < 5)
            {
                if (retries_radar == 0)
                    can_setup();
                can_send(CAN_CMD, CAN_STATUS);
                retries_radar++;
            }
            else
            {
                printf("@status,down,3\n"); // radar error
            }
        }
        else
            init_step = STEP_TEMP;
#else
        init_step = STEP_TEMP;
#endif

    case STEP_TEMP:
        init_step = STEP_FINISH;

    case STEP_FINISH:
        initialised = true;
        if (module_status == STATUS_SYNC)
            module_status = STATUS_OK;
        printf("@status,down,%d\n", module_status);
        return true;
    }
    return false;
}

void version_ack(bool received)
{
    if (received)
        version_sent = true;
    else
    {
        retries_ver   = 0;
        retries_water = 0;
        retries_imu   = 0;
        init_step     = STEP_START;
    }
}

void imu_validate(h_imu_t *h_imu)
{
    static bool    imu_status[2];
    static uint8_t retry;
    // if (state <= stopped_state || id == 0)
    //     return;
    // uart_tx_channel_set(UART_UPHOLE);
    uart_tx_channel_set(UART_DEBUG);
    // water_detect();


    static imu_test_t res[2];
    if (device_init_finish_get())
    {
        for (int i = 0; i < 2; i++)
        {
            res[i] = imu_test(&h_imu[i]);
            if (res[i] == IMU_TEST_OK && !imu_status[i])
            {
                imu_status[i] = true;
                printf("@db,IMU status %d = %d\n", i, imu_status[i]);
            }
            else if (res[i] == IMU_TEST_ERROR && imu_status[i])
            {
                imu_status[i] = false;
                printf("@db,IMU status %d = %d\n", i, imu_status[i]);
            }
        }
        uart_tx_channel_set(UART_UPHOLE);

        // send status to uphole
        uint8_t error_code = 0;
        if (res[0] == IMU_TEST_ERROR)
        {
            if (res[1] == IMU_TEST_ERROR)
                error_code = 2;
            else
                error_code = 3;
        }
        else if (res[1] == IMU_TEST_ERROR)
            error_code = 4;
        if (error_code > 1)
        {
            module_status = error_code;
            if (retry < 3)
            {
                printf("@db,retry IMU init\n");
                imu_test_reset();
                retry++;
            }
        }
        if (res[0] == IMU_TEST_OK)
            imu_status[0] = true;
        else if (res[0] == IMU_TEST_ERROR)
            imu_status[0] = false;
        if (res[1] == IMU_TEST_OK)
            imu_status[1] = true;
        else if (res[1] == IMU_TEST_ERROR)
            imu_status[1] = false;

        // select which IMU to use based on init status
        if (imu_status[0])
            imu_active_set(0);
        else if (imu_status[1])
            imu_active_set(1);
        else
            imu_active_set(30); // invalid
        if ((res[0] == IMU_TEST_OK && imu_status[1]) || (res[1] == IMU_TEST_OK && imu_status[0]))
        {
            // if (reserve_get() > 0)
            // {
            //     if (module_status != 5)
            //     {
            //         module_status = 5;
            //         printf("@init,down,status,%d\n", module_status);
            //     }
            // }
            // else
            // {
            //     module_status = 6;
            //     printf("@init,down,water\n");
            // }
        }
        // printf("@db,active imu: %d\n", imu_active);
        // uart_tx_channel_set(UART_DEBUG);
        // printf("@val,%lf:%lf:%f:%lf\r\n", h_imu[0].h_accel.angle, h_imu[1].h_accel.angle, h_imu[0].h_accel.len, h_imu[1].h_accel.len);
        // uart_tx_channel_undo();
        // imu_active = 1;
        uart_tx_channel_undo();
    }
}

void keepalive_reset(void)
{
    keepalive_timer = HAL_GetTick();
}

bool keepalive_check(void)
{
    if (state_get() == measure_state && HAL_GetTick() > keepalive_timer + 1000)
    {
        uart_tx_channel_set(UART_UPHOLE);
        printf("@status,down,A\n");
        keepalive_timer = HAL_GetTick();
        return true; // Keepalive message was sent
    }
    return false; // No keepalive message was needed
}
