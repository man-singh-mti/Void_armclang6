#ifndef __VMT_DEVICE_H__
#define __VMT_DEVICE_H__

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/* Dependencies required for public struct definitions in this header */
#include "vmt_icm20948.h"
#include "vmt_adc.h"
#include "vmt_uart.h"
#include "vmt_i2c_async.h"
#include "pca9534.h"
#include "xm125_async.h"
#include "mti_radar.h"
#include "mti_void.h"

// #pragma anon_unions

/* ==========================================================================
   CONSTANTS & VERSIONS
   ========================================================================== */
#define FW_VER_MAJOR (0)
#define FW_VER_MINOR (1)
#define FW_VER_SUB   (0)

#define DEV_IMU_NUM          (2)
#define DEV_SPI_CH_NUM       (DEV_IMU_NUM)
#define IMU_SAMPLE_HAL_TIMER (htim7)

/* Sensor Configuration */
#define DEV_ADC_SAMPLE_RATE_DEFAULT (400)
#define DEV_IMU_SAMPLE_RATE         (1000.0)
#define DEV_IMU_SAMPLE_PERIOD       (1000.0 / DEV_IMU_SAMPLE_RATE)
#define DEV_IMU_AVG_TIME            (200.0)
#define DEV_IMU_AVG_SAMPLE_NUM      ((size_t)(DEV_IMU_SAMPLE_RATE * DEV_IMU_AVG_TIME / 1000.0))
#define DEV_IMU_AVG_BUFF_SIZE       ((size_t)DEV_IMU_AVG_SAMPLE_NUM + 1)
#define DEV_IMU_VECTOR_AXIS_N       (3)

/* Radar Configuration */
#define XM125_ENABLED_BANKS ((1u << (VMT_I2C_BANK1 - 1)) | (1u << (VMT_I2C_BANK2 - 1)))

extern const double math_pi;

/* ==========================================================================
   TYPE DEFINITIONS
   ========================================================================== */

/* --- Math & Statistics --- */
typedef struct h_moving_avg_i16_
{
    int16_t *p_buff;
    size_t   buff_len;
    size_t   data_len;
    size_t   data_n;
    double   sum;
    double   avg;
} h_moving_avg_i16_t;

typedef struct h_moving_avg_f_
{
    float *p_buff;
    size_t buff_len;
    size_t data_len;
    size_t data_n;
    double sum;
    double avg;
} h_moving_avg_f_t;

typedef enum imu_vector_
{
    IMU_VECTOR_ACCEL,
    IMU_VECTOR_GYRO,
    IMU_VECTOR_MAG,
    IMU_VECTOR_NUM,
} imu_vector_t;

typedef struct h_vector_
{
    int16_t  data_org[DEV_IMU_VECTOR_AXIS_N];
    uint32_t squ;
    double   len;
    double   angle;
} h_vector_t;

/* --- Sensor Structures --- */
typedef struct sensor_adc_
{
    double volt;
    double thre_h; // Schmitt trigger threshold high
    double thre_l; // Schmitt trigger threshold low
    bool   b_level;
} h_sensor_adc_t;

typedef struct h_sensor_
{
    union
    {
        uint16_t adc_value[ADC_SEQ_NUM];
        struct
        {
            uint16_t power;
            union
            {
                uint16_t water[ADC_SEQ_WATER_NUM];
                struct
                {
                    uint16_t water_1;
                    uint16_t water_2;
                };
            };
            uint16_t temp_wall;
            union
            {
                uint16_t dist[ADC_SEQ_DIST_NUM];
                struct
                {
                    uint16_t dist_1;
                    uint16_t dist_2;
                    uint16_t dist_3;
                };
            };
        };
    };
    h_sensor_adc_t h_adc[ADC_SEQ_NUM];
} h_sensor_t;

typedef struct h_imu_
{
    uint8_t       id;
    h_icm20948_t *p_h_icm20948;
    union
    {
        int16_t data[ICM20948_DATA_NUM];
        struct
        {
            int16_t accel_x, accel_y, accel_z;
            int16_t gyro_x, gyro_y, gyro_z;
            int16_t mag_x, mag_y, mag_z;
        };
    };

    uint8_t             step;
    bool                b_g_log_en : 1, b_g_log_busy : 1, b_update : 1, b_overflow : 1;
    h_moving_avg_i16_t *p_h_g_squ_avg[DEV_IMU_VECTOR_AXIS_N];
    int16_t             gravity[DEV_IMU_VECTOR_AXIS_N];
    double              gravity_len;
    h_vector_t          h_accel;
    h_vector_t          h_gyro;
} h_imu_t;

/* --- Logging Structures --- */
typedef struct h_dev_log_imu_
{
    union
    {
        int16_t accel[DEV_IMU_VECTOR_AXIS_N];
        struct
        {
            int16_t acc_x;
            int16_t acc_y;
            int16_t acc_z;
        };
    };
    int16_t angle_acc;
    int16_t angle_gyro;
} h_dev_log_imu_t;

typedef struct h_dev_log_
{
    uint32_t        time;
    h_dev_log_imu_t h_imu[DEV_IMU_NUM];
} h_dev_log_t;

typedef struct
{
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t acc_x_max;
    int16_t acc_y_max;
    int16_t acc_z_max;
    int16_t acc_y_bottom;
    int16_t amplitude;
    int16_t amplitude_max;
} flash_values_t;

/* --- Hardware Abstraction --- */
typedef uint32_t (*clk_source_get_t)(void);

typedef struct h_dev_spi_send_ h_dev_spi_send_t;
struct h_dev_spi_send_
{
    uint8_t  channel;
    uint8_t *p_tx_buff;
    uint8_t *p_rx_buff;
    size_t   data_size;
    void (*finish_cb)(h_dev_spi_send_t *p_h_spi);
};

typedef enum dev_res_
{
    DEV_RES_ERROR,
    DEV_RES_SUCCESS,
    DEV_RES_BUSY,
    DEV_RES_ERROR_PARA,
    DEV_RES_NUM,
} dev_res_t;

typedef union h_dev_debug_
{
    uint8_t byte;
    struct
    {
        bool b_init           : 1;
        bool b_spi_init       : 1;
        bool b_imu_sample_set : 1;
        bool b_adc_sample     : 1;
        bool b_imu_sample     : 1;
        bool b_void_sample    : 1;
    };
} h_dev_debug_t;

/* ==========================================================================
   API: CORE LIFECYCLE
   ========================================================================== */
void device_init_start(void);
bool device_init_finish_get(void);
void device_process(void);
void device_restart(void);
void dev_sleep(void);
bool device_is_radar_enabled(void);

/* ==========================================================================
   API: DEBUG & TELEMETRY
   ========================================================================== */
void dev_printf_debug_set(h_dev_debug_t *p_h_flag);
void dev_printf_debug_get(h_dev_debug_t *p_h_flag);
void dev_dump_printf_set(bool b_enable);
bool dev_dump_get(void);
void dev_water_det_printf_set(bool b_enable);
bool dev_water_det_get(void);

/* ==========================================================================
   API: SENSORS & IMU
   ========================================================================== */
void imu_sample_rate_set(double sample_rate);
void imu_g_log_en_set(uint8_t imu_select, bool b_enable);
bool imu_g_log_busy_get(uint8_t imu_select);
bool imu_overflow_get(uint8_t imu_select);
void imu_overflow_clear(uint8_t imu_select);
void imu_update_finish_cb(void);

void dev_sensor_power_set(bool b_enable);
bool dev_sensor_power_get(void);
void dev_sensor_volt_get(double voltage[ADC_SEQ_EXT_NUM]);
void dev_sensor_level_get(bool b_level[ADC_SEQ_EXT_NUM]);

/* ==========================================================================
   API: LOGGING & FLASH
   ========================================================================== */
bool dev_log_erase_all_set(void);
bool dev_log_erase_busy_get(void);
bool dev_log_save_enable_set(bool b_enable);
bool dev_log_save_busy_get(void);
bool dev_log_save_overflow_get(void);
bool dev_log_report_enable_set(bool b_enable);
bool dev_log_report_busy_get(void);
void dev_log_report_uart_ch_set(uart_select_t channel);
void dev_log_data_get(h_dev_log_t *p_h_log);
void flash_data_reset(void);

/* ==========================================================================
   API: HARDWARE (SPI)
   ========================================================================== */
dev_res_t dev_spi_send(h_dev_spi_send_t *p_h_spi);

#endif //__VMT_DEVICE_H__
