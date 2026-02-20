#ifndef __VMT_ICM20948_H
#define __VMT_ICM20948_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/* define */
#define ICM20948_VER_MAIN (0)
#define ICM20948_VER_SUB  (3)

#define ICM20948_POWER_ON_DELAY (15)

#define ICM20948_SPI_FREQ_MAX (7000000) // Hz

#define ICM20948_SAMPLE_RATE_DEFAULT (1000.0)
#define ICM20948_SAMPLE_TIME_DEFAULT (1000.0 / (ICM20948_SAMPLE_RATE_DEFAULT))

#define ICM20948_DEV_ADDRESS (0x69) /* 0b110 1001 */
// #define ICM20948_DEV_ADDRESS (0x68) /* 0b110 1000 */

#define AK09916_DEV_ADDR (0x0C) /* 0b000 1100 */

#define ICM20948_MOTION_THRSHOLD_DEFAULT (1.020) // G

#define ICM20948_ERROR_RETRY_N_DEFAULT (5)

/* type define */
typedef enum icm20948_pin_
{
    ICM20948_PIN_PWR,
    ICM20948_PIN_SDO,
    ICM20948_PIN_AD0 = ICM20948_PIN_SDO,
    ICM20948_PIN_FSYNC,
    ICM20948_PIN_INT,
    ICM20948_PIN_CS,
    ICM20948_PIN_SCL,
    ICM20948_PIN_SCLK = ICM20948_PIN_SCL,
    ICM20948_PIN_SDI,
    ICM20948_PIN_SDA = ICM20948_PIN_SDI,
    ICM20948_PIN_NUM,
} icm20948_pin_t;

typedef enum icm20948_reg_
{
    ICM20948_REG_REG_BANK_SEL = 0x7F,
    /* REGISTER BANK 0 */
    ICM20948_REG_WHO_AM_I             = 0x00,
    ICM20948_REG_USER_CTRL            = 0x03,
    ICM20948_REG_LP_CONFIG            = 0x05,
    ICM20948_REG_PWR_MGMT_1           = 0x06,
    ICM20948_REG_PWR_MGMT_2           = 0x07,
    ICM20948_REG_INT_PIN_CFG          = 0x0F,
    ICM20948_REG_INT_ENABLE           = 0x10,
    ICM20948_REG_INT_ENABLE_1         = 0x11,
    ICM20948_REG_INT_ENABLE_2         = 0x12,
    ICM20948_REG_INT_ENABLE_3         = 0x13,
    ICM20948_REG_I2C_MST_STATUS       = 0x17,
    ICM20948_REG_INT_STATUS           = 0x19,
    ICM20948_REG_INT_STATUS_1         = 0x1A,
    ICM20948_REG_INT_STATUS_2         = 0x1B,
    ICM20948_REG_INT_STATUS_3         = 0x1C,
    ICM20948_REG_DELAY_TIMEH          = 0x28,
    ICM20948_REG_DELAY_TIMEL          = 0x29,
    ICM20948_REG_ACCEL_XOUT_H         = 0x2D,
    ICM20948_REG_ACCEL_XOUT_L         = 0x2E,
    ICM20948_REG_ACCEL_YOUT_H         = 0x2F,
    ICM20948_REG_ACCEL_YOUT_L         = 0x30,
    ICM20948_REG_ACCEL_ZOUT_H         = 0x31,
    ICM20948_REG_ACCEL_ZOUT_L         = 0x32,
    ICM20948_REG_GYRO_XOUT_H          = 0x33,
    ICM20948_REG_GYRO_XOUT_L          = 0x34,
    ICM20948_REG_GYRO_YOUT_H          = 0x35,
    ICM20948_REG_GYRO_YOUT_L          = 0x36,
    ICM20948_REG_GYRO_ZOUT_H          = 0x37,
    ICM20948_REG_GYRO_ZOUT_L          = 0x38,
    ICM20948_REG_TEMP_OUT_H           = 0x39,
    ICM20948_REG_TEMP_OUT_L           = 0x3A,
    ICM20948_REG_EXT_SLV_SENS_DATA_00 = 0x3B,
    ICM20948_REG_EXT_SLV_SENS_DATA_01 = 0x3C,
    ICM20948_REG_EXT_SLV_SENS_DATA_02 = 0x3D,
    ICM20948_REG_EXT_SLV_SENS_DATA_03 = 0x3E,
    ICM20948_REG_EXT_SLV_SENS_DATA_04 = 0x3F,
    ICM20948_REG_EXT_SLV_SENS_DATA_05 = 0x40,
    ICM20948_REG_EXT_SLV_SENS_DATA_06 = 0x41,
    ICM20948_REG_EXT_SLV_SENS_DATA_07 = 0x42,
    ICM20948_REG_EXT_SLV_SENS_DATA_08 = 0x43,
    ICM20948_REG_EXT_SLV_SENS_DATA_09 = 0x44,
    ICM20948_REG_EXT_SLV_SENS_DATA_10 = 0x45,
    ICM20948_REG_EXT_SLV_SENS_DATA_11 = 0x46,
    ICM20948_REG_EXT_SLV_SENS_DATA_12 = 0x47,
    ICM20948_REG_EXT_SLV_SENS_DATA_13 = 0x48,
    ICM20948_REG_EXT_SLV_SENS_DATA_14 = 0x49,
    ICM20948_REG_EXT_SLV_SENS_DATA_15 = 0x4A,
    ICM20948_REG_EXT_SLV_SENS_DATA_16 = 0x4B,
    ICM20948_REG_EXT_SLV_SENS_DATA_17 = 0x4C,
    ICM20948_REG_EXT_SLV_SENS_DATA_18 = 0x4D,
    ICM20948_REG_EXT_SLV_SENS_DATA_19 = 0x4E,
    ICM20948_REG_EXT_SLV_SENS_DATA_20 = 0x4F,
    ICM20948_REG_EXT_SLV_SENS_DATA_21 = 0x50,
    ICM20948_REG_EXT_SLV_SENS_DATA_22 = 0x51,
    ICM20948_REG_EXT_SLV_SENS_DATA_23 = 0x52,
    ICM20948_REG_FIFO_EN_1            = 0x66,
    ICM20948_REG_FIFO_EN_2            = 0x67,
    ICM20948_REG_FIFO_RST             = 0x68,
    ICM20948_REG_FIFO_MODE            = 0x69,
    ICM20948_REG_FIFO_COUNTH          = 0x70,
    ICM20948_REG_FIFO_COUNTL          = 0x71,
    ICM20948_REG_FIFO_R_W             = 0x72,
    ICM20948_REG_DATA_RDY_STATUS      = 0x74,
    ICM20948_REG_FIFO_CFG             = 0x76,
    /* REGISTER BANK 1 */
    ICM20948_REG_SELF_TEST_X_GYRO        = 0x02,
    ICM20948_REG_SELF_TEST_Y_GYRO        = 0x03,
    ICM20948_REG_SELF_TEST_Z_GYRO        = 0x04,
    ICM20948_REG_SELF_TEST_X_ACCEL       = 0x0E,
    ICM20948_REG_SELF_TEST_Y_ACCEL       = 0x0F,
    ICM20948_REG_SELF_TEST_Z_ACCEL       = 0x10,
    ICM20948_REG_XA_OFFS_H               = 0x14,
    ICM20948_REG_XA_OFFS_L               = 0x15,
    ICM20948_REG_YA_OFFS_H               = 0x17,
    ICM20948_REG_YA_OFFS_L               = 0x18,
    ICM20948_REG_ZA_OFFS_H               = 0x1A,
    ICM20948_REG_ZA_OFFS_L               = 0x1B,
    ICM20948_REG_TIMEBASE_CORRECTION_PLL = 0x28,
    /* REGISTER BANK 2 */
    ICM20948_REG_GYRO_SMPLRT_DIV    = 0x00,
    ICM20948_REG_GYRO_CONFIG_1      = 0x01,
    ICM20948_REG_GYRO_CONFIG_2      = 0x02,
    ICM20948_REG_XG_OFFS_USRH       = 0x03,
    ICM20948_REG_XG_OFFS_USRL       = 0x04,
    ICM20948_REG_YG_OFFS_USRH       = 0x05,
    ICM20948_REG_YG_OFFS_USRL       = 0x06,
    ICM20948_REG_ZG_OFFS_USRH       = 0x07,
    ICM20948_REG_ZG_OFFS_USRL       = 0x08,
    ICM20948_REG_ODR_ALIGN_EN       = 0x09,
    ICM20948_REG_ACCEL_SMPLRT_DIV_1 = 0x10,
    ICM20948_REG_ACCEL_SMPLRT_DIV_2 = 0x11,
    ICM20948_REG_ACCEL_INTEL_CTRL   = 0x12,
    ICM20948_REG_ACCEL_WOM_THR      = 0x13,
    ICM20948_REG_ACCEL_CONFIG       = 0x14,
    ICM20948_REG_ACCEL_CONFIG_2     = 0x15,
    ICM20948_REG_FSYNC_CONFIG       = 0x52,
    ICM20948_REG_TEMP_CONFIG        = 0x53,
    ICM20948_REG_MOD_CTRL_USR       = 0x54,
    /* REGISTER BANK 3 */
    ICM20948_REG_I2C_MST_ODR_CONFIG = 0x00,
    ICM20948_REG_I2C_MST_CTRL       = 0x01,
    ICM20948_REG_I2C_MST_DELAY_CTRL = 0x02,
    ICM20948_REG_I2C_SLV0_ADDR      = 0x03,
    ICM20948_REG_I2C_SLV0_REG       = 0x04,
    ICM20948_REG_I2C_SLV0_CTRL      = 0x05,
    ICM20948_REG_I2C_SLV0_DO        = 0x06,
    ICM20948_REG_I2C_SLV1_ADDR      = 0x07,
    ICM20948_REG_I2C_SLV1_REG       = 0x08,
    ICM20948_REG_I2C_SLV1_CTRL      = 0x09,
    ICM20948_REG_I2C_SLV1_DO        = 0x0A,
    ICM20948_REG_I2C_SLV2_ADDR      = 0x0B,
    ICM20948_REG_I2C_SLV2_REG       = 0x0C,
    ICM20948_REG_I2C_SLV2_CTRL      = 0x0D,
    ICM20948_REG_I2C_SLV2_DO        = 0x0E,
    ICM20948_REG_I2C_SLV3_ADDR      = 0x0F,
    ICM20948_REG_I2C_SLV3_REG       = 0x10,
    ICM20948_REG_I2C_SLV3_CTRL      = 0x11,
    ICM20948_REG_I2C_SLV3_DO        = 0x12,
    ICM20948_REG_I2C_SLV4_ADDR      = 0x13,
    ICM20948_REG_I2C_SLV4_REG       = 0x14,
    ICM20948_REG_I2C_SLV4_CTRL      = 0x15,
    ICM20948_REG_I2C_SLV4_DO        = 0x16,
    ICM20948_REG_I2C_SLV4_DI        = 0x17,
} icm20948_reg_t;

typedef enum ak09916_reg_
{
    AK09916_REG_WIA1  = 0x00,
    AK09916_REG_WIA2  = 0x01,
    AK09916_REG_RSV1  = 0x02,
    AK09916_REG_RSV2  = 0x03,
    AK09916_REG_ST1   = 0x10,
    AK09916_REG_HXL   = 0x11,
    AK09916_REG_HXH   = 0x12,
    AK09916_REG_HYL   = 0x13,
    AK09916_REG_HYH   = 0x14,
    AK09916_REG_HZL   = 0x15,
    AK09916_REG_HZH   = 0x16,
    AK09916_REG_TMPS  = 0x17,
    AK09916_REG_ST2   = 0x18,
    AK09916_REG_CNTL1 = 0x30,
    AK09916_REG_CNTL2 = 0x31,
    AK09916_REG_CNTL3 = 0x32,
} ak09916_reg_t;

typedef enum icm20948_res_
{
    ICM20948_RES_OK,
    ICM20948_RES_BUSY,
    ICM20948_RES_ERROR,
} icm20948_res_t;

typedef struct h_icm20948_i2c_
{
    uint8_t  dev_addr; // 7bit device address.
    uint8_t *p_tx_buff;
    size_t   tx_size;
    uint8_t *p_rx_buff;
    size_t   rx_size;
} h_icm20948_i2c_t;

typedef struct h_icm20948_spi_
{
    uint8_t *p_tx_buff;
    uint8_t *p_rx_buff;
    size_t   sent_size;
} h_icm20948_spi_t;

typedef enum icm20948_data_
{
    ICM20948_DATA_FIRST,
    ICM20948_DATA_AX = ICM20948_DATA_FIRST,
    ICM20948_DATA_AY,
    ICM20948_DATA_AZ,
    ICM20948_DATA_GX,
    ICM20948_DATA_GY,
    ICM20948_DATA_GZ,
    ICM20948_DATA_MX,
    ICM20948_DATA_MY,
    ICM20948_DATA_MZ,
    ICM20948_DATA_NUM,
} icm20948_data_t;

typedef enum icm20948_sent_
{
    ICM20948_SENT_SPI,
    ICM20948_SENT_I2C,
} icm20948_sent_t;

typedef struct icm20948_reg_list_
{
    bool     b_read_write : 1, b_check_set : 1, b_check_rst : 1, error_jump_en : 1;
    uint8_t  reg_addr;
    uint8_t *p_reg_data;
    uint8_t *p_reg_check;
    size_t   reg_size;
    size_t   retry_n;
    int      error_jump_shift;
    uint32_t retry_delay;
    uint32_t delay;
} h_icm20948_reg_list_t;

typedef struct h_icm20948_setting_
{
    const h_icm20948_reg_list_t *p_h_list;
    size_t                       list_len;
    size_t                       list_n_error;
    bool                         b_busy : 1, b_error : 1;
} h_icm20948_setting_t;

typedef enum icm20948_setting_seq_
{
    ICM20948_SETTING_SEQ_INIT,
    ICM20948_SETTING_SEQ_MOTION,
    ICM20948_SETTING_SEQ_LOW_POWER,
    ICM20948_SETTING_SEQ_NUM,
    ICM20948_SETTING_SEQ_Q_LEN,
} h_icm20948_setting_seq_t;

typedef struct h_icm20948_debug_
{
    bool b_init : 1, b_motion_det : 1, b_low_power : 1, b_setting : 1, b_sample : 1;
} h_icm20948_debug_t;

typedef struct h_icm20948_init_p_
{
    uint8_t              step;
    uint32_t             time_log;
    h_icm20948_setting_t h_setting;
} h_icm20948_init_p_t;

typedef struct h_icm20948_motion_p_
{
    uint8_t              step;
    bool                 b_motion_det_en_log;
    h_icm20948_setting_t h_setting;
} h_icm20948_motion_det_p_t;

typedef struct h_icm20948_low_power_p_
{
    uint8_t              step;
    bool                 b_low_power_log;
    h_icm20948_setting_t h_setting;
} h_icm20948_low_power_p_t;

typedef struct h_icm20948_setting_p_
{
    uint8_t                step;
    h_icm20948_setting_t  *p_h_setting;
    size_t                 reg_list_n;
    size_t                 h_reg_list_len;
    size_t                 retry_count;
    size_t                 retry_n;
    h_icm20948_reg_list_t *p_h_list;
    uint8_t                tx_buff[2];
    uint8_t                rx_buff[2];
    h_icm20948_i2c_t       h_i2c_sent;
    h_icm20948_spi_t       h_spi_sent;
    bool                   b_error;
    uint32_t               time_log;
    uint32_t               time_delay;
} h_icm20948_setting_p_t;

#define ICM20948_SAMPLE_REG_ADDR_BEGIN (ICM20948_REG_ACCEL_XOUT_H)
#define ICM20948_SAMPLE_REG_ADDR_END   (ICM20948_REG_EXT_SLV_SENS_DATA_08)
#define ICM20948_SAMPLE_REG_BUFF_LEN   (1 + ICM20948_SAMPLE_REG_ADDR_END - ICM20948_SAMPLE_REG_ADDR_BEGIN + 1)

typedef struct h_icm20948_sample_p_
{
    uint32_t         timeout;
    uint32_t         time_log;
    uint8_t          step;
    h_icm20948_i2c_t h_i2c_sent;
    h_icm20948_spi_t h_spi_sent;
    uint8_t          tx_buff[ICM20948_SAMPLE_REG_BUFF_LEN];
    uint8_t          rx_buff[ICM20948_SAMPLE_REG_BUFF_LEN];
} h_icm20948_sample_p_t;

typedef struct h_icm20948_
{
    /* api parameter */
    uint32_t (*systick_get_cb)(void); // millisecond system tick get callback
    void (*pin_set_cb)(uint8_t id, icm20948_pin_t pin, bool b_status);
    bool (*i2c_sent_cb)(uint8_t id, h_icm20948_i2c_t *p_h_i2c);
    bool (*spi_sent_cb)(uint8_t id, h_icm20948_spi_t *p_h_spi);
    void (*sample_finish_cb)(uint8_t id);
    void (*init_error_cb)(uint8_t id);

    uint8_t         id;
    icm20948_sent_t sent_sel;

    bool b_init_finish;
    bool b_sample_en;
    bool b_motion_det_en; /* motion detect */
    bool b_low_power;
    bool b_sample_start;

    h_icm20948_debug_t h_debug; /* debug printf enable flag */

    int16_t raw_data[ICM20948_DATA_NUM];

    /* process parameter */
    h_icm20948_init_p_t       h_init_p;
    h_icm20948_motion_det_p_t h_motion_det_p;
    h_icm20948_low_power_p_t  h_low_power_p;
    h_icm20948_setting_p_t    h_setting_p;
    h_icm20948_sample_p_t     h_sample_p;

    uint8_t        sent_error_count;
    bool           b_sent_busy;
    icm20948_res_t sent_res;

    h_icm20948_setting_t *p_h_setting_q[ICM20948_SETTING_SEQ_Q_LEN];
    size_t                setting_q_back;
    size_t                setting_q_front;

    bool b_motion_det_en_current;
    bool b_low_power_current;
} h_icm20948_t;

/* function */
// uint32_t icm20948_systick_get_cb(void);
// void icm20948_pin_set_cb(icm20948_pin_t pin, bool b_status);
// bool icm20948_pin_get_cb(icm20948_pin_t pin);
// void icm20948_init_error_cb(void);
// void icm20948_sample_finish_cb(void);
// bool icm20948_i2c_sent_cb(h_icm20948_i2c_t *p_h_i2c);
// bool icm20948_spi_sent_cb(h_icm20948_spi_t *p_h_spi);
/* reutrn value: true = OK, false = retry. */
bool icm20948_sent_finish(h_icm20948_t *p_h_icm, icm20948_res_t res);

// void icm20948_sample_start(h_icm20948_t *p_h_icm); //sample data once

/* return value: true = busy, false = idle. */
bool icm20948_process(h_icm20948_t *p_h_icm);

// bool icm20948_init_finish_get(void);

// void icm20948_debug_set(icm20948_debug_t *p_handler);
// void icm20948_debug_get(icm20948_debug_t *p_handler);

// bool icm20948_en_set(bool b_enable);
// bool icm20948_en_get(void);

// void icm20948_timer_sample_ms_set(double sample_time);
// double icm20948_timer_sample_ms_get(void);

void icm20948_motion_detect_set(h_icm20948_t *p_h_icm, bool b_enable);
bool icm20948_motion_detect_get(h_icm20948_t *p_h_icm);

void icm20948_low_power_set(h_icm20948_t *p_h_icm, bool b_enable);
bool icm20948_low_power_get(h_icm20948_t *p_h_icm);

// int16_t icm20948_data_get(icm20948_data_t data_n);

#ifdef __cplusplus
}
#endif
#endif /* __VMT_ICM20948_H */
