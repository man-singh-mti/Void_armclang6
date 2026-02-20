#include "vmt_icm20948.h"

#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <gpio.h>

// #include "vmt_i2c.h"
// #include "vmt_uart.h"
// #include "vmt_device.h"

/* type define */
#define ICM20948_SPI_REG_ADDR_READ_MSK (0x80)

/* ICM20948_REG_WHO_AM_I 00 */
#define ICM20948_REG_WHO_AM_I_DEFAULT (0xEA)

/* ICM20948_REG_REG_BANK_SEL */
#define ICM20948_REG_REG_BANK_SEL_SLECECT_POS (4)
#define ICM20948_REG_REG_BANK_SEL_0           (0 << ICM20948_REG_REG_BANK_SEL_SLECECT_POS)
#define ICM20948_REG_REG_BANK_SEL_1           (1 << ICM20948_REG_REG_BANK_SEL_SLECECT_POS)
#define ICM20948_REG_REG_BANK_SEL_2           (2 << ICM20948_REG_REG_BANK_SEL_SLECECT_POS)
#define ICM20948_REG_REG_BANK_SEL_3           (3 << ICM20948_REG_REG_BANK_SEL_SLECECT_POS)

/* REG_WRITE_LP_CONFIG */
#define ICM20948_REG_LP_CONFIG_I2C_MST_CYCLE_MSK (0x40)
#define ICM20948_REG_LP_CONFIG_ACCEL_CYCLE_MSK   (0x20)
#define ICM20948_REG_LP_CONFIG_GYRO_CYCLE_MSK    (0x10)
#define ICM20948_REG_LP_CONFIG_DEFAULT           (ICM20948_REG_LP_CONFIG_I2C_MST_CYCLE_MSK)
#define ICM20948_REG_LP_CONFIG_ACCEL_CYCLE       (ICM20948_REG_LP_CONFIG_I2C_MST_CYCLE_MSK | ICM20948_REG_LP_CONFIG_ACCEL_CYCLE_MSK)

/* REG_WRITE_PWR_MGMT_1 01 */
#define ICM20948_REG_PWR_MGMT_1_DEVICE_RESET_MSK (0x80)
#define ICM20948_REG_PWR_MGMT_1_SLEEP_MSK        (0x40)
#define ICM20948_REG_PWR_MGMT_1_LP_EN_MSK        (0x20)
#define ICM20948_REG_PWR_MGMT_1_TEMP_DIS_MSK     (0x08)
#define ICM20948_REG_PWR_MGMT_1_CLKSEL_POS       (0)
typedef enum icm20948_reg_pwr_mgmt_1_clksel_
{
    ICM20948_REG_PWR_MGMT_1_CLKSEL_20MHZ = 0,
    ICM20948_REG_PWR_MGMT_1_CLKSEL_AUTO  = 1,
    ICM20948_REG_PWR_MGMT_1_CLKSEL_STOP  = 7,
} icm20948_reg_pwr_mgmt_1_clksel_t;
#define ICM20948_REG_PWR_MGMT_1_DEFAULT ((ICM20948_REG_PWR_MGMT_1_TEMP_DIS_MSK) | (ICM20948_REG_PWR_MGMT_1_CLKSEL_AUTO << ICM20948_REG_PWR_MGMT_1_CLKSEL_POS))
#define ICM20948_REG_PWR_MGMT_1_LOW_POWER \
    (ICM20948_REG_PWR_MGMT_1_LP_EN_MSK | (ICM20948_REG_PWR_MGMT_1_TEMP_DIS_MSK) | (ICM20948_REG_PWR_MGMT_1_CLKSEL_AUTO << ICM20948_REG_PWR_MGMT_1_CLKSEL_POS))

/* REG_WRITE_PWR_MGMT_2 */
#define ICM20948_REG_PWR_MGMT_2_DISABLE_ACCEL_MSK (0x31)
#define ICM20948_REG_PWR_MGMT_2_DISABLE_GYRO_MSK  (0x7)
#define ICM20948_REG_PWR_MGMT_2_EN                (0x0)
#define ICM20948_REG_PWR_MGMT_2_DIS_GYRO          (ICM20948_REG_PWR_MGMT_2_DISABLE_GYRO_MSK)

/* REG_WRITE_INT_PIN_CFG */
#define ICM20948_REG_INT_PIN_CFG_ACTL_MSK         (0x80)
#define ICM20948_REG_INT_PIN_CFG_OPEN_MSK         (0x40)
#define ICM20948_REG_INT_PIN_CFG_LATCH_EN_MSK     (0x20)
#define ICM20948_REG_INT_PIN_CFG_ANYRD_2CLEAR_MSK (0x10)
#define ICM20948_REG_INT_PIN_CFG_ACTL_FSYNC_MSK   (0x08)
#define ICM20948_REG_INT_PIN_CFG_FSYNC_INT_MSK    (0x04)
#define ICM20948_REG_INT_PIN_CFG_BYPASS_MSK       (0x02)
#define ICM20948_REG_INT_PIN_CFG_DEFAULT          (ICM20948_REG_INT_PIN_CFG_ACTL_MSK | ICM20948_REG_INT_PIN_CFG_OPEN_MSK)

/* REG_WRITE_INT_ENABLE */
#define ICM20948_REG_INT_ENABLE_WOM_INT_EN_MSK (0x08)
#define ICM20948_REG_INT_ENABLE_WOM_EN         (ICM20948_REG_INT_ENABLE_WOM_INT_EN_MSK)
#define ICM20948_REG_INT_ENABLE_WOM_DIS        (0x0)

/* REG_WRITE_GYRO_CONFIG_1  */
#define ICM20948_REG_GYRO_CONFIG_1_FCHOICE_POS (0)
#define ICM20948_REG_GYRO_CONFIG_1_FS_SEL_POS  (1)
#define ICM20948_REG_GYRO_CONFIG_1_DLPFCFG_POS (3)
typedef enum icm20948_reg_gyro_config_1_fs_sel_
{
    ICM20948_REG_GYRO_CONFIG_1_FS_SEL_250DPS  = 0,
    ICM20948_REG_GYRO_CONFIG_1_FS_SEL_500DPS  = 1,
    ICM20948_REG_GYRO_CONFIG_1_FS_SEL_1000DPS = 2,
    ICM20948_REG_GYRO_CONFIG_1_FS_SEL_2000DPS = 3,
} icm20948_reg_gyro_config_1_fs_sel_t;
#define ICM20948_REG_GYRO_CONFIG_1_DEFAULT                                                                                                    \
    ((0x1 << ICM20948_REG_GYRO_CONFIG_1_FCHOICE_POS) | (ICM20948_REG_GYRO_CONFIG_1_FS_SEL_1000DPS << ICM20948_REG_GYRO_CONFIG_1_FS_SEL_POS) | \
     (0x2 << ICM20948_REG_GYRO_CONFIG_1_DLPFCFG_POS))

/* ICM20948_REG_GYRO_SMPLRT_DIV */
#define ICM20948_REG_GYRO_SMPLRT_DIV_DEFAULT ((1125 + ICM20948_SAMPLE_RATE_DEFAULT - 1) / ICM20948_SAMPLE_RATE_DEFAULT - 1)

/* REG_WRITE_ACCEL_INTEL_CTRL */
#define ICM20948_REG_ACCEL_INTEL_MODE_INT_POS (0)
#define ICM20948_REG_ACCEL_INTEL_EN_POS       (1)
#define ICM20948_REG_ACCEL_INTEL_CTRL_EN      ((1 << ICM20948_REG_ACCEL_INTEL_MODE_INT_POS) | (1 << ICM20948_REG_ACCEL_INTEL_EN_POS))
#define ICM20948_REG_ACCEL_INTEL_CTRL_DIS     (0x0)

/* REG_WRITE_WOM_THR */
#define ICM20948_REG_WOM_THR_DEFAULT (ICM20948_MOTION_THRSHOLD_DEFAULT / 0.004)

/* ICM20948_REG_ACCEL_CONFIG */
#define ICM20948_REG_ACCEL_CONFIG_FCHOICE_POS (0)
#define ICM20948_REG_ACCEL_CONFIG_FS_SEL_POS  (1)
#define ICM20948_REG_ACCEL_CONFIG_DLPFCFG_POS (3)
typedef enum icm20948_reg_accel_config_fs_sel_
{
    ICM20948_REG_ACCEL_CONFIG_FS_SEL_2G  = 0,
    ICM20948_REG_ACCEL_CONFIG_FS_SEL_4G  = 1,
    ICM20948_REG_ACCEL_CONFIG_FS_SEL_8G  = 2,
    ICM20948_REG_ACCEL_CONFIG_FS_SEL_16G = 3,
} icm20948_reg_accel_config_fs_sel_t;
#define ICM20948_REG_ACCEL_CONFIG_DEFAULT                                                                                              \
    ((0x1 << ICM20948_REG_ACCEL_CONFIG_FCHOICE_POS) | (ICM20948_REG_ACCEL_CONFIG_FS_SEL_16G << ICM20948_REG_ACCEL_CONFIG_FS_SEL_POS) | \
     (0x2 << ICM20948_REG_ACCEL_CONFIG_DLPFCFG_POS))

/* ICM20948_REG_ACCEL_SMPLRT_DIV */
#define ICM20948_REG_ACCEL_SMPLRT_DIV_DEFAULT ((1125 + ICM20948_SAMPLE_RATE_DEFAULT - 1) / ICM20948_SAMPLE_RATE_DEFAULT - 1)

/* ICM20948_REG_INT_PIN_CFG */
#define ICM20948_REG_INT_PIN_CFG_INT1_ACTL_POS         (7)
#define ICM20948_REG_INT_PIN_CFG_INT1_OPEN_POS         (6)
#define ICM20948_REG_INT_PIN_CFG_INT1_LATCH_EN_POS     (5)
#define ICM20948_REG_INT_PIN_CFG_INT_ANYRD_2CLEAR_POS  (4)
#define ICM20948_REG_INT_PIN_CFG_ACTL_FSYNC_POS        (3)
#define ICM20948_REG_INT_PIN_CFG_FSYNC_INT_MODE_EN_POS (2)
#define ICM20948_REG_INT_PIN_CFG_BYPASS_EN_POS         (1)

/* ICM20948_REG_I2C_MST_CTRL */
#define ICM20948_REG_I2C_MST_CTRL_I2C_MST_CLK_POS   (0)
#define ICM20948_REG_I2C_MST_CTRL_I2C_MST_P_NSR_POS (4)
#define ICM20948_REG_I2C_MST_CTRL_MULT_MST_EN_POS   (7)
#define ICM20948_REG_I2C_MST_CTRL_DEFAULT           ((1 << ICM20948_REG_I2C_MST_CTRL_I2C_MST_P_NSR_POS) | (7 << ICM20948_REG_I2C_MST_CTRL_I2C_MST_CLK_POS))

/* ICM20948_REG_USER_CTRL */
#define ICM20948_REG_USER_CTRL_DMP_EN_POS      (7)
#define ICM20948_REG_USER_CTRL_FIFO_EN_POS     (6)
#define ICM20948_REG_USER_CTRL_I2C_MST_EN_POS  (5)
#define ICM20948_REG_USER_CTRL_I2C_IF_DIS_POS  (4)
#define ICM20948_REG_USER_CTRL_DMP_RST_POS     (3)
#define ICM20948_REG_USER_CTRL_SRAM_RST_POS    (2)
#define ICM20948_REG_USER_CTRL_I2C_MST_RST_POS (1)
#define ICM20948_REG_USER_CTRL_DEFAULT         (1 << ICM20948_REG_USER_CTRL_I2C_MST_EN_POS) // (138) //  (10)

/* ICM20948_REG_I2C_SLV4_ADDR 8C */
#define ICM20948_REG_I2C_SLV4_ADDR_RNW_POS       (7)
#define ICM20948_REG_I2C_SLV4_ADDR_ID_POS        (0)
#define ICM20948_REG_I2C_SLV4_ADDR_WRITE_AK09916 (AK09916_DEV_ADDR << ICM20948_REG_I2C_SLV4_ADDR_ID_POS)
#define ICM20948_REG_I2C_SLV4_ADDR_READ_AK09916  ((1 << ICM20948_REG_I2C_SLV4_ADDR_RNW_POS) | (AK09916_DEV_ADDR << ICM20948_REG_I2C_SLV4_ADDR_ID_POS))

//    ICM20948_REG_I2C_SLV4_CTRL 80
#define ICM20948_REG_I2C_SLV4_CTRL_I2C_SLV4_EN_POS      (7)
#define ICM20948_REG_I2C_SLV4_CTRL_I2C_SLV4_INT_EN_POS  (6)
#define ICM20948_REG_I2C_SLV4_CTRL_I2C_SLV4_REG_DIS_POS (5)
#define ICM20948_REG_I2C_SLV4_CTRL_I2C_SLV4_DLY_POS     (0)
#define ICM20948_REG_I2C_SLV4_CTRL_START                (1 << ICM20948_REG_I2C_SLV4_CTRL_I2C_SLV4_EN_POS)

/* ICM20948_REG_I2C_MST_STATUS  40 */
#define ICM20948_REG_I2C_MST_STATUS_SLV4_DONE_POS (6)
#define ICM20948_REG_I2C_MST_STATUS_SLV4_DONE_MSK (0x1 << ICM20948_REG_I2C_MST_STATUS_SLV4_DONE_POS)

/* ICM20948_REG_I2C_SLV0_ADDR 8C */
#define ICM20948_REG_I2C_SLV0_ADDR_RNW_POS       (7)
#define ICM20948_REG_I2C_SLV0_ADDR_ID_POS        (0)
#define ICM20948_REG_I2C_SLV0_ADDR_WRITE_AK09916 (AK09916_DEV_ADDR << ICM20948_REG_I2C_SLV0_ADDR_ID_POS)
#define ICM20948_REG_I2C_SLV0_ADDR_READ_AK09916  ((1 << ICM20948_REG_I2C_SLV0_ADDR_RNW_POS) | (AK09916_DEV_ADDR << ICM20948_REG_I2C_SLV0_ADDR_ID_POS))

/* ICM20948_REG_I2C_SLV0_CTRL 89 */
#define ICM20948_REG_I2C_SLV0_CTRL_EN_MSK      (0x80)
#define ICM20948_REG_I2C_SLV0_CTRL_LENG_POS    (0)
#define ICM20948_REG_I2C_SLV0_CTRL_START_LEN_9 ((ICM20948_REG_I2C_SLV0_CTRL_EN_MSK) | (9 << ICM20948_REG_I2C_SLV0_CTRL_LENG_POS))

/* ICM20948_REG_INT_STATUS_1 01 */
#define ICM20948_REG_INT_STATUS_1_ATA_0_RDY_INT_MSK (0x1)

/* AK09916_REG_WIA1 0x48 */
#define AK09916_REG_WIA1_DEFAULT (0x48)

/* AK09916_REG_WIA2 0x09 */
#define AK09916_REG_WIA2_DEFAULT (0x09)

/* AK09916_REG_CNTL2 0x80 */
typedef enum ak09916_reg_cntl2_
{
    AK09916_REG_CNTL2_POWER_DOWN = 0x00,
    AK09916_REG_CNTL2_SINGLE     = 0x01,
    AK09916_REG_CNTL2_MODE1      = 0x02,
    AK09916_REG_CNTL2_MODE2      = 0x04,
    AK09916_REG_CNTL2_MODE3      = 0x06,
    AK09916_REG_CNTL2_MODE4      = 0x08,
    AK09916_REG_CNTL2_SELF_TEST  = 0x10,
} ak09916_reg_cntl2_t;
#define AK09916_REG_CNTL2_DEFAULT (AK09916_REG_CNTL2_MODE4)

/* function define */
static bool icm20948_init_process(h_icm20948_t *p_h_icm);
static bool icm20948_motion_det_set_process(h_icm20948_t *p_h_icm);
bool        icm20948_low_power_set_process(h_icm20948_t *p_h_icm);
static bool icm20948_setting_process(h_icm20948_t *p_h_icm);
static bool icm20948_sample_process(h_icm20948_t *p_h_icm);

static bool icm20948_setting_set(h_icm20948_t *p_h_icm, h_icm20948_setting_t *p_h_setting);

/* function */
// __weak void icm20948_init_error_cb(void)
// {
// }

// __weak void icm20948_sample_finish_cb(void)
// {
// }

// __weak bool icm20948_i2c_sent_cb(h_icm20948_i2c_t *p_h_i2c)
// {
//     return false;
// }

// __weak bool icm20948_spi_sent_cb(h_icm20948_spi_t *p_h_spi)
// {
//     return false;
// }

// __weak uint32_t icm20948_systick_get_cb(void)
// {
//     return 0;
// }
// __weak void icm20948_pin_set_cb(icm20948_pin_t pin, bool b_status)
// {
// }

/* reutrn value: true = OK, false = retry. */
bool icm20948_sent_finish(h_icm20948_t *p_h_icm, icm20948_res_t res)
{
    p_h_icm->sent_res = res;
    if (p_h_icm->sent_res != ICM20948_RES_OK)
    {
        p_h_icm->sent_error_count++;
        if (p_h_icm->sent_error_count < ICM20948_ERROR_RETRY_N_DEFAULT)
            return false;
    }
    p_h_icm->b_sent_busy = false;
    return true;
}

// void icm20948_sample_start(void)
// {
//     p_h_icm->b_sample_start = true;
// }

// __weak bool icm20948_pin_get_cb(icm20948_pin_t pin)
// {
//     return true;
// }

/* return value: true = busy, false = idle. */
bool icm20948_process(h_icm20948_t *p_h_icm)
{
    bool b_busy = icm20948_init_process(p_h_icm);
    b_busy |= icm20948_motion_det_set_process(p_h_icm);
    b_busy |= icm20948_low_power_set_process(p_h_icm);
    b_busy |= icm20948_setting_process(p_h_icm);
    b_busy |= icm20948_sample_process(p_h_icm);
    return b_busy;
}

static bool icm20948_init_process(h_icm20948_t *p_h_icm)
{
    if (p_h_icm == NULL)
        return false;

    typedef enum
    {
        STEP_IDLE,
        // STEP_GPIO_INIT,
        // STEP_TIMER_INIT,
        STEP_SCAN_TEST,
        STEP_POWER_ON_START,
        STEP_POWER_ON_WAIT,
        STEP_REG_LIST_SET,
        STEP_REG_LIST_WAIT,
        STEP_FINISH,
    } step_t;

    typedef enum reg_write_
    {
        REG_WRITE_BANK_SEL_0,
        REG_WRITE_BANK_SEL_1,
        REG_WRITE_BANK_SEL_2,
        REG_WRITE_BANK_SEL_3,
        REG_WRITE_PWR_MGMT_1_RST,
        REG_WRITE_PWR_MGMT_1_DEFAULT,
        REG_WRITE_LP_CONFIG,
        REG_WRITE_GYRO_CONFIG_1,
        REG_WRITE_GYRO_SMPLRT_DIV,
        REG_WRITE_ACCEL_CONFIG,
        REG_WRITE_ACCEL_SMPLRT_DIV_2,
        REG_WRITE_WOM_THR,
        REG_WRITE_INT_PIN_CFG,
        REG_WRITE_I2C_MST_CTRL,
        REG_WRITE_USER_CTRL,
        REG_WRITE_I2C_SLV4_ADDR_READ_AK09916,
        REG_WRITE_I2C_SLV4_ADDR_WRITE_AK09916,
        REG_WRITE_I2C_SLV4_REG_WIA1,
        REG_WRITE_I2C_SLV4_REG_WIA2,
        REG_WRITE_I2C_SLV4_REG_CNTL2,
        REG_WRITE_I2C_SLV4_CTRL_START,
        REG_WRITE_I2C_SLV0_ADDR_READ_AK09916,
        REG_WRITE_I2C_SLV0_REG_ST1,
        REG_WRITE_I2C_SLV0_CTRL_START_LEN_9,
        REG_WRITE_CNTL2,
        REG_WRITE_NUM,
    } reg_write_t;

    typedef enum reg_check_
    {
        REG_CHECK_WHO_AM_I,
        REG_CHECK_PWR_MGMT_1_RST,
        REG_CHECK_I2C_MST_STATUS_SLV4_DONE,
        REG_CHECK_WIA1,
        REG_CHECK_WIA2,
        REG_CHECK_INT_STATUS_1,
        REG_CHECK_NUM,
    } reg_check_t;

    // static uint8_t reg_buff[1];
    static uint8_t reg_write[REG_WRITE_NUM] = {
        [REG_WRITE_BANK_SEL_0]                  = ICM20948_REG_REG_BANK_SEL_0,
        [REG_WRITE_BANK_SEL_1]                  = ICM20948_REG_REG_BANK_SEL_1,
        [REG_WRITE_BANK_SEL_2]                  = ICM20948_REG_REG_BANK_SEL_2,
        [REG_WRITE_BANK_SEL_3]                  = ICM20948_REG_REG_BANK_SEL_3,
        [REG_WRITE_PWR_MGMT_1_RST]              = ICM20948_REG_PWR_MGMT_1_DEVICE_RESET_MSK,
        [REG_WRITE_PWR_MGMT_1_DEFAULT]          = ICM20948_REG_PWR_MGMT_1_DEFAULT,
        [REG_WRITE_LP_CONFIG]                   = ICM20948_REG_LP_CONFIG_DEFAULT,
        [REG_WRITE_GYRO_CONFIG_1]               = ICM20948_REG_GYRO_CONFIG_1_DEFAULT,
        [REG_WRITE_GYRO_SMPLRT_DIV]             = ICM20948_REG_GYRO_SMPLRT_DIV_DEFAULT,
        [REG_WRITE_ACCEL_CONFIG]                = ICM20948_REG_ACCEL_CONFIG_DEFAULT,
        [REG_WRITE_ACCEL_SMPLRT_DIV_2]          = ICM20948_REG_ACCEL_SMPLRT_DIV_DEFAULT,
        [REG_WRITE_WOM_THR]                     = ICM20948_REG_WOM_THR_DEFAULT,
        [REG_WRITE_INT_PIN_CFG]                 = ICM20948_REG_INT_PIN_CFG_DEFAULT,
        [REG_WRITE_I2C_MST_CTRL]                = ICM20948_REG_I2C_MST_CTRL_DEFAULT,
        [REG_WRITE_USER_CTRL]                   = ICM20948_REG_USER_CTRL_DEFAULT,
        [REG_WRITE_I2C_SLV4_ADDR_READ_AK09916]  = ICM20948_REG_I2C_SLV4_ADDR_READ_AK09916,
        [REG_WRITE_I2C_SLV4_ADDR_WRITE_AK09916] = ICM20948_REG_I2C_SLV4_ADDR_WRITE_AK09916,
        [REG_WRITE_I2C_SLV4_REG_WIA1]           = AK09916_REG_WIA1,
        [REG_WRITE_I2C_SLV4_REG_WIA2]           = AK09916_REG_WIA2,
        [REG_WRITE_I2C_SLV4_REG_CNTL2]          = AK09916_REG_CNTL2,
        [REG_WRITE_I2C_SLV4_CTRL_START]         = ICM20948_REG_I2C_SLV4_CTRL_START,
        [REG_WRITE_CNTL2]                       = AK09916_REG_CNTL2_DEFAULT,
        [REG_WRITE_I2C_SLV0_ADDR_READ_AK09916]  = ICM20948_REG_I2C_SLV0_ADDR_READ_AK09916,
        [REG_WRITE_I2C_SLV0_REG_ST1]            = AK09916_REG_ST1,
        [REG_WRITE_I2C_SLV0_CTRL_START_LEN_9]   = ICM20948_REG_I2C_SLV0_CTRL_START_LEN_9,
    };
    static uint8_t reg_check[REG_CHECK_NUM] = {
        [REG_CHECK_WHO_AM_I]                 = ICM20948_REG_WHO_AM_I_DEFAULT,
        [REG_CHECK_PWR_MGMT_1_RST]           = ICM20948_REG_PWR_MGMT_1_DEVICE_RESET_MSK,
        [REG_CHECK_I2C_MST_STATUS_SLV4_DONE] = ICM20948_REG_I2C_MST_STATUS_SLV4_DONE_MSK,
        [REG_CHECK_WIA1]                     = AK09916_REG_WIA1_DEFAULT,
        [REG_CHECK_WIA2]                     = AK09916_REG_WIA2_DEFAULT,
        [REG_CHECK_INT_STATUS_1]             = ICM20948_REG_INT_STATUS_1_ATA_0_RDY_INT_MSK,
    };
    static h_icm20948_reg_list_t h_reg_list[] = {
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_REG_BANK_SEL,
            .p_reg_data   = &reg_write[REG_WRITE_BANK_SEL_0],
            .reg_size     = 1,
            .retry_n      = 5,
            .retry_delay  = 1,
            .delay        = 2,
        },
        {
            .b_read_write = true,
            .b_check_set  = true,
            .b_check_rst  = true,
            .reg_addr     = ICM20948_REG_WHO_AM_I,
            // .p_reg_data = reg_buff,
            .p_reg_check = &reg_check[REG_CHECK_WHO_AM_I],
            .reg_size    = 1,
            .retry_n     = 100,
            .retry_delay = 2,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_PWR_MGMT_1,
            .p_reg_data   = &reg_write[REG_WRITE_PWR_MGMT_1_RST],
            .reg_size     = 1,
            .delay        = 20,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_REG_BANK_SEL,
            .p_reg_data   = &reg_write[REG_WRITE_BANK_SEL_0],
            .reg_size     = 1,
            .delay        = 2,
        },
        {
            .b_read_write = true,
            .b_check_rst  = true,
            .reg_addr     = ICM20948_REG_PWR_MGMT_1,
            // .p_reg_data = reg_buff,
            .p_reg_check = &reg_check[REG_CHECK_PWR_MGMT_1_RST],
            .reg_size    = 1,
            .retry_n     = 5,
            .retry_delay = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_PWR_MGMT_1,
            .p_reg_data   = &reg_write[REG_WRITE_PWR_MGMT_1_DEFAULT],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_LP_CONFIG,
            .p_reg_data   = &reg_write[REG_WRITE_LP_CONFIG],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_INT_PIN_CFG,
            .p_reg_data   = &reg_write[REG_WRITE_INT_PIN_CFG],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_REG_BANK_SEL,
            .p_reg_data   = &reg_write[REG_WRITE_BANK_SEL_2],
            .reg_size     = 1,
            .delay        = 2,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_GYRO_CONFIG_1,
            .p_reg_data   = &reg_write[REG_WRITE_GYRO_CONFIG_1],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_GYRO_SMPLRT_DIV,
            .p_reg_data   = &reg_write[REG_WRITE_GYRO_SMPLRT_DIV],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_ACCEL_CONFIG,
            .p_reg_data   = &reg_write[REG_WRITE_ACCEL_CONFIG],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_ACCEL_SMPLRT_DIV_2,
            .p_reg_data   = &reg_write[REG_WRITE_ACCEL_SMPLRT_DIV_2],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_ACCEL_WOM_THR,
            .p_reg_data   = (uint8_t *)&reg_write[REG_WRITE_WOM_THR],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_REG_BANK_SEL,
            .p_reg_data   = &reg_write[REG_WRITE_BANK_SEL_3],
            .reg_size     = 1,
            .delay        = 2,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_I2C_MST_CTRL,
            .p_reg_data   = &reg_write[REG_WRITE_I2C_MST_CTRL],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_REG_BANK_SEL,
            .p_reg_data   = &reg_write[REG_WRITE_BANK_SEL_0],
            .reg_size     = 1,
            .delay        = 2,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_USER_CTRL,
            .p_reg_data   = &reg_write[REG_WRITE_USER_CTRL],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_REG_BANK_SEL,
            .p_reg_data   = &reg_write[REG_WRITE_BANK_SEL_3],
            .reg_size     = 1,
            .delay        = 2,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_I2C_SLV4_ADDR,
            .p_reg_data   = &reg_write[REG_WRITE_I2C_SLV4_ADDR_READ_AK09916],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_I2C_SLV4_REG,
            .p_reg_data   = &reg_write[REG_WRITE_I2C_SLV4_REG_WIA1],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_I2C_SLV4_CTRL,
            .p_reg_data   = &reg_write[REG_WRITE_I2C_SLV4_CTRL_START],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_REG_BANK_SEL,
            .p_reg_data   = &reg_write[REG_WRITE_BANK_SEL_0],
            .reg_size     = 1,
            .delay        = 2,
        },
        {
            .b_read_write = true,
            .b_check_set  = true,
            .reg_addr     = ICM20948_REG_I2C_MST_STATUS,
            // .p_reg_data = reg_buff,
            .p_reg_check      = &reg_check[REG_CHECK_I2C_MST_STATUS_SLV4_DONE],
            .reg_size         = 1,
            .retry_n          = 5,
            .retry_delay      = 1,
            .error_jump_en    = true,
            .error_jump_shift = -5,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_REG_BANK_SEL,
            .p_reg_data   = &reg_write[REG_WRITE_BANK_SEL_3],
            .reg_size     = 1,
            .delay        = 2,
        },
        {
            .b_read_write = true,
            .b_check_set  = true,
            .b_check_rst  = true,
            .reg_addr     = ICM20948_REG_I2C_SLV4_DI,
            // .p_reg_data = reg_buff,
            .p_reg_check      = &reg_check[REG_CHECK_WIA1],
            .reg_size         = 1,
            .error_jump_en    = true,
            .error_jump_shift = -7,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_I2C_SLV4_ADDR,
            .p_reg_data   = &reg_write[REG_WRITE_I2C_SLV4_ADDR_READ_AK09916],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_I2C_SLV4_REG,
            .p_reg_data   = &reg_write[REG_WRITE_I2C_SLV4_REG_WIA2],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_I2C_SLV4_CTRL,
            .p_reg_data   = &reg_write[REG_WRITE_I2C_SLV4_CTRL_START],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_REG_BANK_SEL,
            .p_reg_data   = &reg_write[REG_WRITE_BANK_SEL_0],
            .reg_size     = 1,
            .delay        = 2,
        },
        {
            .b_read_write = true,
            .b_check_set  = true,
            .reg_addr     = ICM20948_REG_I2C_MST_STATUS,
            // .p_reg_data = reg_buff,
            .p_reg_check = &reg_check[REG_CHECK_I2C_MST_STATUS_SLV4_DONE],
            .reg_size    = 1,
            .retry_n     = 5,
            .retry_delay = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_REG_BANK_SEL,
            .p_reg_data   = &reg_write[REG_WRITE_BANK_SEL_3],
            .reg_size     = 1,
            .delay        = 2,
        },
        {
            .b_read_write = true,
            .b_check_set  = true,
            .b_check_rst  = true,
            .reg_addr     = ICM20948_REG_I2C_SLV4_DI,
            // .p_reg_data = reg_buff,
            .p_reg_check = &reg_check[REG_CHECK_WIA2],
            .reg_size    = 1,
            .retry_n     = 5,
            .retry_delay = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_I2C_SLV4_ADDR,
            .p_reg_data   = &reg_write[REG_WRITE_I2C_SLV4_ADDR_WRITE_AK09916],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_I2C_SLV4_REG,
            .p_reg_data   = &reg_write[REG_WRITE_I2C_SLV4_REG_CNTL2],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_I2C_SLV4_DO,
            .p_reg_data   = &reg_write[REG_WRITE_CNTL2],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_I2C_SLV4_CTRL,
            .p_reg_data   = &reg_write[REG_WRITE_I2C_SLV4_CTRL_START],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_REG_BANK_SEL,
            .p_reg_data   = &reg_write[REG_WRITE_BANK_SEL_0],
            .reg_size     = 1,
            .delay        = 2,
        },
        {
            .b_read_write = true,
            .b_check_set  = true,
            .reg_addr     = ICM20948_REG_I2C_MST_STATUS,
            // .p_reg_data   = reg_buff,
            .p_reg_check = &reg_check[REG_CHECK_I2C_MST_STATUS_SLV4_DONE],
            .reg_size    = 1,
            .retry_n     = 5,
            .retry_delay = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_REG_BANK_SEL,
            .p_reg_data   = &reg_write[REG_WRITE_BANK_SEL_3],
            .reg_size     = 1,
            .delay        = 5,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_I2C_SLV0_ADDR,
            .p_reg_data   = &reg_write[REG_WRITE_I2C_SLV0_ADDR_READ_AK09916],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_I2C_SLV0_REG,
            .p_reg_data   = &reg_write[REG_WRITE_I2C_SLV0_REG_ST1],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_I2C_SLV0_CTRL,
            .p_reg_data   = &reg_write[REG_WRITE_I2C_SLV0_CTRL_START_LEN_9],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_REG_BANK_SEL,
            .p_reg_data   = &reg_write[REG_WRITE_BANK_SEL_0],
            .reg_size     = 1,
            .delay        = 5,
        },
        {
            .b_read_write = true,
            .b_check_set  = true,
            .reg_addr     = ICM20948_REG_INT_STATUS_1,
            // .p_reg_data = reg_buff,
            .p_reg_check = &reg_check[REG_CHECK_INT_STATUS_1],
            .reg_size    = 1,
            .retry_n     = 5,
            .retry_delay = 1,
        },
    };

    const size_t h_reg_list_len = sizeof(h_reg_list) / sizeof(h_icm20948_reg_list_t);

    h_icm20948_init_p_t *p_h_process = &p_h_icm->h_init_p;
    bool                 b_printf    = p_h_icm->h_debug.b_init;

    // static uint8_t              step = STEP_IDLE;
    // static uint32_t             time_log;
    // static h_icm20948_setting_t h_setting;

    switch (p_h_process->step)
    {
    case STEP_IDLE:
        if (p_h_icm->b_init_finish)
            break;
        if (b_printf)
        {
            printf("> icm20948[%d]_init:ver", p_h_icm->id);
            printf(",%X", ICM20948_VER_MAIN);
            printf(",%02X", ICM20948_VER_SUB);
            printf("\r\n");
        }
    //     printf("> icm20948[%d]_init:gpio\r\n", p_h_icm->id);
    //     step = STEP_GPIO_INIT;
    // case STEP_GPIO_INIT:
    //     icm20948_gpio_init();
    //     if (b_printf)
    //     {
    //         printf("> icm20948[%d]_init:timer\r\n", p_h_icm->id);
    //     }
    //     step = STEP_TIMER_INIT;
    //     break;
    // case STEP_TIMER_INIT:
    //     icm20948_timer_init();
    case STEP_POWER_ON_START:
    {
        if (b_printf)
        {
            printf("> icm20948[%d]_init:power_on\r\n", p_h_icm->id);
        }
        if (p_h_icm->pin_set_cb != NULL)
            p_h_icm->pin_set_cb(p_h_icm->id, ICM20948_PIN_PWR, true);

        if (p_h_icm->systick_get_cb != NULL)
            p_h_process->time_log = p_h_icm->systick_get_cb();
        p_h_process->step = STEP_POWER_ON_WAIT;
    }
    case STEP_POWER_ON_WAIT:
    {
        if (p_h_icm->systick_get_cb != NULL)
        {
            uint32_t time_now = p_h_icm->systick_get_cb();
            if (time_now - p_h_process->time_log < ICM20948_POWER_ON_DELAY)
                break;
        }

        h_icm20948_setting_t *p_h_setting = &p_h_process->h_setting;
        p_h_setting->p_h_list             = h_reg_list;
        p_h_setting->list_len             = h_reg_list_len;
        if (b_printf)
        {
            printf("> icm20948[%d]_init:setting", p_h_icm->id);
            printf(",%d", p_h_setting->list_len);
            printf("\r\n");
        }
        p_h_process->step = STEP_REG_LIST_SET;
    }
    case STEP_REG_LIST_SET:
        if (icm20948_setting_set(p_h_icm, &p_h_process->h_setting) == false)
            break;

        p_h_process->step = STEP_REG_LIST_WAIT;
        break;
    case STEP_REG_LIST_WAIT:
        if (p_h_process->h_setting.b_busy)
            break;
        p_h_process->step = STEP_FINISH;
    case STEP_FINISH:
    {
        h_icm20948_setting_t *p_h_setting = &p_h_process->h_setting;
        p_h_icm->b_init_finish            = p_h_setting->b_error == false;
        if (b_printf)
        {
            printf("> icm20948[%d]_init:", p_h_icm->id);
            if (p_h_setting->b_error)
            {
                printf("fail,%d", p_h_setting->list_n_error);
            }
            else
            {
                printf("finish");
            }
            printf("\r\n");
        }
        p_h_process->step = STEP_IDLE;
        break;
    }
    default:
        p_h_process->step = STEP_IDLE;
    }
    return p_h_process->step != STEP_IDLE;
}

static bool icm20948_motion_det_set_process(h_icm20948_t *p_h_icm)
{
    typedef enum
    {
        STEP_IDLE,
        STEP_TIMER_INIT,
        STEP_SCAN_TEST,
        STEP_POWER_ON_WAIT,
        STEP_REG_LIST_SET,
        STEP_REG_LIST_WAIT,
        STEP_FINISH,
    } step_t;

    typedef enum reg_write_
    {
        REG_WRITE_BANK_SEL_0,
        REG_WRITE_BANK_SEL_2,
        REG_WRITE_ACCEL_INTEL_EN,
        REG_WRITE_ACCEL_INTEL_DIS,
        REG_WRITE_INT_ENABLE_WOM_EN,
        REG_WRITE_INT_ENABLE_WOM_DIS,
        REG_WRITE_NUM,
    } reg_write_t;

    static const uint8_t reg_write[REG_WRITE_NUM] = {
        [REG_WRITE_BANK_SEL_0] = ICM20948_REG_REG_BANK_SEL_0,           [REG_WRITE_BANK_SEL_2] = ICM20948_REG_REG_BANK_SEL_2,
        [REG_WRITE_ACCEL_INTEL_EN] = ICM20948_REG_ACCEL_INTEL_CTRL_EN,  [REG_WRITE_ACCEL_INTEL_DIS] = ICM20948_REG_ACCEL_INTEL_CTRL_DIS,
        [REG_WRITE_INT_ENABLE_WOM_EN] = ICM20948_REG_INT_ENABLE_WOM_EN, [REG_WRITE_INT_ENABLE_WOM_DIS] = ICM20948_REG_INT_ENABLE_WOM_DIS,
    };
    static const h_icm20948_reg_list_t h_reg_list_en[] = {
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_REG_BANK_SEL,
            .p_reg_data   = (uint8_t *)&reg_write[REG_WRITE_BANK_SEL_2],
            .reg_size     = 1,
            .delay        = 2,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_ACCEL_INTEL_CTRL,
            .p_reg_data   = (uint8_t *)&reg_write[REG_WRITE_ACCEL_INTEL_EN],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_REG_BANK_SEL,
            .p_reg_data   = (uint8_t *)&reg_write[REG_WRITE_BANK_SEL_0],
            .reg_size     = 1,
            .delay        = 5,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_INT_ENABLE,
            .p_reg_data   = (uint8_t *)&reg_write[REG_WRITE_INT_ENABLE_WOM_EN],
            .reg_size     = 1,
        },
    };
    static const size_t                h_reg_list_en_len = sizeof(h_reg_list_en) / sizeof(h_icm20948_reg_list_t);
    static const h_icm20948_reg_list_t h_reg_list_dis[]  = {
        {
             .b_read_write = false,
             .reg_addr     = ICM20948_REG_REG_BANK_SEL,
             .p_reg_data   = (uint8_t *)&reg_write[REG_WRITE_BANK_SEL_2],
             .reg_size     = 1,
             .delay        = 2,
        },
        {
             .b_read_write = false,
             .reg_addr     = ICM20948_REG_ACCEL_INTEL_CTRL,
             .p_reg_data   = (uint8_t *)&reg_write[REG_WRITE_ACCEL_INTEL_DIS],
             .reg_size     = 1,
             .delay        = 2,
        },
        {
             .b_read_write = false,
             .reg_addr     = ICM20948_REG_REG_BANK_SEL,
             .p_reg_data   = (uint8_t *)&reg_write[REG_WRITE_BANK_SEL_0],
             .reg_size     = 1,
             .delay        = 5,
        },
        {
             .b_read_write = false,
             .reg_addr     = ICM20948_REG_INT_ENABLE,
             .p_reg_data   = (uint8_t *)&reg_write[REG_WRITE_INT_ENABLE_WOM_DIS],
             .reg_size     = 1,
             .delay        = 2,
        },
    };
    static const size_t h_reg_list_dis_len = sizeof(h_reg_list_dis) / sizeof(h_icm20948_reg_list_t);

    h_icm20948_motion_det_p_t *p_h_process = &p_h_icm->h_motion_det_p;
    bool                       b_printf    = p_h_icm->h_debug.b_motion_det;

    // static step_t               step                   = STEP_IDLE;
    // static bool                 b_motion_en_target_log = false;
    // static h_icm20948_setting_t h_setting;

    switch (p_h_process->step)
    {
    case STEP_IDLE:
    {
        if (p_h_icm->b_init_finish == false)
            break;
        if (p_h_icm->b_motion_det_en_current == p_h_icm->b_motion_det_en)
            break;
        p_h_process->b_motion_det_en_log = p_h_icm->b_motion_det_en;

        h_icm20948_setting_t *p_h_setting = &p_h_process->h_setting;
        if (p_h_process->b_motion_det_en_log)
        {
            p_h_setting->p_h_list = h_reg_list_en;
            p_h_setting->list_len = h_reg_list_en_len;
        }
        else
        {
            p_h_setting->p_h_list = h_reg_list_dis;
            p_h_setting->list_len = h_reg_list_dis_len;
        }

        if (b_printf)
        {
            printf("> icm20948[%d]_motion:start", p_h_icm->id);
            printf(",%d", p_h_process->b_motion_det_en_log);
            printf("\r\n");
        }
        p_h_process->step = STEP_REG_LIST_SET;
    }
    case STEP_REG_LIST_SET:
        if (icm20948_setting_set(p_h_icm, &p_h_process->h_setting) == false)
            break;

        p_h_process->step = STEP_REG_LIST_WAIT;
        break;
    case STEP_REG_LIST_WAIT:
        if (p_h_process->h_setting.b_busy)
            break;
        p_h_process->step = STEP_FINISH;
    case STEP_FINISH:
    {
        h_icm20948_setting_t *p_h_setting = &p_h_process->h_setting;
        if (b_printf)
        {
            printf("> icm20948[%d]_motion:finish", p_h_icm->id);
            printf(",%d", p_h_setting->b_error);
            if (p_h_setting->b_error)
                printf(",%d", p_h_setting->list_n_error);
            printf("\r\n");
        }
        if (p_h_setting->b_error && p_h_icm->init_error_cb != NULL)
        {
            p_h_icm->init_error_cb(p_h_icm->id);
        }
        p_h_icm->b_motion_det_en_current = p_h_process->b_motion_det_en_log;
        p_h_process->step                = STEP_IDLE;
        break;
    }
    default:
        p_h_process->step = STEP_IDLE;
    }
    return p_h_process->step != STEP_IDLE;
}

bool icm20948_low_power_set_process(h_icm20948_t *p_h_icm)
{
    typedef enum
    {
        STEP_IDLE,
        STEP_TIMER_INIT,
        STEP_SCAN_TEST,
        STEP_POWER_ON_WAIT,
        STEP_REG_LIST_SET,
        STEP_REG_LIST_WAIT,
        STEP_FINISH,
    } step_t;

    typedef enum reg_write_
    {
        REG_WRITE_BANK_SEL_0,
        REG_WRITE_BANK_SEL_3,
        REG_WRITE_I2C_SLV4_ADDR_WRITE_AK09916,
        REG_WRITE_I2C_SLV4_REG_CNTL2,
        REG_WRITE_CNTL2_DEFAULT,
        REG_WRITE_CNTL2_POWER_DOWN,
        REG_WRITE_I2C_SLV4_CTRL_START,
        REG_WRITE_LP_CONFIG_DEFAULT,
        REG_WRITE_LP_CONFIG_A_CYCLE,
        REG_WRITE_PWR_MGMT_1_LOW_POWER,
        REG_WRITE_PWR_MGMT_1_DEFAULT,
        REG_WRITE_PWR_MGMT_2_EN,
        REG_WRITE_PWR_MGMT_2_DIS_GYRO,
        REG_WRITE_INT_ENABLE_WOM_EN,
        REG_WRITE_INT_ENABLE_WOM_DIS,
        REG_WRITE_NUM,
    } reg_write_t;

    static const uint8_t reg_write[REG_WRITE_NUM] = {
        [REG_WRITE_BANK_SEL_0]                  = ICM20948_REG_REG_BANK_SEL_0,
        [REG_WRITE_BANK_SEL_3]                  = ICM20948_REG_REG_BANK_SEL_3,
        [REG_WRITE_I2C_SLV4_ADDR_WRITE_AK09916] = ICM20948_REG_I2C_SLV4_ADDR_WRITE_AK09916,
        [REG_WRITE_I2C_SLV4_REG_CNTL2]          = AK09916_REG_CNTL2,
        [REG_WRITE_CNTL2_POWER_DOWN]            = AK09916_REG_CNTL2_POWER_DOWN,
        [REG_WRITE_CNTL2_DEFAULT]               = AK09916_REG_CNTL2_DEFAULT,
        [REG_WRITE_I2C_SLV4_CTRL_START]         = ICM20948_REG_I2C_SLV4_CTRL_START,
        [REG_WRITE_PWR_MGMT_2_EN]               = ICM20948_REG_PWR_MGMT_2_EN,
        [REG_WRITE_PWR_MGMT_2_DIS_GYRO]         = ICM20948_REG_PWR_MGMT_2_DIS_GYRO,
        [REG_WRITE_PWR_MGMT_1_LOW_POWER]        = ICM20948_REG_PWR_MGMT_1_LOW_POWER,
        [REG_WRITE_PWR_MGMT_1_DEFAULT]          = ICM20948_REG_PWR_MGMT_1_DEFAULT,
        [REG_WRITE_LP_CONFIG_DEFAULT]           = ICM20948_REG_LP_CONFIG_DEFAULT,
        [REG_WRITE_LP_CONFIG_A_CYCLE]           = ICM20948_REG_LP_CONFIG_ACCEL_CYCLE,
    };

    static const h_icm20948_reg_list_t h_reg_list_en[] = {
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_REG_BANK_SEL,
            .p_reg_data   = (void *)&reg_write[REG_WRITE_BANK_SEL_3],
            .reg_size     = 1,
            .delay        = 2,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_I2C_SLV4_ADDR,
            .p_reg_data   = (void *)&reg_write[REG_WRITE_I2C_SLV4_ADDR_WRITE_AK09916],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_I2C_SLV4_REG,
            .p_reg_data   = (void *)&reg_write[REG_WRITE_I2C_SLV4_REG_CNTL2],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_I2C_SLV4_DO,
            .p_reg_data   = (void *)&reg_write[REG_WRITE_CNTL2_POWER_DOWN],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_I2C_SLV4_CTRL,
            .p_reg_data   = (void *)&reg_write[REG_WRITE_I2C_SLV4_CTRL_START],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_REG_BANK_SEL,
            .p_reg_data   = (uint8_t *)&reg_write[REG_WRITE_BANK_SEL_0],
            .reg_size     = 1,
            .delay        = 5,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_LP_CONFIG,
            .p_reg_data   = (uint8_t *)&reg_write[REG_WRITE_LP_CONFIG_DEFAULT],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_PWR_MGMT_2,
            .p_reg_data   = (uint8_t *)&reg_write[REG_WRITE_PWR_MGMT_2_EN],
            .reg_size     = 1,
        },
        {
            .b_read_write = false,
            .reg_addr     = ICM20948_REG_PWR_MGMT_1,
            .p_reg_data   = (uint8_t *)&reg_write[REG_WRITE_PWR_MGMT_1_LOW_POWER],
            .reg_size     = 1,
        },
    };
    static const size_t                h_reg_list_en_len = sizeof(h_reg_list_en) / sizeof(h_icm20948_reg_list_t);
    static const h_icm20948_reg_list_t h_reg_list_dis[]  = {
        {
             .b_read_write = false,
             .reg_addr     = ICM20948_REG_REG_BANK_SEL,
             .p_reg_data   = (uint8_t *)&reg_write[REG_WRITE_BANK_SEL_0],
             .reg_size     = 1,
             .delay        = 5,
        },
        {
             .b_read_write = false,
             .reg_addr     = ICM20948_REG_PWR_MGMT_1,
             .p_reg_data   = (uint8_t *)&reg_write[REG_WRITE_PWR_MGMT_1_DEFAULT],
             .reg_size     = 1,
        },
        {
             .b_read_write = false,
             .reg_addr     = ICM20948_REG_LP_CONFIG,
             .p_reg_data   = (uint8_t *)&reg_write[REG_WRITE_LP_CONFIG_DEFAULT],
             .reg_size     = 1,
        },
        {
             .b_read_write = false,
             .reg_addr     = ICM20948_REG_PWR_MGMT_2,
             .p_reg_data   = (uint8_t *)&reg_write[REG_WRITE_PWR_MGMT_2_EN],
             .reg_size     = 1,
        },
        {
             .b_read_write = false,
             .reg_addr     = ICM20948_REG_REG_BANK_SEL,
             .p_reg_data   = (void *)&reg_write[REG_WRITE_BANK_SEL_3],
             .reg_size     = 1,
             .delay        = 2,
        },
        {
             .b_read_write = false,
             .reg_addr     = ICM20948_REG_I2C_SLV4_ADDR,
             .p_reg_data   = (void *)&reg_write[REG_WRITE_I2C_SLV4_ADDR_WRITE_AK09916],
             .reg_size     = 1,
        },
        {
             .b_read_write = false,
             .reg_addr     = ICM20948_REG_I2C_SLV4_REG,
             .p_reg_data   = (void *)&reg_write[REG_WRITE_I2C_SLV4_REG_CNTL2],
             .reg_size     = 1,
        },
        {
             .b_read_write = false,
             .reg_addr     = ICM20948_REG_I2C_SLV4_DO,
             .p_reg_data   = (void *)&reg_write[REG_WRITE_CNTL2_DEFAULT],
             .reg_size     = 1,
        },
        {
             .b_read_write = false,
             .reg_addr     = ICM20948_REG_I2C_SLV4_CTRL,
             .p_reg_data   = (void *)&reg_write[REG_WRITE_I2C_SLV4_CTRL_START],
             .reg_size     = 1,
        },
        {
             .b_read_write = false,
             .reg_addr     = ICM20948_REG_REG_BANK_SEL,
             .p_reg_data   = (uint8_t *)&reg_write[REG_WRITE_BANK_SEL_0],
             .reg_size     = 1,
             .delay        = 5,
        },
    };
    static const size_t h_reg_list_dis_len = sizeof(h_reg_list_dis) / sizeof(h_icm20948_reg_list_t);

    // static bool                 b_low_power_target_log = false;
    // static h_icm20948_setting_t h_setting;

    h_icm20948_low_power_p_t *p_h_process = &p_h_icm->h_low_power_p;
    bool                      b_printf    = p_h_icm->h_debug.b_low_power;

    switch (p_h_process->step)
    {
    case STEP_IDLE:
    {
        if (p_h_icm->b_init_finish == false)
            break;
        if (p_h_icm->b_low_power_current == p_h_icm->b_low_power)
            break;
        p_h_process->b_low_power_log = p_h_icm->b_low_power;

        h_icm20948_setting_t *p_h_setting = &p_h_process->h_setting;
        if (p_h_process->b_low_power_log)
        {
            p_h_setting->p_h_list = h_reg_list_en;
            p_h_setting->list_len = h_reg_list_en_len;
        }
        else
        {
            p_h_setting->p_h_list = h_reg_list_dis;
            p_h_setting->list_len = h_reg_list_dis_len;
        }

        if (b_printf)
        {
            printf("> icm20948[%d]_low_power:start", p_h_icm->id);
            printf(",%d", p_h_process->b_low_power_log);
            printf("\r\n");
        }
        p_h_process->step = STEP_REG_LIST_SET;
    }
    case STEP_REG_LIST_SET:
        if (icm20948_setting_set(p_h_icm, &p_h_process->h_setting) == false)
            break;

        p_h_process->step = STEP_REG_LIST_WAIT;
        break;
    case STEP_REG_LIST_WAIT:
        if (p_h_process->h_setting.b_busy)
            break;
        p_h_process->step = STEP_FINISH;
    case STEP_FINISH:
    {
        h_icm20948_setting_t *p_h_setting = &p_h_process->h_setting;
        if (b_printf)
        {
            printf("> icm20948[%d]_low_power:finish", p_h_icm->id);
            printf(",%d", p_h_setting->b_error);
            if (p_h_setting->b_error)
                printf(",%d", p_h_setting->list_n_error);
            printf("\r\n");
        }
        p_h_icm->b_low_power_current = p_h_icm->b_low_power;
        p_h_process->step            = STEP_IDLE;
        break;
    }
    default:
        p_h_process->step = STEP_IDLE;
    }
    return p_h_process->step != STEP_IDLE;
}

static bool icm20948_setting_process(h_icm20948_t *p_h_icm)
{
    typedef enum step_
    {
        STEP_IDLE,
        STEP_LIST_N_CHECK,
        STEP_LIST_GET,
        STEP_SENT_START,
        STEP_I2C_SET,
        STEP_SPI_START,
        STEP_SPI_SET,
        STEP_SENT_WAIT,
        STEP_DATA_CHECK,
        STEP_ERROR_CHECK,
        STEP_CHECK_DELAY,
        STEP_DELAY,
        STEP_FINISH,
    } step_t;

    // static uint8_t                step           = STEP_IDLE;
    // static h_icm20948_setting_t  *p_h_setting    = NULL;
    // static size_t                 reg_list_n     = 0;
    // static size_t                 h_reg_list_len = 0;
    // static size_t                 retry_count    = 0;
    // static size_t                 retry_n        = 0;
    // static h_icm20948_reg_list_t *p_h_list       = NULL;
    // static uint8_t                tx_buff[2]     = { 0x0 };
    // static uint8_t                rx_buff[2]     = { 0x0 };
    // //    static h_icm20948_reg_read_t h_reg_read;
    // //    static h_icm20948_reg_write_t h_reg_write;
    // static h_icm20948_i2c_t h_i2c_sent;
    // static h_icm20948_spi_t h_spi_sent;
    // static bool             b_error;
    // static uint32_t         time_log;
    // static uint32_t         time_delay;

    h_icm20948_setting_p_t *p_h_process = &p_h_icm->h_setting_p;
    bool                    b_printf    = p_h_icm->h_debug.b_setting;

    switch (p_h_process->step)
    {
    case STEP_FINISH:
    {
        p_h_icm->setting_q_front++;
        if (p_h_icm->setting_q_front >= ICM20948_SETTING_SEQ_Q_LEN)
            p_h_icm->setting_q_front = 0;

        h_icm20948_setting_t *p_h_setting = p_h_process->p_h_setting;
        p_h_setting->b_busy               = false;
        if (b_printf)
        {
            printf("> icm20948[%d]_setting:finish", p_h_icm->id);
            if (p_h_setting->b_error)
            {
                printf(",error");
                printf(",%d", p_h_process->reg_list_n);
            }
            printf("\r\n");
        }
        p_h_process->step = STEP_IDLE;
    }
    case STEP_IDLE:
    {
        if (p_h_icm->setting_q_front == p_h_icm->setting_q_back)
            break;

        h_icm20948_setting_t *p_h_setting;
        p_h_setting = p_h_icm->p_h_setting_q[p_h_icm->setting_q_front];
        if (p_h_setting == NULL)
        {
            p_h_icm->setting_q_front++;
            if (p_h_icm->setting_q_front >= ICM20948_SETTING_SEQ_Q_LEN)
                p_h_icm->setting_q_front = 0;
            break;
        }
        p_h_process->p_h_setting = p_h_setting;

        p_h_setting->b_busy         = true;
        p_h_process->h_reg_list_len = p_h_setting->list_len;
        p_h_process->reg_list_n     = 0;
        if (b_printf)
        {
            printf("> icm20948[%d]_setting:start", p_h_icm->id);
            printf(",%d", p_h_icm->setting_q_front);
            printf(",%d", p_h_process->h_reg_list_len);
            printf("\r\n");
        }
        p_h_process->step = STEP_LIST_N_CHECK;
    }
    case STEP_LIST_N_CHECK:
        if (p_h_process->reg_list_n >= p_h_process->h_reg_list_len)
        {
            p_h_process->p_h_setting->b_error = false;
            p_h_process->step                 = STEP_FINISH;
            break;
        }
        p_h_process->step = STEP_LIST_GET;
    case STEP_LIST_GET:
    {
        h_icm20948_setting_t  *p_h_setting = p_h_process->p_h_setting;
        h_icm20948_reg_list_t *p_h_list;
        p_h_list = (void *)&p_h_setting->p_h_list[p_h_process->reg_list_n];
        if (p_h_list == NULL)
        {
            p_h_setting->b_error      = true;
            p_h_setting->list_n_error = p_h_process->reg_list_n;
            p_h_process->step         = STEP_FINISH;
            break;
        }
        p_h_process->p_h_list = p_h_list;

        p_h_process->retry_count = 0;
        p_h_process->retry_n     = p_h_list->retry_n;
        if (p_h_list->b_read_write)
        {
            switch (p_h_icm->sent_sel)
            {
            case ICM20948_SENT_I2C:
            {
                h_icm20948_i2c_t *p_h_i2c_sent = &p_h_process->h_i2c_sent;
                p_h_i2c_sent->dev_addr         = ICM20948_DEV_ADDRESS;
                p_h_i2c_sent->p_tx_buff        = &p_h_list->reg_addr;
                p_h_i2c_sent->tx_size          = 1;
                p_h_i2c_sent->p_rx_buff        = &p_h_process->rx_buff[1];
                p_h_i2c_sent->rx_size          = 1;
                //                p_h_i2c_sent->rx_size = p_h_list->reg_size;
                break;
            }
            case ICM20948_SENT_SPI:
            {
                h_icm20948_spi_t *p_h_spi_sent = &p_h_process->h_spi_sent;
                p_h_process->tx_buff[0]        = p_h_list->reg_addr | ICM20948_SPI_REG_ADDR_READ_MSK;
                p_h_spi_sent->p_tx_buff        = p_h_process->tx_buff;
                p_h_spi_sent->p_rx_buff        = p_h_process->rx_buff;
                p_h_spi_sent->sent_size        = 2;
            }
            }
        }
        else
        {
            if (p_h_list->p_reg_data == NULL)
            {
                p_h_setting->b_error      = true;
                p_h_setting->list_n_error = p_h_process->reg_list_n;
                p_h_process->step         = STEP_FINISH;
                break;
            }
            p_h_process->tx_buff[0] = p_h_list->reg_addr;
            p_h_process->tx_buff[1] = *p_h_list->p_reg_data;
            switch (p_h_icm->sent_sel)
            {
            case ICM20948_SENT_I2C:
            {
                h_icm20948_i2c_t *p_h_i2c_sent = &p_h_process->h_i2c_sent;
                p_h_i2c_sent->dev_addr         = ICM20948_DEV_ADDRESS;
                p_h_i2c_sent->p_tx_buff        = p_h_process->tx_buff;
                p_h_i2c_sent->tx_size          = 2;
            }
            break;
            case ICM20948_SENT_SPI:
            {
                h_icm20948_spi_t *p_h_spi_sent = &p_h_process->h_spi_sent;
                p_h_spi_sent->p_tx_buff        = p_h_process->tx_buff;
                p_h_spi_sent->p_rx_buff        = p_h_process->rx_buff;
                p_h_spi_sent->sent_size        = 2;
            }
            }
        }

        if (b_printf)
        {
            printf("> icm20948[%d]_setting:run", p_h_icm->id);
            printf(",%02X", p_h_list->reg_addr);
            printf(",%02X", p_h_process->tx_buff[0]);
            printf(",%d", p_h_list->reg_size);
            if (p_h_list->b_read_write)
            {
                printf(",r");
                printf(",%d", p_h_list->b_check_set);
                printf(",%d", p_h_list->b_check_rst);
            }
            else
            {
                printf(",w");
                printf(",%02X", *p_h_list->p_reg_data);
            }
            printf("\r\n");
        }
        p_h_process->step = STEP_SENT_START;
    }
    case STEP_SENT_START:
        p_h_icm->sent_error_count = 0;
        p_h_icm->b_sent_busy      = true;

        switch (p_h_icm->sent_sel)
        {
        case ICM20948_SENT_I2C:
            p_h_process->step = STEP_I2C_SET;
            break;
        case ICM20948_SENT_SPI:
            p_h_process->step = STEP_SPI_SET;
            break;
        default:
        {
            h_icm20948_setting_t *p_h_setting = p_h_process->p_h_setting;
            p_h_setting->b_error              = true;
            p_h_setting->list_n_error         = p_h_process->reg_list_n;
            p_h_process->step                 = STEP_FINISH;
        }
        }
        break;
    case STEP_I2C_SET:
        if (p_h_icm->i2c_sent_cb == NULL)
        {
            p_h_icm->b_sent_busy = false;
            p_h_process->step    = STEP_SENT_WAIT;
            break;
        }
        if (p_h_icm->i2c_sent_cb(p_h_icm->id, &p_h_process->h_i2c_sent) == false)
            break;
        p_h_process->step = STEP_SENT_WAIT;
        break;
    case STEP_SPI_SET:
        if (p_h_icm->spi_sent_cb == NULL)
        {
            p_h_icm->b_sent_busy = false;
            p_h_process->step    = STEP_SENT_WAIT;
            break;
        }
        if (p_h_icm->spi_sent_cb(p_h_icm->id, &p_h_process->h_spi_sent) == false)
            break;
        p_h_process->step = STEP_SENT_WAIT;
        break;
    case STEP_SENT_WAIT:
    {
        if (p_h_icm->b_sent_busy)
            break;

        h_icm20948_reg_list_t *p_h_list = p_h_process->p_h_list;
        if (p_h_list->b_read_write && (p_h_list->p_reg_data != NULL))
            p_h_list->p_reg_data[0] = p_h_process->rx_buff[1];

        p_h_process->b_error = p_h_icm->sent_res != ICM20948_RES_OK;

        bool b_check = p_h_list->b_check_set;
        b_check |= p_h_list->b_check_set;
        b_check &= p_h_list->b_read_write;
        b_check &= p_h_process->b_error == false;

        if (b_printf)
        {
            printf("> icm20948[%d]_setting:sent", p_h_icm->id);
            printf(",%d", p_h_process->b_error);
            printf(",%d", b_check);
            size_t   reg_size  = p_h_list->reg_size;
            uint8_t *p_rx_buff = &p_h_process->rx_buff[1];
            if (p_h_list->b_read_write)
            {
                printf(",r\r\n");
                for (size_t byte_n = 0; byte_n < reg_size; byte_n++)
                    printf("%02X ", p_rx_buff[byte_n]);
                printf("\r\n");
            }
            else
            {
                printf(",w\r\n");
            }
        }
        if (b_check == false)
        {
            p_h_process->step = STEP_ERROR_CHECK;
            break;
        }
        p_h_process->step = STEP_DATA_CHECK;
    }
        //        break;
    case STEP_DATA_CHECK:
    {
        h_icm20948_reg_list_t *p_h_list     = p_h_process->p_h_list;
        size_t                 reg_size     = p_h_list->reg_size;
        uint8_t               *p_check_buff = p_h_list->p_reg_check;
        size_t                 byte_n;
        bool                   b_error = p_check_buff == NULL;
        p_h_process->b_error           = b_error;
        if (p_h_process->b_error)
        {
            h_icm20948_setting_t *p_h_setting = p_h_process->p_h_setting;
            p_h_setting->b_error              = true;
            p_h_setting->list_n_error         = p_h_process->reg_list_n;
            p_h_process->step                 = STEP_FINISH;
            break;
        }

        uint8_t *p_rx_buff = &p_h_process->rx_buff[1];
        if (p_h_list->b_check_set)
        {
            if (p_h_list->b_check_rst)
            {
                for (byte_n = 0; byte_n < reg_size; byte_n++)
                {
                    b_error |= p_rx_buff[byte_n] != p_check_buff[byte_n];
                    if (b_error)
                        break;
                }
            }
            else
            {
                uint8_t data_check;
                for (byte_n = 0; byte_n < reg_size; byte_n++)
                {
                    data_check = p_rx_buff[byte_n] & p_check_buff[byte_n];
                    b_error |= data_check != p_check_buff[byte_n];
                    if (b_error)
                        break;
                }
            }
        }
        else if (p_h_list->b_check_rst)
        {
            uint8_t data_check;
            for (byte_n = 0; byte_n < reg_size; byte_n++)
            {
                data_check = ~(p_rx_buff[byte_n]) & p_check_buff[byte_n];
                b_error |= data_check != p_check_buff[byte_n];
                if (b_error)
                    break;
            }
        }

        if (b_printf)
        {
            printf("> icm20948[%d]_setting:check", p_h_icm->id);
            size_t   reg_size   = p_h_list->reg_size;
            uint8_t *p_reg_buff = p_h_list->p_reg_check;
            printf(",");
            for (size_t byte_n = 0; byte_n < reg_size; byte_n++)
                printf("%02X ", p_reg_buff[byte_n]);
            printf("\r\n");
        }
        p_h_process->step = STEP_ERROR_CHECK;
    }
    break;
    case STEP_ERROR_CHECK:
    {
        h_icm20948_reg_list_t *p_h_list = p_h_process->p_h_list;
        if (p_h_process->b_error)
        {
            p_h_process->retry_count++;
            if (p_h_process->retry_count < p_h_process->retry_n)
            {
                p_h_process->time_delay = p_h_list->retry_delay;
                if (p_h_process->time_delay > 0)
                {
                    if (p_h_icm->systick_get_cb != NULL)
                        p_h_process->time_log = p_h_icm->systick_get_cb();
                    p_h_process->step = STEP_CHECK_DELAY;
                }
                else
                {
                    p_h_process->step = STEP_SENT_START;
                }
            }
            else if (p_h_list->error_jump_en)
            {
                if (p_h_process->reg_list_n > -p_h_list->error_jump_shift)
                {
                    p_h_process->reg_list_n += p_h_list->error_jump_shift;
                }
                else
                {
                    p_h_process->reg_list_n = 0;
                }
                p_h_process->step = STEP_LIST_N_CHECK;
            }
            else
            {
                h_icm20948_setting_t *p_h_setting = p_h_process->p_h_setting;
                p_h_setting->b_error              = true;
                p_h_setting->list_n_error         = p_h_process->reg_list_n;
                p_h_process->step                 = STEP_FINISH;
            }
        }
        else
        {
            p_h_process->time_delay = p_h_list->delay;
            if (p_h_process->time_delay > 0)
            {
                if (p_h_icm->systick_get_cb != NULL)
                    p_h_process->time_log = p_h_icm->systick_get_cb();
                p_h_process->step = STEP_DELAY;
            }
            else
            {
                p_h_process->reg_list_n++;
                p_h_process->step = STEP_LIST_N_CHECK;
            }
        }

        if (b_printf)
        {
            printf("> icm20948[%d]_setting:res", p_h_icm->id);
            printf(",%d", p_h_process->b_error);
            printf(",%d", p_h_icm->sent_res);
            printf(",%d", p_h_process->retry_count);
            printf(",%d", p_h_process->retry_n);
            printf("\r\n");
        }
        break;
    }
    case STEP_CHECK_DELAY:
    {
        if (p_h_icm->systick_get_cb != NULL)
        {
            uint32_t time_now = p_h_icm->systick_get_cb();
            if (time_now - p_h_process->time_log < p_h_process->time_delay)
                break;
        }

        p_h_process->step = STEP_SENT_START;
        break;
    }
    case STEP_DELAY:
    {
        uint32_t time_diff = 0;
        if (p_h_icm->systick_get_cb != NULL)
        {
            time_diff = p_h_icm->systick_get_cb() - p_h_process->time_log;
            if (time_diff < p_h_process->time_delay)
                break;
        }

        p_h_process->reg_list_n++;
        if (b_printf)
        {
            printf("> icm20948[%d]_setting:delay", p_h_icm->id);
            printf(",%d", p_h_process->time_delay);
            printf(",%d", time_diff);
            printf("\r\n");
        }
        p_h_process->step = STEP_LIST_N_CHECK;
        break;
    }
    default:
        p_h_process->step = STEP_IDLE;
    }
    return p_h_process->step != STEP_IDLE;
}

static bool icm20948_sample_process(h_icm20948_t *p_h_icm)
{
    typedef enum
    {
        STEP_IDLE,
        STEP_START,
        STEP_I2C_SET,
        STEP_SPI_SET,
        STEP_SENT_WAIT,
        STEP_UPDATE,
        STEP_FINISH,
    } step_t;

    // static step_t           step = STEP_IDLE;
    // static h_icm20948_i2c_t h_i2c_sent;
    // static h_icm20948_spi_t h_spi_sent;
    // static uint8_t          tx_buff[ICM20948_SAMPLE_REG_BUFF_LEN] = { 0x0 };
    // static uint8_t          rx_buff[ICM20948_SAMPLE_REG_BUFF_LEN] = { 0x0 };
    // //    static uint32_t time_log = 0;
    static const uint8_t reg_addr_begin = ICM20948_SAMPLE_REG_ADDR_BEGIN;

    h_icm20948_sample_p_t *p_h_process = &p_h_icm->h_sample_p;
    bool                   b_printf    = p_h_icm->h_debug.b_sample;

    if (p_h_icm->b_sample_en == false)
    {
        p_h_process->step = STEP_IDLE;
        return false;
    }

    switch (p_h_process->step)
    {
    case STEP_FINISH:
        p_h_process->step = STEP_IDLE;
    case STEP_IDLE:
        if (p_h_icm->b_init_finish == false)
            break;
        if (p_h_icm->setting_q_front != p_h_icm->setting_q_back)
            break;
        if (p_h_icm->b_sample_start == false)
            break;
        p_h_icm->b_sample_start = false;

        p_h_process->step = STEP_START;
    case STEP_START:
        p_h_icm->sent_error_count = 0;
        p_h_icm->b_sent_busy      = true;

        switch (p_h_icm->sent_sel)
        {
        case ICM20948_SENT_I2C:
        {
            h_icm20948_i2c_t *p_h_i2c_sent = &p_h_process->h_i2c_sent;
            p_h_i2c_sent->dev_addr         = ICM20948_DEV_ADDRESS;
            p_h_i2c_sent->p_tx_buff        = (void *)&reg_addr_begin;
            p_h_i2c_sent->tx_size          = 1;
            p_h_i2c_sent->p_rx_buff        = &p_h_process->rx_buff[1];
            p_h_i2c_sent->rx_size          = ICM20948_SAMPLE_REG_BUFF_LEN - 1;

            p_h_process->step = STEP_I2C_SET;
            break;
        }
        case ICM20948_SENT_SPI:
        {
            h_icm20948_spi_t *p_h_spi_sent = &p_h_process->h_spi_sent;
            p_h_process->tx_buff[0]        = reg_addr_begin | ICM20948_SPI_REG_ADDR_READ_MSK;
            p_h_spi_sent->p_tx_buff        = p_h_process->tx_buff;
            p_h_spi_sent->p_rx_buff        = p_h_process->rx_buff;
            p_h_spi_sent->sent_size        = ICM20948_SAMPLE_REG_BUFF_LEN;

            p_h_process->step = STEP_SPI_SET;
        }
        break;
        default:
            p_h_process->step = STEP_UPDATE;
        }
        // if (b_printf)
        // {
        //     printf(">icm20948_sample:send_start");
        //     printf(",%d", p_h_icm->sent_sel);
        //     printf(",%02X", reg_addr_begin);
        //     printf(",%d", ICM20948_SAMPLE_REG_BUFF_LEN);
        //     printf("\r\n");
        // }
        p_h_process->time_log = p_h_icm->systick_get_cb();
        break;
    case STEP_I2C_SET:
        if (p_h_icm->i2c_sent_cb == NULL)
        {
            p_h_icm->b_sent_busy = false;
            p_h_process->step    = STEP_SENT_WAIT;
            break;
        }
        if (p_h_icm->i2c_sent_cb(p_h_icm->id, &p_h_process->h_i2c_sent) == false)
            break;
        p_h_process->step = STEP_SENT_WAIT;
        break;
    case STEP_SPI_SET:
        if (p_h_icm->spi_sent_cb == NULL)
        {
            p_h_icm->b_sent_busy = false;
            p_h_process->step    = STEP_SENT_WAIT;
            break;
        }
        if (p_h_icm->spi_sent_cb(p_h_icm->id, &p_h_process->h_spi_sent) == false)
            break;
        p_h_process->step = STEP_SENT_WAIT;
        break;
    case STEP_SENT_WAIT:
    {
        uint32_t timeout = p_h_process->timeout;
        if (timeout > 0)
        {
            uint32_t time_diff;
            time_diff = p_h_icm->systick_get_cb() - p_h_process->time_log;
            if (time_diff >= p_h_process->timeout)
            {
                p_h_process->step = STEP_START;
                break;
            }
        }
    }
        if (p_h_icm->b_sent_busy)
            break;
        p_h_process->step = STEP_UPDATE;
    case STEP_UPDATE:
    {
        uint8_t  buff_n_begin = 1;
        size_t   buff_n;
        uint16_t data_u16;
        uint8_t *p_rx_buff = p_h_process->rx_buff;

        buff_n                              = buff_n_begin + ICM20948_REG_ACCEL_XOUT_H - reg_addr_begin;
        data_u16                            = (p_rx_buff[buff_n] << 8) | (p_rx_buff[buff_n + 1]);
        p_h_icm->raw_data[ICM20948_DATA_AX] = data_u16;
        buff_n                              = buff_n_begin + ICM20948_REG_ACCEL_YOUT_H - reg_addr_begin;
        data_u16                            = (p_rx_buff[buff_n] << 8) | (p_rx_buff[buff_n + 1]);
        p_h_icm->raw_data[ICM20948_DATA_AY] = data_u16;
        buff_n                              = buff_n_begin + ICM20948_REG_ACCEL_ZOUT_H - reg_addr_begin;
        data_u16                            = (p_rx_buff[buff_n] << 8) | (p_rx_buff[buff_n + 1]);
        p_h_icm->raw_data[ICM20948_DATA_AZ] = data_u16;
        buff_n                              = buff_n_begin + ICM20948_REG_GYRO_XOUT_H - reg_addr_begin;
        data_u16                            = (p_rx_buff[buff_n] << 8) | (p_rx_buff[buff_n + 1]);
        p_h_icm->raw_data[ICM20948_DATA_GX] = data_u16;
        buff_n                              = buff_n_begin + ICM20948_REG_GYRO_YOUT_H - reg_addr_begin;
        data_u16                            = (p_rx_buff[buff_n] << 8) | (p_rx_buff[buff_n + 1]);
        p_h_icm->raw_data[ICM20948_DATA_GY] = data_u16;
        buff_n                              = buff_n_begin + ICM20948_REG_GYRO_ZOUT_H - reg_addr_begin;
        data_u16                            = (p_rx_buff[buff_n] << 8) | (p_rx_buff[buff_n + 1]);
        p_h_icm->raw_data[ICM20948_DATA_GZ] = data_u16;

        buff_n_begin                        = ICM20948_REG_EXT_SLV_SENS_DATA_00 - reg_addr_begin;
        uint8_t reg_addr_begin_ak           = AK09916_REG_ST1;
        buff_n                              = buff_n_begin + AK09916_REG_HXL - reg_addr_begin_ak;
        data_u16                            = (p_rx_buff[buff_n + 1] << 8) | (p_rx_buff[buff_n]);
        p_h_icm->raw_data[ICM20948_DATA_MX] = data_u16;
        buff_n                              = buff_n_begin + AK09916_REG_HYL - reg_addr_begin_ak;
        data_u16                            = (p_rx_buff[buff_n + 1] << 8) | (p_rx_buff[buff_n]);
        p_h_icm->raw_data[ICM20948_DATA_MY] = data_u16;
        buff_n                              = buff_n_begin + AK09916_REG_HZL - reg_addr_begin_ak;
        data_u16                            = (p_rx_buff[buff_n + 1] << 8) | (p_rx_buff[buff_n]);
        p_h_icm->raw_data[ICM20948_DATA_MZ] = data_u16;

        if (p_h_icm->sample_finish_cb != NULL)
            p_h_icm->sample_finish_cb(p_h_icm->id);

        if (b_printf)
        {
            printf(">icm20948_sample:update,%d", p_h_icm->id);
            printf(",");
            for (size_t data_n = 0; data_n < ICM20948_DATA_NUM; data_n++)
                printf("%6d", p_h_icm->raw_data[data_n]);
            //                printf("%04X ", raw_data[data_n]);
            printf("\r\n");
        }
        p_h_process->step = STEP_FINISH;
    }
    }
    return p_h_process->step != STEP_IDLE;
}

// bool icm20948_init_finish_get(void)
// {
//     return p_h_icm->b_init_finish;
// }

static bool icm20948_setting_set(h_icm20948_t *p_h_icm, h_icm20948_setting_t *p_h_setting)
{
    if (p_h_setting == NULL)
        return false;
    if (p_h_setting->list_len <= 0)
        return false;

    size_t back_next = p_h_icm->setting_q_back + 1;
    if (back_next >= ICM20948_SETTING_SEQ_Q_LEN)
        back_next = 0;
    if (back_next == p_h_icm->setting_q_front)
        return false;

    p_h_setting->b_busy                             = true;
    p_h_setting->b_error                            = false;
    p_h_setting->list_n_error                       = 0;
    p_h_icm->p_h_setting_q[p_h_icm->setting_q_back] = p_h_setting;

    if (p_h_icm->h_debug.b_setting)
    {
        printf("> icm20948[%d]_setting:set", p_h_icm->id);
        printf(",%d", p_h_icm->setting_q_back);
        printf("\r\n");
    }

    p_h_icm->setting_q_back = back_next;
    return true;
}

// void icm20948_debug_set(h_icm20948_debug_t *p_handler)
// {
//     memcpy(&icm20948_debug, p_handler, sizeof(h_icm20948_debug_t));
// }
// void icm20948_debug_get(h_icm20948_debug_t *p_handler)
// {
//     memcpy(p_handler, &icm20948_debug, sizeof(h_icm20948_debug_t));
// }

// bool icm20948_en_set(bool b_enable)
// {
//     b_icm20948_en = p_h_icm->b_init_finish && b_enable;

//     if (icm20948_debug.b_enable)
//     {
//         printf("> icm20948_en:set");
//         printf(",%d", p_h_icm->b_init_finish);
//         printf(",%d", b_enable);
//         printf(",%d", b_icm20948_en);
//         printf("\r\n");
//     }
//     return b_icm20948_en;
// }
// bool icm20948_en_get(void)
// {
//     return b_icm20948_en;
// }

void icm20948_motion_detect_set(h_icm20948_t *p_h_icm, bool b_enable)
{
    p_h_icm->b_motion_det_en = b_enable;
}

bool icm20948_motion_detect_get(h_icm20948_t *p_h_icm)
{
    return p_h_icm->b_motion_det_en_current;
}

void icm20948_low_power_set(h_icm20948_t *p_h_icm, bool b_enable)
{
    p_h_icm->b_low_power = b_enable;
}

bool icm20948_low_power_get(h_icm20948_t *p_h_icm)
{
    return p_h_icm->b_low_power_current;
}

// int16_t icm20948_data_get(icm20948_data_t data_n)
// {
//     if (data_n >= ICM20948_DATA_NUM)
//         return 0;
//     return raw_data[data_n];
// }
