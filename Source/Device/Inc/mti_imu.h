#include "vmt_device.h"

typedef enum
{
    IMU_TEST_FILL,
    IMU_TEST_COMPARE,
    IMU_TEST_COMPLETE,
    IMU_TEST_OK,
    IMU_TEST_ERROR,

    IMU_TEST_NUM
} imu_test_t;

void       bottom_detect(h_imu_t *p_h_imu);
void       imu_reset(h_imu_t *p_h_imu_1, h_imu_t *p_h_imu_2);
void       imu_compare(h_imu_t *p_h_imu_1, h_imu_t *p_h_imu_2);
imu_test_t imu_test(h_imu_t *p_h_imu);
void       imu_test_reset(void);
bool       accelerating_get(void);
void       y_max_reset(void);
void       bump_detected(void);
void       imu_value_set(char *var_name, char *var);
int16_t    gyro_angle_get(void);
uint8_t    imu_active_get(void);
void       imu_active_set(uint8_t);
void       imu_profile_set(int16_t profile);
int16_t    imu_profile_get(void);

extern uint8_t       live_imu;
extern uart_select_t imu_ch;
extern float         ratio_air;
extern float         ratio_water;
