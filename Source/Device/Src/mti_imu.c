#include <stdlib.h>
#include <math.h>
#include "mti_imu.h"
#include "mti_water.h"
#include "mti_system.h"
// #include "vmt_device.h"
#include "tim.h"
#include "vmt_icm20948.h"
#include "vmt_command.h"

int16_t  acc_y_last = 0;
int16_t  acc_y_diff = 0;
float    jerk_ratio = 0;
int16_t  acc_y_max  = 0;
double   acc_verticle;
double   angle_radians;
float    component_x;
float    component_y;
float    component_z;
float    ratio_air   = 100;
float    ratio_water = 2.5;
uint16_t threshold_dynamic;
uint16_t threshold         = 2048 * 2;
uint16_t threshold_average = 3500;
float    threshold_ratio;
float    acc_y_ave;
uint8_t  ave_count;
int16_t  acc_y_values[15];
int32_t  acc_y_ave_sum;
float    acc_h;
float    acc_h_last = 0;
float    acc_h_diff = 0;
double   angle_gyro;
double   angle_delta;
double   angle_x;
double   angle_z;
double   length_x;
double   length_z;
double   length_r;
float    inc_x;
float    inc_z;
bool     gyro_reset;
uint32_t time_calc;
uint32_t time_pulse;
// uint32_t time_fall;
int16_t  jerk = 0;
double   last_acceleration;
bool     jerking;
bool     accelerating_y;
bool     accelerating_l;
bool     acc_data = false;
bool     idle;
bool     idle_confirm;
bool     rotating;
bool     fallen_over;
int16_t  amplitude;
uint32_t time_start_mpu;
uint16_t cycles       = 0;
uint16_t stat_samples = 0;
uint32_t time_pulse;
uint32_t time_sample;
// uint32_t time_fall;
uint32_t      time_idle;
uint8_t       messageID = 0;
uint8_t       live_imu  = 0;
uart_select_t imu_ch;
uint32_t      time_idle;
uint16_t      samples;
uint32_t      startTime;
uint32_t      endTime;
uint8_t       imu_active = 10;
int16_t       imu_profile;

h_imu_t  data_FIFO[2][10];
uint8_t  self_test_complete[2];
bool     water_send;
uint16_t threshold_air    = 4000;
uint16_t threshold_water  = 3000;
float    div_differential = 0.5;
float    div_acc_max      = 1.25;
uint8_t  ave_sample       = 5;
uint8_t  ave_mode         = 1;

uint8_t imu_active_get(void)
{
    return imu_active; // 10: not set, 30: error
}

void imu_active_set(uint8_t active)
{
    imu_active = active;
}

void imu_profile_set(int16_t profile)
{
    imu_profile = profile;
}

int16_t imu_profile_get(void)
{
    return imu_profile;
}

void imu_value_set(char *var_name, char *var)
{
    if (strcmp(var_name, "threshold_air") == 0)
        threshold_air = atoi(var);
    else if (strcmp(var_name, "threshold_water") == 0)
        threshold_water = atoi(var);
    else if (strcmp(var_name, "diff") == 0)
        div_differential = atof(var);
    else if (strcmp(var_name, "max") == 0)
        div_acc_max = atof(var);
    else if (strcmp(var_name, "ave_sample") == 0)
        ave_sample = atoi(var);
    else if (strcmp(var_name, "ave_mode") == 0)
    {
        if (strcmp(var, "y") == 0)
            ave_mode = 1;
        else if (strcmp(var, "l") == 0)
            ave_mode = 2;
    }
    else if (strcmp(var_name, "ave_threshold") == 0)
        threshold_average = atoi(var);
    else
    {
        printf("@db,Invaid\n");
        return;
    }
    uart_tx_channel_set(UART_UPHOLE);
}

void imu_test_reset(void)
{
    self_test_complete[0] = 0;
    self_test_complete[1] = 0;
}

imu_test_t imu_test(h_imu_t *p_h_imu)
{
    uint8_t id = p_h_imu->id;

    if (self_test_complete[id] == 200)
        return IMU_TEST_COMPLETE;
    for (int i = 9; i > 0; i--)
    {
        data_FIFO[id][i] = data_FIFO[id][i - 1]; // shift data FIFO
    }
    data_FIFO[id][0] = *p_h_imu;
    // data_FIFO[id][0].accel_x = 0;
    if (self_test_complete[id] < 20)
    { // run through a few times before testing

        self_test_complete[id]++;
        // printf("@db,first 10\n");
        return IMU_TEST_FILL;
    }
    // printf("@val,Acc X: %d/%d\n",data_FIFO[id][0].accel_x,p_h_imu->accel_x);
    for (int i = 0; i < 9; i++)
    { // scan through array for unchanged values
        bool hold = false;
        if (data_FIFO[id][i].accel_x == data_FIFO[id][i + 1].accel_x &&
            (data_FIFO[id][i].accel_x == -32768 || data_FIFO[id][i].accel_x == -1 || data_FIFO[id][i].accel_x == 32767))
            hold = true;
        if (data_FIFO[id][i].accel_y == data_FIFO[id][i + 1].accel_y &&
            (data_FIFO[id][i].accel_y == -32768 || data_FIFO[id][i].accel_y == -1 || data_FIFO[id][i].accel_y == 32767))
            hold = true;
        if (data_FIFO[id][i].accel_z == data_FIFO[id][i + 1].accel_z &&
            (data_FIFO[id][i].accel_z == -32768 || data_FIFO[id][i].accel_z == -1 || data_FIFO[id][i].accel_z == 32767))
            hold = true;
        if (data_FIFO[id][i].gyro_x == data_FIFO[id][i + 1].gyro_x &&
            (data_FIFO[id][i].gyro_x == -32768 || data_FIFO[id][i].gyro_x == -1 || data_FIFO[id][i].gyro_x == 32767))
            hold = true;
        if (data_FIFO[id][i].gyro_y == data_FIFO[id][i + 1].gyro_y &&
            (data_FIFO[id][i].gyro_y == -32768 || data_FIFO[id][i].gyro_y == -1 || data_FIFO[id][i].gyro_y == 32767))
            hold = true;
        if (data_FIFO[id][i].gyro_z == data_FIFO[id][i + 1].gyro_z &&
            (data_FIFO[id][i].gyro_z == -32768 || data_FIFO[id][i].gyro_z == -1 || data_FIFO[id][i].gyro_y == 32767))
            hold = true;
        if (data_FIFO[id][i].mag_x == data_FIFO[id][i + 1].mag_x &&
            (data_FIFO[id][i].mag_x == -32768 || data_FIFO[id][i].mag_x == -1 || data_FIFO[id][i].mag_x == 32767))
            hold = true;
        if (data_FIFO[id][i].mag_y == data_FIFO[id][i + 1].mag_y &&
            (data_FIFO[id][i].mag_y == -32768 || data_FIFO[id][i].mag_y == -1 || data_FIFO[id][i].mag_y == 32767))
            hold = true;
        if (data_FIFO[id][i].mag_z == data_FIFO[id][i + 1].mag_z &&
            (data_FIFO[id][i].mag_z == -32768 || data_FIFO[id][i].mag_z == -1 || data_FIFO[id][i].mag_z == 32767))
            hold = true;
        if (!hold)
        {
            self_test_complete[id] = 200;
            return IMU_TEST_OK; // keep scanning if unchanged value detected;
        }
    }
    self_test_complete[id] = 200;
    return IMU_TEST_ERROR;
}

void bottom_detect(h_imu_t *p_h_imu)
{
    if (samples == 0)
        startTime = HAL_GetTick();

    if (p_h_imu->accel_y == acc_y_last)
        return;
    // else if (samples == 10000)
    // {
    //     endTime         = HAL_GetTick();
    //     double duration = endTime - startTime;
    //     duration /= 1000;
    //     printf("@db,Time: %0.3lfs | Frequency: %lfHz\n", duration, 10000 / duration);
    // }
    // samples++;
    // return;
    // NOTE timediff not currently used
    static uint32_t time_diff;
    time_diff   = HAL_GetTick() - time_sample;
    time_sample = HAL_GetTick();
    if (time_diff > 200)
        time_diff = 0;

    int i;

    angle_radians = math_pi * p_h_imu->h_accel.angle / 180;
    component_y   = cos(angle_radians);
    acc_verticle  = component_y * 2024;
    amplitude     = p_h_imu->accel_y - acc_verticle;
    amplitude     = abs(amplitude);

    // Calculate y acc running average // redundant
    acc_y_ave_sum -= acc_y_values[ave_sample - 1];
    for (i = ave_sample - 1; i > 0; i--)
    {
        acc_y_values[i] = acc_y_values[i - 1];
    }
    if (ave_mode == 1)
        acc_y_values[0] = p_h_imu->accel_y;
    else if (ave_mode == 2)
        acc_y_values[0] = p_h_imu->h_accel.len;
    acc_y_ave_sum += acc_y_values[0];
    // acc_y_last = acc_y_ave;
    acc_y_ave = (float)acc_y_ave_sum / ave_sample;

    if (acc_y_ave - 2070 < 500 && acc_y_ave - 2070 > -500)
        accelerating_y = false;
    else
        accelerating_y = true;
    // end

    // MPU9255_read_accel(p_h_mpu9255); acc_y_ave = p_h_mpu9255->accel_y.i16; //bypass average

    // calc horizontal acc //redundant
    acc_h = sqrt((p_h_imu->accel_x) * (p_h_imu->accel_x) + (p_h_imu->accel_z) * (p_h_imu->accel_z));
    // end

    // time interval
    acc_y_diff = p_h_imu->accel_y - acc_y_last;
    acc_y_diff = abs(acc_y_diff);
    jerk_ratio = (float)acc_y_diff / (float)amplitude;


    // p_h_imu->gyro_x -=210;
    // p_h_imu->gyro_y -=270;

    if (p_h_imu->gyro_x > 200 && p_h_imu->gyro_x + 365 < -200 && p_h_imu->gyro_z > 200 && p_h_imu->gyro_z < -200)
        rotating = true;
    else
        rotating = false;

    // Accelerating l
    if (p_h_imu->h_accel.len - 2070 < 350 && p_h_imu->h_accel.len - 2070 > -350)
    {
        // Phi (spherical from vertical) (eq10)
        // angle_acc = acos((float)p_h_imu_1->accel_y/p_h_imu_1->accel_len)*180/math_pi;
        // Theta (cartesian x from vertical) (eq11)
        double square = sqrt(p_h_imu->accel_y * p_h_imu->accel_y + p_h_imu->accel_z * p_h_imu->accel_z);
        inc_x         = atan((double)p_h_imu->accel_x / square) * 180 / math_pi;
        component_x   = sin(inc_x * math_pi / 180); // redundant
        // Psi (cartesian z from vertical) (eq12)
        square      = sqrt(p_h_imu->accel_x * p_h_imu->accel_x + p_h_imu->accel_y * p_h_imu->accel_y);
        inc_z       = atan((double)p_h_imu->accel_z / square) * 180 / math_pi;
        component_z = sin(inc_z * math_pi / 180); // redunant

        // gyro_reset = false;
        accelerating_l = false;
    }

    else
    {
        accelerating_l = true;
        idle           = false;
        idle_confirm   = false;
    }

    if (amplitude < threshold)
    {
        // gyro propoertional reset
        if (!accelerating_l && !rotating && p_h_imu->h_accel.angle < 90)
        {
            angle_x += (inc_z - angle_x) / 40;
            angle_z += (inc_x - angle_z) / 40;
        }
        // gyro angle integration
        time_calc   = HAL_GetTick() - time_calc;
        angle_delta = -(float)p_h_imu->gyro_x * (time_calc) / 32768;
        if (angle_delta > 0.5 || angle_delta < -0.5)
            angle_x += angle_delta;

        angle_delta = (float)p_h_imu->gyro_z * (time_calc) / 32768;
        if (angle_delta > 0.5 || angle_delta < -0.5)
            angle_z += angle_delta;

        time_calc = HAL_GetTick();
    }
    length_x   = sin(angle_x * math_pi / 180);
    length_z   = sin(angle_z * math_pi / 180);
    length_r   = sqrt(length_x * length_x + length_z * length_z);
    angle_gyro = 90 - (180 / math_pi * acos(length_r)); // change

    // angle_gyro = sqrt(angle_x*angle_x + angle_z*angle_z); //todo revise

    uart_tx_channel_set(UART_UPHOLE);
    if (HAL_GetTick() - time_pulse > 800)
        acc_y_max = 0;

    // Fallen over conditions
    if (angle_gyro > 50 && p_h_imu->h_accel.angle > 50 && !fallen_over)
    {
        fallen_over = true;
        // printf("!tilt,%d,1\r\n", messageID);
        // printf("!tilt,%d,1\r\n", messageID);
        // if (messageID == 9)
        //     messageID = 0;
        // else
        //     messageID++;
        printf("@db,Fallen over\n");
        return;
    }
    else if (angle_gyro < 20 && p_h_imu->h_accel.angle < 20 && fallen_over)
    { // return to upright
        fallen_over = false;
        // printf("!tilt,%d,0\r\n", messageID);
        // printf("!tilt,%d,0\r\n", messageID);
        // if (messageID == 9)
        //     messageID = 0;
        // else
        //     messageID++;
        printf("@db,Upright\n");
    }

    // Bump    (4g+)
    if (water_get())
    { // in water
        if (amplitude > 2500)
            printf("@db,BUMP amp: %d, acc_y_ave: %f\n", amplitude, acc_y_ave); // remove
        if (acc_y_ave > threshold_average || amplitude > threshold_water)
        {
            bump_detected();
        }
    } // not in water
    else if (HAL_GetTick() - time_pulse > 400 || amplitude > acc_y_max / div_acc_max)
    {
        if (amplitude > 3000)
            printf("@db,BUMP amp: %d, acc_y_diff: %d\n", amplitude, acc_y_diff); // remove
        if (amplitude > threshold_air && acc_y_diff >= threshold_air / div_differential)
        {
            //&& (float)amplitude/p_h_imu->h_accel.len > component_y/2.2) { //remove
            bump_detected();
        } // threshold

    } // out of water
    if (!accelerating_l)
    { // time stamp change of state of idle
        if (!idle)
        {
            time_idle = HAL_GetTick();
            // if(debug) printf("@db,idle timestamp\r\n");
            idle         = true;
            idle_confirm = false;
            return;
        }
        else if (HAL_GetTick() - time_idle > 1500 && !idle_confirm)
        { // 1.5s since CoS
            keepalive_reset();
            printf("!idle,%d,%d\n", messageID, 1);
            printf("!idle,%d,%d\n", messageID, 1);
            if (messageID == 9)
                messageID = 0;
            else
                messageID++;
            idle_confirm = true;
            return;
        } // idle time
    } // idle

    if (HAL_GetTick() - time_pulse > 480)
        water_send = false;
    else if (water_get() && water_send && time_pulse + 100 < HAL_GetTick())
    { // resend water
        water_send = false;
        keepalive_reset();
        printf("!water,%d,1\r\n", messageID);
        printf("!water,%d,1\r\n", messageID);
        if (messageID == 9)
            messageID = 0;
        else
            messageID++;
    }
    acc_y_last = p_h_imu->accel_y;
} // bottom detect function

void bump_detected(void)
{
    time_pulse = HAL_GetTick();
    acc_y_max  = amplitude;
    // state = bottom_state;
    if (fallen_over && false)
    { // fallen over
        if (debug_get())
            printf("@db,bump ignored(fallen over)\r\n");
        return;
    } // fallen over
    else
    { // not fallen over
        if (jerk_ratio < 0)
            jerk_ratio *= -1; // absolute value
        keepalive_reset();
        printf("!bump,%d,%d,%d,%d\n", messageID, amplitude, acc_y_diff, water_get());
        printf("!bump,%d,%d,%d,%d\n", messageID, amplitude, acc_y_diff, water_get());
        // printf("!acc,%d,%f,%d,%d\n",messageID,jerk_ratio,p_h_imu->accel_y,water_get());
        water_send = true;
        if (messageID == 9)
            messageID = 0;
        else
            messageID++;
        return;
    } // not fallen over
}

void imu_compare(h_imu_t *p_h_imu_1, h_imu_t *p_h_imu_2)
{
    // printf("@val,%d:%d:%d\r\n",p_h_imu_1->mag_x,p_h_imu_1->mag_y,p_h_imu_1->mag_z);
    int16_t diff_acc_x = p_h_imu_1->accel_x - p_h_imu_2->accel_x;
    diff_acc_x         = abs(diff_acc_x);
    int16_t diff_acc_y = p_h_imu_1->accel_y - p_h_imu_2->accel_y;
    diff_acc_y         = abs(diff_acc_y);
    int16_t diff_acc_z = p_h_imu_1->accel_z - p_h_imu_2->accel_z;
    diff_acc_z         = abs(diff_acc_z);

    int16_t diff_gyro_x = p_h_imu_1->gyro_x - p_h_imu_2->gyro_x;
    diff_gyro_x         = abs(diff_gyro_x);
    int16_t diff_gyro_y = p_h_imu_1->gyro_y - p_h_imu_2->gyro_y;
    diff_gyro_y         = abs(diff_gyro_y);
    int16_t diff_gyro_z = p_h_imu_1->gyro_z - p_h_imu_2->gyro_z;
    diff_gyro_z         = abs(diff_gyro_z);

    uart_tx_channel_set(imu_ch);
    switch (live_imu)
    {
    case 1:
        printf("@val,%d:%d:%d\r\n", p_h_imu_1->accel_x, p_h_imu_1->accel_y, p_h_imu_1->accel_z);
        break;
    case 2:
        printf("@val,%d:%d:%d||%d,%d,%d\r\n", p_h_imu_1->gyro_x, p_h_imu_1->gyro_y, p_h_imu_1->gyro_z, p_h_imu_2->gyro_x, p_h_imu_2->gyro_y, p_h_imu_2->gyro_z);
        break;
    case 3:
        printf("@val,%d:%d:%f\r\n", amplitude, acc_y_diff, jerk_ratio);
        break;
    case 4:
        printf("@val,%lf:%lf:%f\r\n", p_h_imu_1->h_accel.angle, p_h_imu_2->h_accel.angle, angle_gyro);
        break;
    case 5:
        printf("@val,%lf:%lf:%lf:%lf\r\n", inc_x, inc_z, angle_x, angle_z);
        break;
    case 6:
        printf("@val,%d:%d:%d||%d,%d,%d\r\n", p_h_imu_1->accel_x, p_h_imu_1->accel_y, p_h_imu_1->accel_z, p_h_imu_2->accel_x, p_h_imu_2->accel_y, p_h_imu_2->accel_z);
        break;
    case 7:
        printf("@val,%d\n", p_h_imu_1->accel_y);
        break;
    case 8:
        printf("@val,%d\n", p_h_imu_1->accel_y);
        break;
    case 9:
        printf("@val,%d,%d,%d\n", diff_acc_x, diff_acc_y, diff_acc_z);
        break;
    case 10:
        printf("@val,%d,%d,%d\n", diff_gyro_x, diff_gyro_y, diff_gyro_z);
        break;
    case 11:
        printf("@val,%d:%d:%d\r\n", p_h_imu_1->mag_x, p_h_imu_1->mag_y, p_h_imu_1->mag_z);
        break;
    case 12:
        printf("@val,%lf:%f\r\n", p_h_imu_2->h_accel.len, p_h_imu_2->h_gyro.len);
        break;
    case 13:
        printf("@val,%.2lf:%.2f:%.2lf:%d:%d\r\n", p_h_imu_2->h_accel.angle, p_h_imu_2->h_accel.len, acc_verticle, p_h_imu_2->accel_y, amplitude);
        break;
    case 14:
        printf("@val,%.2lf:%d:%d:%.2lf\r\n", acc_verticle, p_h_imu_2->accel_y, amplitude, p_h_imu_2->h_accel.len);
        break;
    case 15:
        printf("@val,%.4f:%.4f:%.4f\r\n", component_x, component_y, component_z);
        break;
    case 16:
        printf("@val,%d:%.4f:%d:%.4lf\r\n", p_h_imu_2->accel_y, component_y, amplitude, p_h_imu_2->h_accel.len);
        break;
    case 17:
        printf("@val,%d:%d:%d\r\n", p_h_imu_1->gyro_x, p_h_imu_1->gyro_y, p_h_imu_1->gyro_z);
        break;
    case 18:
        printf("@val,%d:%d:%d\r\n", p_h_imu_2->gyro_x, p_h_imu_2->gyro_y, p_h_imu_2->gyro_z);
        break;
    case 19:
        printf("@val,%.2lf:%.2lf:%.4lf:%.4lf,%.2lf\r\n", angle_x, angle_z, length_x, length_z, angle_gyro);
        break;
    default:
        printf("@db,No entry for data set %d\n", live_imu);
        live_imu = 0;
        break;
    }
    uart_tx_channel_set(UART_UPHOLE);
}

void y_max_reset(void)
{
    acc_y_max = 0;
}

void imu_reset(h_imu_t *p_h_imu_1, h_imu_t *p_h_imu_2)
{
    fallen_over  = false;
    idle         = false;
    idle_confirm = false;
    acc_y_max    = 0;
    // MPU9255_read_accel(p_h_mpu9255);
    if (imu_active == 0)
    {
        angle_x = asin((double)p_h_imu_1->accel_z / 2048) * 180 / math_pi;
        angle_z = asin((double)p_h_imu_1->accel_x / 2048) * 180 / math_pi;
    }
    else if (imu_active == 1)
    {
        angle_x = asin((double)p_h_imu_2->accel_z / 2048) * 180 / math_pi;
        angle_z = asin((double)p_h_imu_2->accel_x / 2048) * 180 / math_pi;
    }
    else
    {
        angle_x = 0;
        angle_z = 0;
    }
    time_calc = HAL_GetTick();
    // if(debug) printf("@db,%2.2f:%2.2f:%2.2f:%2.2f\r\n", angle_x,inc_x,angle_z,inc_z);
    if (debug_get())
        printf("@db,Gyro position set: %2.2f:%2.2f\r\n", angle_x, angle_z);
    // b_water = false;
    // bottom_confirmed = false;
    cycles = 0;
}

bool accelerating_get(void)
{
    return accelerating_l;
}

int16_t gyro_angle_get(void)
{
    return angle_gyro * 100;
}
