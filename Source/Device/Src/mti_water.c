#include <stdlib.h>
#include <math.h>
#include "mti_water.h"
#include "mti_imu.h"
#include "mti_system.h"
#include "vmt_device.h"
#include "tim.h"
#include "vmt_command.h"

float         water_threshold_ratio = 1.5;
uint16_t      water_high_reserve    = 0;
uint16_t      water_value;
uint16_t      water_value_1;
uint16_t      water_value_2;
uint16_t      water_high;
uint16_t      water_low;
uint8_t       messageID_water  = 10;
float         water_multiplier = 1.4;
bool          b_water;
bool          adc_print;
bool          configured;
uart_select_t water_ch;


bool water_detect(void)
{
    uart_tx_channel_set(water_ch);
    if (state_get() != measure_state)
        return b_water;

    water_value_1 = adc_value_get(ADC_SEQ_WATER_BEGIN);
    water_value_2 = adc_value_get(ADC_SEQ_WATER_BEGIN + 1);
    if (water_value_1 > water_value_2)
        water_value = water_value_1;
    else
        water_value = water_value_2;
    if (adc_print)
        printf("@val,%.0lf,%u,%u\n", adc_value_get(ADC_SEQ_TEMP), water_value_1, water_value_2);
    // printf("@val,%d\n",water_value);
    uart_tx_channel_set(UART_UPHOLE);
    if (!b_water)
    {
        if (water_value > water_high_reserve)
        {
            keepalive_reset();
            printf("!water,%d,1\r\n", messageID_water);
            printf("!water,%d,1\r\n", messageID_water);
            if (messageID_water == 19)
                messageID_water = 10;
            else
                messageID_water++;
            b_water = true;
            y_max_reset();
        }
    }
    else
    { // b_water == true
        if (water_high < water_value)
        {
            water_high = water_value;
            water_low  = water_high / water_multiplier; // increase threshold as water makes contact;
        }
        if (water_low > water_high_reserve)
            water_low = water_high_reserve / water_multiplier;
        if (water_value < water_high_reserve)
        {
            // water_low = water_value;
            // water_high = water_value*water_multiplier;
            // if(water_high < water_high_reserve) water_high = water_high_reserve; //reserve threshold
            keepalive_reset();
            printf("!water,%d,0\r\n", messageID_water);
            printf("!water,%d,0\r\n", messageID_water);
            if (messageID_water == 19)
                messageID_water = 10;
            else
                messageID_water++;
            b_water = false;
        }
    }

    // if(debug) printf("@val,%dV\r\n",water_value);
    // printf("water: %d,%f,%d\n",pin, adc_value_get(ADC_CH_WATER),HAL_GPIO_ReadPin(WATER_DET_GPIO_Port, WATER_DET_Pin));
    // printf("water: %f,%f\n", adc_value_get(ADC_CH_WATER),adc_value_get(2));

    // HAL_Delay(100);
    return b_water;
}

void multiplier_set(float multiplier)
{
    water_multiplier = multiplier;
}

void reserve_set(uint16_t reserve)
{
    if (reserve == 0)
        water_high_reserve = 1280;
    else
        water_high_reserve = reserve;
    if (module_status_get() == STATUS_SYNC)
        module_status_set(STATUS_OK);
}

uint16_t reserve_get(void)
{
    return water_high_reserve;
}

bool water_get(void)
{
    return b_water;
}

void water_reset(void)
{
    water_value = adc_value_get(ADC_SEQ_WATER_BEGIN);
    // water_low = water_value;
    water_high = water_value * water_multiplier;
    if (water_high < water_high_reserve)
        water_high = water_high_reserve; // reserve threshold
    b_water = false;
    printf("@db,water threshold set:%d|%d (%d)\n", water_low, water_high, water_high_reserve);
}
