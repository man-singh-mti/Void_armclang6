#include "vmt_adc.h"

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "adc.h"
#include "tim.h"

/* type define */
typedef uint32_t (*clk_source_get_t)(void);

typedef struct h_moving_avg_i16_
{
    uint16_t *p_buff;
    size_t    buff_len;
    size_t    data_len;
    size_t    data_n;
    double    sum;
    double    avg;
} h_moving_avg_u16_t;

/* function declaration */
static void moving_avg_u16_update(h_moving_avg_u16_t *p_h_avg, uint16_t new_data);
static void moving_avg_u16_reset(h_moving_avg_u16_t *p_h_avg);

/* parameter */
static ADC_HandleTypeDef *p_h_hal_adc[ADC_HAL_SEQ_NUM] = {
    [ADC_HAL_SEQ_TEMP]  = &hadc1,
    [ADC_HAL_SEQ_WATER] = &hadc2,
};

#define ADC_REG_TEMP  (ADC1)
#define ADC_REG_WATER (ADC2)

static TIM_HandleTypeDef *p_h_hal_timer[ADC_HAL_SEQ_NUM] = {
    [ADC_HAL_SEQ_TEMP]  = &htim2,
    [ADC_HAL_SEQ_WATER] = &htim4,
};

static clk_source_get_t clk_source_get[ADC_HAL_SEQ_NUM] = {
    [ADC_HAL_SEQ_TEMP]  = HAL_RCC_GetPCLK1Freq,
    [ADC_HAL_SEQ_WATER] = HAL_RCC_GetPCLK1Freq,
};

static uint16_t adc_buff[ADC_SEQ_NUM];
static size_t   adc_buff_len[ADC_HAL_SEQ_NUM] = {
    [ADC_HAL_SEQ_TEMP]  = 1,
    [ADC_HAL_SEQ_WATER] = ADC_SEQ_WATER_NUM,
};

static uint16_t avg_buff_temp[ADC_AVG_BUFF_LEN_TEMP];
static uint16_t avg_buff_water[ADC_SEQ_WATER_NUM][ADC_AVG_BUFF_LEN_WATER];
static h_moving_avg_u16_t h_avg[ADC_SEQ_NUM] = {
    [ADC_SEQ_TEMP] = {
        .p_buff = avg_buff_temp,
        .buff_len = ADC_AVG_BUFF_LEN_TEMP, },
    [ADC_SEQ_WATER_BEGIN + 0] = {
        .p_buff = avg_buff_water[0],
        .buff_len = ADC_AVG_BUFF_LEN_WATER,},
    [ADC_SEQ_WATER_BEGIN + 1] = {
        .p_buff = avg_buff_water[1],
        .buff_len = ADC_AVG_BUFF_LEN_WATER, }, };

/* adc_volt_coeffi is coefficient of adc value to analog voltage */
static double adc_volt_coeffi[ADC_SEQ_NUM] = {
    [ADC_SEQ_TEMP]            = (ADC_VDD_VOLT * (1000 + 240)) / (4096 * 240),
    [ADC_SEQ_WATER_BEGIN + 0] = (ADC_VDD_VOLT) / (4096),
    [ADC_SEQ_WATER_BEGIN + 1] = (ADC_VDD_VOLT) / (4096),
};

static h_adc_debug_t h_adc_debug = {
    .b_init       = true,
    .b_enable     = true,
    .b_sample_set = true,
};

/* function */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    switch ((size_t)hadc->Instance)
    {
    case (size_t)ADC_REG_TEMP:
        adc_sample_finish_cb(ADC_SEQ_TEMP);
        moving_avg_u16_update(&h_avg[ADC_SEQ_TEMP], adc_buff[ADC_SEQ_TEMP]);
        break;
    case (size_t)ADC_REG_WATER:
        for (adc_seq_t seq = ADC_SEQ_WATER_BEGIN; seq <= ADC_SEQ_WATER_END; seq++)
        {
            adc_sample_finish_cb(seq);
            moving_avg_u16_update(&h_avg[seq], adc_buff[seq]);
        }
    }
}

__weak void adc_sample_finish_cb(adc_seq_t sequence)
{
}

bool adc_init(adc_hal_seq_t seq)
{
    if (h_adc_debug.b_init)
    {
        printf("> adc init:start,%d\r\n", seq);
    }

    void *p_buff;
    switch (seq)
    {
    case ADC_HAL_SEQ_TEMP:
        p_buff = &adc_buff[ADC_SEQ_TEMP];
        break;
    case ADC_HAL_SEQ_WATER:
        p_buff = &adc_buff[ADC_SEQ_WATER_BEGIN];
        break;
    default:
        return false;
    }

    HAL_StatusTypeDef res;
    res = HAL_ADC_Start_DMA(p_h_hal_adc[seq], p_buff, adc_buff_len[seq]);
    if (h_adc_debug.b_init)
    {
        printf("> adc init:finish,%d\r\n", res);
    }

    switch (seq)
    {
    case ADC_HAL_SEQ_TEMP:
        moving_avg_u16_reset(&h_avg[ADC_SEQ_TEMP]);
        break;
    case ADC_HAL_SEQ_WATER:
        for (adc_seq_t seq_value = ADC_SEQ_WATER_BEGIN; seq_value <= ADC_SEQ_WATER_END; seq_value++)
        {
            moving_avg_u16_reset(&h_avg[seq_value]);
        }
    }
    return res == HAL_OK;
}

void adc_deinit(adc_hal_seq_t seq)
{
    if (seq >= ADC_HAL_SEQ_NUM)
        return;

    HAL_TIM_Base_DeInit(p_h_hal_timer[seq]);
    HAL_ADC_DeInit(p_h_hal_adc[seq]);
}

void adc_debug_set(h_adc_debug_t *p_handle)
{
    memcpy(&h_adc_debug, p_handle, sizeof(h_adc_debug_t));
}

void adc_debug_get(h_adc_debug_t *p_handle)
{
    memcpy(p_handle, &h_adc_debug, sizeof(h_adc_debug_t));
}

bool adc_en_set(adc_hal_seq_t seq, bool b_enable)
{
    if (seq >= ADC_HAL_SEQ_NUM)
        return false;
    if (h_adc_debug.b_enable)
    {
        printf("> adc enable:set,%d,%d\r\n", seq, b_enable);
    }
    HAL_StatusTypeDef res;
    if (b_enable)
    {
        __HAL_ADC_ENABLE(p_h_hal_adc[seq]);
        res = HAL_TIM_Base_Start(p_h_hal_timer[seq]);
    }
    else
    {
        res = HAL_TIM_Base_Stop(p_h_hal_timer[seq]);
        __HAL_ADC_DISABLE(p_h_hal_adc[seq]);
    }
    if (h_adc_debug.b_enable)
    {
        printf("> adc enable:res,%d\r\n", res);
    }
    return res == HAL_OK;
}

bool adc_sample_rate_set(adc_hal_seq_t seq, double sample_rate)
{
    if (seq >= ADC_HAL_SEQ_NUM)
        return false;
    if (h_adc_debug.b_sample_set)
    {
        printf("> ADC sample_rate:set,%f\r\n", sample_rate);
    }
    TIM_HandleTypeDef *p_h_hal     = p_h_hal_timer[seq];
    uint32_t           period_tick = clk_source_get[seq]() / sample_rate;
    uint32_t           prescaler   = (period_tick + UINT16_MAX) / (UINT16_MAX + 1);
    uint32_t           reload      = period_tick / prescaler;
    __HAL_TIM_SET_PRESCALER(p_h_hal, prescaler - 1);
    __HAL_TIM_SET_AUTORELOAD(p_h_hal, reload - 1);

    if (h_adc_debug.b_sample_set)
    {
        printf("> adc sample_rate:finish");
        printf(",%d", period_tick);
        printf(",%d", prescaler);
        printf(",%d", reload);
        printf("\r\n");
    }
    return true;
}

double adc_value_get(adc_seq_t sequence)
{
    if (sequence >= ADC_SEQ_NUM)
        return 0;
    return h_avg[sequence].avg;
}

double adc_volt_get(adc_seq_t sequence)
{
    if (sequence >= ADC_SEQ_NUM)
        return 0;
    return adc_volt_coeffi[sequence] * h_avg[sequence].avg;
}

uint16_t adc_value_org_get(adc_seq_t sequence)
{
    return adc_buff[sequence];
}

static void moving_avg_u16_update(h_moving_avg_u16_t *p_h_avg, uint16_t new_data)
{
    if (p_h_avg->p_buff == NULL)
        return;
    p_h_avg->data_n %= p_h_avg->buff_len;
    p_h_avg->p_buff[p_h_avg->data_n] = new_data;
    p_h_avg->data_n                  = (p_h_avg->data_n + 1) % p_h_avg->buff_len;
    p_h_avg->sum += new_data;
    if (p_h_avg->data_len < p_h_avg->buff_len - 1)
    {
        p_h_avg->data_len++;
    }
    else
    {
        p_h_avg->sum -= p_h_avg->p_buff[p_h_avg->data_n];
        p_h_avg->data_len = p_h_avg->buff_len - 1;
    }
    p_h_avg->avg = p_h_avg->sum / p_h_avg->data_len;
}

static void moving_avg_u16_reset(h_moving_avg_u16_t *p_h_avg)
{
    memset(p_h_avg->p_buff, 0x0, sizeof(*p_h_avg->p_buff) * p_h_avg->buff_len);
    p_h_avg->data_len = 0;
    p_h_avg->data_n   = 0;
    p_h_avg->sum      = 0;
    p_h_avg->avg      = 0;
}
