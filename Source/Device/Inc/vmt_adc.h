#ifndef __VMT_ADC_H__
#define __VMT_ADC_H__

#include <stdint.h>
#include <stdbool.h>

/* define */
#define ADC_VER_MAIN (0)
#define ADC_VER_SUB  (0)

// #define ADC_VDD_VOLT (3.3)
#define ADC_VDD_VOLT (0.808 * (1.1 + 37.4 / 12.1))

/* type define */
typedef enum adc_hal_seq_
{
    ADC_HAL_SEQ_FIRST,
    ADC_HAL_SEQ_TEMP = ADC_HAL_SEQ_FIRST,
    ADC_HAL_SEQ_WATER,
    ADC_HAL_SEQ_NUM,
} adc_hal_seq_t;
// ADC
#define ADC_SEQ_EXT_NUM   (4) // external signal number
#define ADC_SEQ_DIST_NUM  (3) // distance sensor number
#define ADC_SEQ_WATER_NUM (2)

typedef enum adc_seq_
{
    ADC_SEQ_BEGIN,

    ADC_SEQ_TEMP = ADC_SEQ_BEGIN,
    ADC_SEQ_WATER_BEGIN,
    ADC_SEQ_WATER_END = ADC_SEQ_WATER_BEGIN + ADC_SEQ_WATER_NUM - 1,

    ADC_SEQ_NUM,
} adc_seq_t;

/* Average number of samples */
#define ADC_AVG_NUM_TEMP       (40)
#define ADC_AVG_NUM_WATER      (40)
#define ADC_AVG_BUFF_LEN_TEMP  (ADC_AVG_NUM_TEMP + 1)
#define ADC_AVG_BUFF_LEN_WATER (ADC_AVG_NUM_TEMP + 1)

typedef struct h_adc_debug_
{
    bool b_init : 1, b_enable : 1, b_sample_set : 1;
} h_adc_debug_t;

/* function declaration */
void adc_sample_finish_cb(adc_seq_t sequence);

bool adc_init(adc_hal_seq_t seq);
void adc_deinit(adc_hal_seq_t seq);

void adc_debug_set(h_adc_debug_t *p_handle);
void adc_debug_get(h_adc_debug_t *p_handle);

bool adc_sample_rate_set(adc_hal_seq_t seq, double sample_rate);
bool adc_en_set(adc_hal_seq_t seq, bool b_enable);

double   adc_value_get(adc_seq_t sequence); // get value
double   adc_volt_get(adc_seq_t sequence);  // get voltage
uint16_t adc_value_org_get(adc_seq_t sequence);

#endif //__VMT_ADC_H__
