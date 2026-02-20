#include <stdint.h>
#include <stdbool.h>
#include "vmt_adc.h"
#include "vmt_device.h"

bool     water_detect(void);
void     water_reset(void);
void     multiplier_set(float multiplier);
void     reserve_set(uint16_t reserve);
uint16_t reserve_get(void);
bool     water_get(void);

extern bool          adc_print;
extern uart_select_t water_ch;
