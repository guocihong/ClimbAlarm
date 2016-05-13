#ifndef _ADC_TASK_H_
#define _ADC_TASK_H_

#include "task/uart/uart_task.h"
#include "driver/adc/adc_drv.h"
#include "config.h"

void adc_task_init(void);
void adc_task(void);
Byte check_still_stress(Byte index);
void ad_reset_scene(void);

#endif