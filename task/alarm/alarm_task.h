#ifndef _ALARM_TASK_H_
#define _ALARM_TASK_H_

#include "config.h"

#define ALARM_TASK_START   0
#define ALARM_TASK_IDLE    1

void  alarm_task_init(void);
void  alarm_task(void);

#endif
