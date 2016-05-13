#include "task/status/status_task.h"

#define DLY_BF_GetBase  (5000/SCHEDULER_TICK)   //基准值采样前延时，单位：tick

/* 系统计时 */
extern xdata  Uint16  gl_delay_tick;   //通用延时用tick

/* for system */
extern idata  Byte    system_status;   //系统状态

void status_task_init(void)
{
	system_status = SYS_PowerON;       //初始上电
}

void status_task(void)
{
	Uint16 temp16;

	switch (system_status)
	{
	case  SYS_PowerON://初始上电
		Disable_interrupt();
		gl_delay_tick = 0;
		Enable_interrupt();
		system_status = SYS_B5S;
		break;

	case  SYS_B5S:    //基准值采样前延时(约5秒)
		Disable_interrupt();
		temp16 = gl_delay_tick;
		Enable_interrupt();
		if (temp16 > DLY_BF_GetBase)
			system_status = SYS_SAMP_BASE;
		break;
	default:
		break;
	}
}
