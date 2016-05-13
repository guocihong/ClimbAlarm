#include "task/status/status_task.h"

#define DLY_BF_GetBase  (5000/SCHEDULER_TICK)   //��׼ֵ����ǰ��ʱ����λ��tick

/* ϵͳ��ʱ */
extern xdata  Uint16  gl_delay_tick;   //ͨ����ʱ��tick

/* for system */
extern idata  Byte    system_status;   //ϵͳ״̬

void status_task_init(void)
{
	system_status = SYS_PowerON;       //��ʼ�ϵ�
}

void status_task(void)
{
	Uint16 temp16;

	switch (system_status)
	{
	case  SYS_PowerON://��ʼ�ϵ�
		Disable_interrupt();
		gl_delay_tick = 0;
		Enable_interrupt();
		system_status = SYS_B5S;
		break;

	case  SYS_B5S:    //��׼ֵ����ǰ��ʱ(Լ5��)
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