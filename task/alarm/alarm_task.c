#include "task/alarm/alarm_task.h"

/* AD */
extern bdata  bit     ad_sensor_mask;   //传感器掩码

/* Doorkeep */
extern bdata  bit
gl_dk_status;     //门磁开关状态: 1 - 闭合; 0 - 打开(需要报警)

/* for system */
extern idata  Byte    system_status;    //系统状态

/* variables for alarm output */
extern bdata  bit     ad_alarm_flag;    //外力报警标志
extern bdata  bit     alarm1_flag;
extern data   Uint16  alarm1_timer;     // 计时器，报警器1已报警时间,单位tick

void alarm_task_init(void)
{
    alarm1_flag = 0;
}

void alarm_task(void)
{
    Uint16 temp16;

    //报警
    if ((!gl_dk_status || ad_alarm_flag) && (ad_sensor_mask == 1) &&
        (system_status == SYS_CHECK))
    {
        //门磁打开或张力异常: 新报警或继续报警
        Disable_interrupt();
        alarm1_timer = 0;     //清已报警时间(至少报警3秒)
        Enable_interrupt();
        if (alarm1_flag == 0)   //新报警
        {
            bRelay_A1 = 0;     //报警
            alarm1_flag = 1;   //报警器1报警
        }
    }

    //检查报警时间是否已到
    if (alarm1_flag)   //报警口1正在报警
    {
        Disable_interrupt();
        temp16 = alarm1_timer;
        Enable_interrupt();
        if (temp16 > ALARM_TEMPO)
        {
            //报警口1已经到最大报警时间, 停止报警
            alarm1_flag = 0;
            bRelay_A1 = 1;
        }
    }
}
