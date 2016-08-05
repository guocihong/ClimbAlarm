#include "task/alarm/alarm_task.h"

/* AD */
extern bdata  bit     ad_sensor_mask;   //����������

/* Doorkeep */
extern bdata  bit
gl_dk_status;     //�Ŵſ���״̬: 1 - �պ�; 0 - ��(��Ҫ����)

/* for system */
extern idata  Byte    system_status;    //ϵͳ״̬

/* variables for alarm output */
extern bdata  bit     ad_alarm_flag;    //����������־
extern bdata  bit     alarm1_flag;
extern data   Uint16  alarm1_timer;     // ��ʱ����������1�ѱ���ʱ��,��λtick

void alarm_task_init(void)
{
    alarm1_flag = 0;
}

void alarm_task(void)
{
    Uint16 temp16;

    //����
    if ((!gl_dk_status || ad_alarm_flag) && (ad_sensor_mask == 1) &&
        (system_status == SYS_CHECK))
    {
        //�ŴŴ򿪻������쳣: �±������������
        Disable_interrupt();
        alarm1_timer = 0;     //���ѱ���ʱ��(���ٱ���3��)
        Enable_interrupt();
        if (alarm1_flag == 0)   //�±���
        {
            bRelay_A1 = 0;     //����
            alarm1_flag = 1;   //������1����
        }
    }

    //��鱨��ʱ���Ƿ��ѵ�
    if (alarm1_flag)   //������1���ڱ���
    {
        Disable_interrupt();
        temp16 = alarm1_timer;
        Enable_interrupt();
        if (temp16 > ALARM_TEMPO)
        {
            //������1�Ѿ�����󱨾�ʱ��, ֹͣ����
            alarm1_flag = 0;
            bRelay_A1 = 1;
        }
    }
}
