#include "system_init.h"

/*
1����������������-->���������˺���и�����
2��������Ҫ���ñ�����ֵ
*/

void main(void)
{       	
	//ϵͳ��ʼ��
    system_init();

    //�����ж�
    Enable_interrupt();
	
    //ʹ�ܿ��Ź�
    Wdt_enable();// 2.276s �������      
    
    while(1){
		//ϵͳ״̬����
		status_task();
		
        //����AD�ɼ��Ľ��
        adc_task();
        
        //����uart���յ��������Լ������ϱ�������Ϣ
        uart_task();
        
		//�ŴŴ���  
		doorkeep_task();
				
		//��������
		alarm_task();
				
        //ι��
        Wdt_refresh();
    }
}