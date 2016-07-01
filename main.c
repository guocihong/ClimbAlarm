#include "system_init.h"

/*
1、攀爬报警主程序-->用在受力杆和轴承杆上面
2、出厂需要设置报警阀值
*/

void main(void)
{       	
	//系统初始化
    system_init();

    //打开总中断
    Enable_interrupt();
	
    //使能看门狗
    Wdt_enable();// 2.276s 溢出周期      
    
    while(1){
		//系统状态处理
		status_task();
		
        //处理AD采集的结果
        adc_task();
        
        //解析uart接收到的命令以及主动上报报警信息
        uart_task();
        
		//门磁处理  
		doorkeep_task();
				
		//报警处理
		alarm_task();
				
        //喂狗
        Wdt_refresh();
    }
}