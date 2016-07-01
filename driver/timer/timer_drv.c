#include "driver/timer/timer_drv.h"
#include "driver/adc/adc_drv.h"

/* UART1 */
extern idata  Byte        recv1_state;                 // receive state
extern idata  Byte        recv1_timer;                 // receive time-out, 用于字节间超时判定

/* for AD */
extern data   sAD_Sample  ad_sample;                   // 保存当前采样值

/* Doorkeep */   
extern xdata  Byte        gl_dk_tick;                  //门磁检测计时tick  

/* 系统计时 */
extern xdata  Uint16      gl_delay_tick;               //通用延时用tick
extern xdata  Byte        gl_ack_tick;	               //应答延时计时 tick


/* variables for alarm output */ 
extern bdata  bit         alarm1_flag;
extern data   Uint16      alarm1_timer;                // 计时器，报警器1已报警时间,单位tick 
extern data   Uint16      ad_alarm_tick;               //各通道报警计时tick
 
/* 传感器采样偏差 */
extern xdata  Uint16      sensor_sample_offset;        //传感器采样偏差：没有外力时，传感器采样值不为0，大约400左右，需要矫正。瞬间张力 = 采样值 - 采样偏差

void timer0_init(void)   // 5ms@22.1184MHz
{    
    // 定时器0初始化
	AUXR &= 0x7F;		 // 设置为12T模式
	TMOD &= 0xF0;		 // 设置为工作模式1
	TMOD |= 0x01;
	TL0 = 0x00;		     // 设置定时初值
	TH0 = 0xDC;		     // 设置定时初值
	TF0 = 0;		     // 清除TF0标志
    ET0 = 1;             // 使能T0中断允许位
    IPH |= (1 << 1);
    PT0 = 0;             // 设置中断优先级为优先级2
	TR0 = 1;		     // 定时器0开始计时
	
	// 启动AD转换
	ADC_CONTR |= ADC_START;
}

void timer0_isr(void) interrupt TF0_VECTOR using 1
{	               
    // 重装初值
	TL0 = 0x00;		     // 设置定时初值
	TH0 = 0xDC;		     // 设置定时初值
	TR0 = 1;		     // 定时器0开始计时
        
    // AD转换完成,将ADC_FLAG转换完成标志清零
    ADC_CONTR &= ~ADC_FLAG;

	// 读AD值
	if (ad_sample.valid == FALSE) {
		// 原数据已经被处理, 可以写入新数据
		ad_sample.val   = ADC_RES;             // 读高8位
		ad_sample.val   = ad_sample.val << 2;
		ad_sample.val   += (ADC_RESL & 0x03);  // 得到10bit采样值
		ad_sample.valid = TRUE;
		
		//采样值减去采样偏差
/*  		if (ad_sample.val > sensor_sample_offset) {
			ad_sample.val -= sensor_sample_offset;
		} else {
			ad_sample.val = 0;
		}  */
		
		ADC_CONTR |= ADC_START;                   // 启动转换
	}

	// increment task tick counters
	gl_dk_tick++;                                 //门磁检测计时tick
	gl_delay_tick++;                              //通用延时用tick
	if (gl_ack_tick > 0) {
		gl_ack_tick--;                            //应答延时计时
	}

	ad_alarm_tick++;
	
	if (alarm1_flag) {
		alarm1_timer++;
	}
 
	// UART1字节之间接收超时
	if (recv1_state != FSA_INIT) {
		//非初始状态，需要检测是否超时
		if (recv1_timer > 0) {
			recv1_timer--;
		}
		
		if (recv1_timer == 0){
			recv1_state = FSA_INIT;   //接收超时, 恢复至初始状态
		}
	}
}