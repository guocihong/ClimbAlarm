#include "config.h"

/* UART1 */		
xdata  Byte     msg1_buf[MAX_RecvFrame];     // received message, used for proceding
bdata  bit      msg1_buf_valid;	             // received valid flag
xdata  Byte     recv1_buf[MAX_RecvFrame];    // receiving buffer
idata  Byte     recv1_state;                 // receive state
idata  Byte     recv1_timer;                 // receive time-out, 用于字节间超时判定
idata  Byte     recv1_chksum;                // computed checksum
idata  Byte     recv1_ctr;                   // reveiving pointer

xdata  Byte     trans1_buf[MAX_TransFrame];  // uart transfer message buffer
idata  Byte     trans1_ctr;                  // transfer pointer
idata  Byte     trans1_size;                 // transfer bytes number
idata  Byte     trans1_chksum;               // computed check-sum of already transfered message
bdata  bit      trans1_occupy;               // 发送器被占用标志，1-被占用, 0-空闲

data  Byte      uart1_q_index;               // 正在发送某队列项的序号：若为0xFF, 表示没有任何项进入发送流程
xdata sUART_Q   uart1_q[UART_QUEUE_NUM];	 // 串口队列				 						  								   

/* AD sample */
bdata  bit         ad_sensor_mask;           //传感器掩码
 data  sAD_Sample  ad_sample;                //保存当前采样值
idata  sAD_Sum     ad_samp_equ;              //均衡去嘈声求和
xdata  Union16     ad_chn_sample;            //最新一轮采样值（已均衡去噪声，每通道一个点，循环保存）
idata  Uint16      ad_samp_pnum;             //采样点数(计算静态基准值时总采样点数)
idata  sAD_Sum     ad_samp_sum;              //阶段求和
xdata  sAD_BASE    ad_chn_base;              //各通道运行时静态基准值/上下限阀值（单位：采样值）
 data  Byte        ad_chn_over;              //各通道连续采样点(均衡后)的阀值判定： 0 - 范围内； 1 - 超阀值
xdata  Uint16      ad_still_dn;              //静态拉力值下限
xdata  Uint16      ad_still_up;              //静态拉力值上限
 xdata  Byte       ad_still_Dup;             //报警阀值上限
idata  Uint16      ad_alarm_exts;            //外力报警标志（未经mask）：位值 0 - 无； 1 - 超阀值                    

/* for system */
idata  Byte     system_status;               //系统状态
										 	 // 0 - 初始上电
											 // 1 - 基准值采样前延时
											 // 2 - 基准值采样(10秒左右)
											 // 3 - 实时监测
idata  Byte     gl_comm_addr;                //本模块485通信地址
bdata  bit      system_2or1;                 //双/单防区标志: 0 - 双; 1 - 单		

/* 系统计时 */
xdata  Byte     gl_ack_tick = 0;	         //上位机485口应答延时计时 tick
xdata  Uint16   gl_delay_tick;               //通用延时用tick
 
/* variables for alarm output */
bdata  bit      ad_alarm_flag;               //外力报警标志
 data  Uint16   ad_alarm_tick;               //各通道报警计时tick
bdata  bit      alarm1_flag;
 data  Uint16   alarm1_timer;                //计时器，报警器1已报警时间,单位:tick 

/* variables for beep */
xdata  Uint16   beep_during_temp;            //预设的一次蜂鸣持续时间, 单位:tick 

/* Doorkeep(门磁) */
bdata  bit      gl_dk_status;                //门磁开关状态（每1s动态检测）: 1 - 闭合; 0 - 打开(需要报警)                    
xdata  Byte     gl_dk_tick;  	             //门磁检测计时tick

/* 传感器采样偏差 */
xdata  Uint16   sensor_sample_offset;        //传感器采样偏差：没有外力时，传感器采样值不为0，大约400左右，需要矫正。瞬间张力 = 采样值 - 采样偏差
									         