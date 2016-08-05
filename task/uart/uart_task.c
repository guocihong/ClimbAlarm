#include "task/uart/uart_task.h"

#define REPLY_DLY   (10/SCHEDULER_TICK)             //收到PC命令后的应答延时

/* 系统计时 */
extern xdata  Byte     gl_ack_tick;	                //应答延时计时 tick

/* UART1 */
extern xdata  Byte     msg1_buf[MAX_RecvFrame];     // received message, used for proceding
extern bdata  bit      msg1_buf_valid;	            // received valid flag
extern xdata  Byte     recv1_buf[MAX_RecvFrame];    // receiving buffer
extern idata  Byte     recv1_state;                 // receive state
extern idata  Byte     recv1_timer;                 // receive time-out, 用于字节间超时判定
extern idata  Byte     recv1_chksum;                // computed checksum
extern idata  Byte     recv1_ctr;                   // reveiving pointer

extern xdata  Byte     trans1_buf[MAX_TransFrame];  // uart transfer message buffer
extern idata  Byte     trans1_ctr;                  // transfer pointer
extern idata  Byte     trans1_size;                 // transfer bytes number
extern idata  Byte     trans1_chksum;               // computed check-sum of already transfered message
extern bdata  bit      trans1_occupy;               // 发送器被占用标志，1-被占用, 0-空闲

extern  data  Byte     uart1_q_index;               // 正在发送某队列项的序号：若为0xFF, 表示没有任何项进入发送流程
extern xdata  sUART_Q  uart1_q[UART_QUEUE_NUM];	    // 串口队列

/* for system */
extern idata  Byte     system_status;               //系统状态
extern bdata  bit      system_2or1;                 //双/单防区标志: 0 - 双; 1 - 单
extern idata  Byte     gl_comm_addr;                //本模块485通信地址

/* variables for alarm output */
extern bdata  bit      ad_alarm_flag;               //外力报警标志

/* variables for beep */
extern xdata  Uint16   beep_during_temp;            //预设的一次蜂鸣持续时间, 单位:tick 

/* Doorkeep(门磁) */
extern bdata  bit      gl_dk_status;                //门磁开关状态（每1s动态检测）: 1 - 闭合; 0 - 打开(需要报警)

/* AD sample */
extern xdata  Union16  ad_chn_sample;               //最新一轮采样值（已均衡去噪声，每通道一个点，循环保存）
extern xdata  sAD_BASE ad_chn_base;                 //各通道运行时静态基准值/上下限阀值（单位：采样值）
extern xdata  Uint16   ad_still_dn;                 //静态拉力值下限
extern xdata  Uint16   ad_still_up;                 //静态拉力值上限
extern xdata  Byte     ad_still_Dup;                //报警阀值上限

/* 传感器采样偏差 */
extern xdata  Uint16   sensor_sample_offset;        //传感器采样偏差：没有外力时，传感器采样值不为0，大约400左右，需要矫正。瞬间张力 = 采样值 - 采样偏差
                                                    

Byte check_still_stress(Byte index);

void uart_task_init(void)
{
	Byte i;

	//uart1相关变量初始化
	msg1_buf_valid = FALSE;
	recv1_state = FSA_INIT;
	recv1_timer = 0;
	recv1_ctr = 0;
	recv1_chksum = 0;
	trans1_size = 0;
	trans1_ctr = 0;
	trans1_occupy = 0;       //空闲

	for (i = 0; i < UART_QUEUE_NUM; i++){
		uart1_q[i].flag = 0; //均空闲
	}
	uart1_q_index = 0xFF;    //无队列项进入发送流程

	//UART硬件初始化
	uart_init();             //之后，已经准备好串口收发，只是还未使能全局中断
}

void uart_task(void)
{
	Byte   i, j;
	Uint16 temp16;

	//1.接收并处理 UART1：来自上位机的命令包
	if (msg1_buf_valid) {
		//a.1 是否需要执行本命令
		if ((msg1_buf[0] == CMD_ADDR_BC) || (msg1_buf[0] == gl_comm_addr)) {
			//广播地址或指定本设备, 需要执行
			switch (msg1_buf[3])
			{
			case CMD_ZL_PRE://张力/脉冲专用命令标志
				switch (msg1_buf[5])
				{
				case 0x10: //读配置参数
					//在UART队列中找空闲Buffer
					i = uart1_get_buffer();
					if (i < UART_QUEUE_NUM) {
						//找到了空闲buffer, 写入data
						uart1_q[i].tdata[0]  = FRAME_STX;
						uart1_q[i].tdata[1]  = msg1_buf[1];	             //目的地址
						uart1_q[i].tdata[2]  = gl_comm_addr;	         //源地址
						uart1_q[i].tdata[3]  = 0x1E;                     //命令长度
						uart1_q[i].tdata[4]  = CMD_ZL_PRE;		         //命令ID
						uart1_q[i].tdata[5]  = 0x1C;                     //参数1
						uart1_q[i].tdata[6]  = 0x08;                     //参数2
						uart1_q[i].tdata[7]  = 0x00;  //张力掩码左
						uart1_q[i].tdata[8]  = 0x00;   //张力掩码右
						uart1_q[i].tdata[9]  = HIGH(ad_still_dn);        //静态张力允许下限高
						uart1_q[i].tdata[10] = LOW(ad_still_dn);         //静态张力允许下限低
						uart1_q[i].tdata[11] = HIGH(ad_still_up);        //静态张力允许上限高
						uart1_q[i].tdata[12] = LOW(ad_still_up);         //静态张力允许上限低
						uart1_q[i].tdata[13] = 66;                       //报警阀值下浮比例，固定为66
						for (j = 0; j < 7; j++) {                        //报警阀值允许上浮左1~8
							uart1_q[i].tdata[14 + j] = ad_still_Dup;
						}

						uart1_q[i].tdata[21] = 0;
						for (j = 0; j < 7; j++) {                        //报警阀值允许上浮右1~8
							uart1_q[i].tdata[22 + j] = ad_still_Dup;
						}
						uart1_q[i].tdata[29] = 0;
						uart1_q[i].tdata[30] = system_2or1;              //双/单防区
						uart1_q[i].tdata[31] = gl_comm_addr;             //拨码地址
						uart1_q[i].tdata[32] = (Byte)(((Uint32)beep_during_temp * SCHEDULER_TICK) / 1000);	//声光报警输出时间
						uart1_q[i].tdata[33] = 0;                        //联动输出时间
						uart1_q[i].len = 35;
					} else {
						//无空闲buffer, 丢弃本命令
						//检查: 若有队列项正在发送, 等待它完成
						while (uart1_q_index != 0xFF);	//若死锁,将引起 WDT 复位
					}
					break;

				case 0x12: //读报警信息
					//在UART队列中找空闲Buffer
					i = uart1_get_buffer();
					if (i < UART_QUEUE_NUM) {
						//找到了空闲buffer, 写入data
						uart1_q[i].tdata[0]  = FRAME_STX;
						uart1_q[i].tdata[1]  = msg1_buf[1];	            //目的地址
						uart1_q[i].tdata[2]  = gl_comm_addr;	        //源地址
						uart1_q[i].tdata[3]  = 0x0A;                    //命令长度
						uart1_q[i].tdata[4]  = CMD_ZL_PRE;              //命令ID
						uart1_q[i].tdata[5]  = 0x08;                    //参数1
						uart1_q[i].tdata[6]  = 0x1A;                    //参数1
						uart1_q[i].tdata[7]  = 0x00;                    //张力掩码左
						uart1_q[i].tdata[8]  = 0x00;                    //张力掩码右

						uart1_q[i].tdata[9]  = (Byte)ad_alarm_flag;     //外力报警左
						uart1_q[i].tdata[10] = 0x00;                    //外力报警右

						uart1_q[i].tdata[11] = 0x00;                    //静态张力报警左
						uart1_q[i].tdata[12] = 0x00;                    //静态张力报警右
						
						uart1_q[i].tdata[13] = (Byte)(!gl_dk_status);   //门磁
						uart1_q[i].len = 15;
					} else {
						//无空闲buffer, 丢弃本命令
						//检查: 若有队列项正在发送, 等待它完成
						while (uart1_q_index != 0xFF);	//若死锁,将引起 WDT 复位
					}
					break;

				case 0x14: //读瞬间张力
					//在UART队列中找空闲Buffer
					i = uart1_get_buffer();
					if (i < UART_QUEUE_NUM) {
						//找到了空闲buffer, 写入data
						uart1_q[i].tdata[0] = FRAME_STX;
						uart1_q[i].tdata[1] = msg1_buf[1];	    //目的地址
						uart1_q[i].tdata[2] = gl_comm_addr;	    //源地址
						uart1_q[i].tdata[3] = 0x23;
						uart1_q[i].tdata[4] = CMD_ZL_PRE;
						uart1_q[i].tdata[5] = 0x21;
						uart1_q[i].tdata[6] = 0x1C;
						
						//左1
						temp16 = ad_chn_sample.w;
						uart1_q[i].tdata[7] = HIGH(temp16);
						uart1_q[i].tdata[8] = LOW(temp16);
							
						for (j = 1; j < 8; j++) { //左2~8
							uart1_q[i].tdata[7 + (j << 1)] = 0x00;
							uart1_q[i].tdata[8 + (j << 1)] = 0x00;
						}

						
						for (j = 0; j < 8; j++) { //右1~8
							uart1_q[i].tdata[23 + (j << 1)] = 0x00;
							uart1_q[i].tdata[24 + (j << 1)] = 0x00;
						}

						uart1_q[i].len = 40;
					} else {
						//无空闲buffer, 丢弃本命令
						//检查: 若有队列项正在发送, 等待它完成
						while (uart1_q_index != 0xFF);	//若死锁,将引起 WDT 复位
					}
					break;

				case 0x15: //读静态张力基准
					//在UART队列中找空闲Buffer
					i = uart1_get_buffer();
					if (i < UART_QUEUE_NUM) {
						//找到了空闲buffer, 写入data
						uart1_q[i].tdata[0] = FRAME_STX;
						uart1_q[i].tdata[1] = msg1_buf[1];	    //目的地址
						uart1_q[i].tdata[2] = gl_comm_addr;	    //源地址
						uart1_q[i].tdata[3] = 0x23;
						uart1_q[i].tdata[4] = CMD_ZL_PRE;
						uart1_q[i].tdata[5] = 0x21;
						uart1_q[i].tdata[6] = 0x1D;
						
						//左1
						temp16 = ad_chn_base.base;
						uart1_q[i].tdata[7] = HIGH(temp16);
						uart1_q[i].tdata[8] = LOW(temp16);
							
						for (j = 1; j < 8; j++) { //左2~8
							uart1_q[i].tdata[7 + (j << 1)] = 0x00;
							uart1_q[i].tdata[8 + (j << 1)] = 0x00;
						}

						for (j = 0; j < 8; j++) { //右1~8
							uart1_q[i].tdata[23 + (j << 1)] = 0x00;
							uart1_q[i].tdata[24 + (j << 1)] = 0x00;
						}

						uart1_q[i].len = 40;
					} else {
						//无空闲buffer, 丢弃本命令
						//检查: 若有队列项正在发送, 等待它完成
						while (uart1_q_index != 0xFF);	//若死锁,将引起 WDT 复位
					}
					break;

				case 0x40: //设置静态张力值范围
					//1. 写入flash
					flash_enable();
					flash_erase(EEPROM_SECTOR3);
					flash_write(msg1_buf[6], EEPROM_SECTOR3 + 1);
					flash_write(msg1_buf[7], EEPROM_SECTOR3 + 2);
					flash_write(msg1_buf[8], EEPROM_SECTOR3 + 3);
					flash_write(msg1_buf[9], EEPROM_SECTOR3 + 4);
					flash_write(0x5a, EEPROM_SECTOR3);
					flash_disable();

					//2. 更新变量
					ad_still_dn = ((Uint16)msg1_buf[6] << 8) + msg1_buf[7];	 //下限
					ad_still_up = ((Uint16)msg1_buf[8] << 8) + msg1_buf[9];	 //上限

					break;
					
				case 0x50: //设置报警阀值
					//1. 写入flash并更新变量
					flash_enable();
					flash_erase(EEPROM_SECTOR4);

					ad_still_Dup = msg1_buf[7];
					flash_write(msg1_buf[7], EEPROM_SECTOR4 + 1);

					flash_write(0x5a, EEPROM_SECTOR4);
					flash_disable();

					//下限固定取基准值的 1/3
					//2. 更新换算后的张力报警上限(采样值)
					if (system_status == SYS_CHECK) {
						//已开始运行检测
						if ((1023 - ad_chn_base.base) > ad_still_Dup)
							ad_chn_base.base_up = ad_chn_base.base + ad_still_Dup;
						else
							ad_chn_base.base_up = 1023;
					}
					break;

				case 0x60: //设置声光报警时间
					//1. 写入flash
					flash_enable();
					flash_erase(EEPROM_SECTOR5);
					flash_write(msg1_buf[6], EEPROM_SECTOR5 + 1);
					flash_write(0x5a, EEPROM_SECTOR5);
					flash_disable();

					//2. 更新变量
					beep_during_temp = (Uint16)(((Uint32)msg1_buf[6] * 1000) / SCHEDULER_TICK);
					break;
					
				case 0xF1: //设置传感器采样偏差---->消除电路上的误差                  
					//1. 写入flash并更新变量
					flash_enable();
					flash_erase(EEPROM_SECTOR6);

                    sensor_sample_offset = ((Uint16)msg1_buf[6] << 8) + msg1_buf[7];
					flash_write(msg1_buf[6], EEPROM_SECTOR6 + 1);
					flash_write(msg1_buf[7], EEPROM_SECTOR6 + 2);
						
					flash_write(0x5a, EEPROM_SECTOR6);
					flash_disable(); 

					break;
				}
				break;
			}
		}

		//a.3 设置应答延时
		Disable_interrupt();
		gl_ack_tick = REPLY_DLY;
		Enable_interrupt();

		//a.4 复位标志
		msg1_buf_valid = FALSE;
	}

	//3. UART1 队列发送
	if ((uart1_q_index == 0xFF) && (recv1_state == FSA_INIT) && (gl_ack_tick == 0)) {
		//UART1无进入发送流程的队列项, 找是否有等待发送的项
		for (i = 0; i < UART_QUEUE_NUM; i++) {
			if (uart1_q[i].flag == 1) {
				//有等待发送的项，安排此项发送
				uart1_q[i].flag = 2;
				uart1_q_index = i;
				memcpy(trans1_buf, uart1_q[i].tdata, uart1_q[i].len - 1);
				trans1_size = uart1_q[i].len;
				uart1_start_trans();
				break;
			}
		}
	}
}

/***************************************************************************
* NAME: uart1_get_buffer
*----------------------------------------------------------------------------
* PARAMS:
* return: Byte
*         若返回值 >= UART_QUEUE_NUM, 则表示没有申请到空闲buffer
*----------------------------------------------------------------------------
* PURPOSE: 在串口队列中寻找空闲队列项，若找到，返回队列项序号(0 ~ (UART_QUEUE_NUM-1))
*----------------------------------------------------------------------------
* REQUIREMENTS:
*****************************************************************************/
Byte uart1_get_buffer(void)
{
	Byte i, flag;

	for (i = 0; i < UART_QUEUE_NUM; i++) {
		Disable_interrupt();
		flag = uart1_q[i].flag;
		Enable_interrupt();
		if (flag == 0) { //已找到空闲Buffer
			uart1_q[i].flag = 1;
			break;
		}
	}
	return i;
}