#include "driver/uart/uart_drv.h"

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


void uart_init(void)    //9600bps@22.1184MHz
{
	//uart1硬件初始化
	SCON = 0x50;		//8位数据,可变波特率,接收使能
	PCON &= 0x3F;		//波特率不倍速
	AUXR &= 0xFB;		//独立波特率发生器时钟为Fosc/12,即12T
	BRT = 0xFA;		    //设定独立波特率发生器重装值
	AUXR |= 0x01;		//串口1选择独立波特率发生器为波特率发生器
	AUXR |= 0x10;		//启动独立波特率发生器
	ES = 1;             //使能串口1中断
	PS = 1;             //设置中断优先级为优先级1
	IPH &= 0xEF;

	_nop_();
	_nop_();
	_nop_();
	_nop_();
}

//上位机
void uart1_isr(void) interrupt SIO_VECTOR using 2
{
	Byte c;

	if (_testbit_(TI)) { //发送中断
		trans1_ctr++;   //取下一个待传送index
		if (trans1_ctr < trans1_size) { //未传送完成
			if (trans1_ctr == (trans1_size - 1)) { //已经指向校验字节
				SBUF = trans1_chksum;    //发送校验字节
			} else { //非校验字节, 需要传送并计算checksum
				SBUF = trans1_buf[trans1_ctr];
				if (trans1_ctr > 0) { //计算check_sum
					trans1_chksum += trans1_buf[trans1_ctr];   //更新chksum
				}
			}
		} else { //已经全部传送完成(含校验字节)，可以置发送器空闲
			//目前设计：均不需等待应答, 可以释放该队列项
			if (uart1_q_index < UART_QUEUE_NUM)
				uart1_q[uart1_q_index].flag = 0;   //该队列项空闲
			uart1_q_index = 0xFF;	//无队列项在发送
			trans1_occupy = 0;		//发送器空闲
		}
		TI = 0;   //must clear by user software
	}

	if (_testbit_(RI)) { //接收中断
		c = SBUF;
		switch (recv1_state)
		{
		case FSA_INIT://是否为帧头
			if (c == FRAME_STX) { //为帧头, 开始新的一帧
				recv1_ctr = 0;
				recv1_chksum = 0;
				recv1_timer = RECV_TIMEOUT;
				recv1_state = FSA_ADDR_D;
			}
			break;

		case FSA_ADDR_D://为目的地址, 开始保存并计算效验和
			recv1_buf[recv1_ctr++] = c;
			recv1_chksum += c;
			recv1_timer = RECV_TIMEOUT;
			recv1_state = FSA_ADDR_S;
			break;

		case FSA_ADDR_S://为源地址
			recv1_buf[recv1_ctr++] = c;
			recv1_chksum += c;
			recv1_timer = RECV_TIMEOUT;
			recv1_state = FSA_LENGTH;
			break;

		case FSA_LENGTH://为长度字节
			if ((c > 0) && (c < (MAX_RecvFrame - 3))) { //有效串
				recv1_buf[recv1_ctr++] = c;    //第三个字节保存长度
				recv1_chksum += c;
				recv1_timer = RECV_TIMEOUT;
				recv1_state = FSA_DATA;
			} else {	//非有效串
				recv1_state = FSA_INIT;
			}
			break;

		case FSA_DATA://读取命令串
			recv1_buf[recv1_ctr] = c;
			recv1_chksum += c;   //更新校验和
			if (recv1_ctr == (recv1_buf[2] + 2)){ //已经收到指定长度的命令数据
				recv1_state = FSA_CHKSUM;
			}else{//还未结束
				recv1_ctr ++;
			}
			recv1_timer = RECV_TIMEOUT;
			break;

		case FSA_CHKSUM://检查校验字节
			if ((recv1_chksum == c) && (msg1_buf_valid == FALSE)){
				//已经收到完整一帧并且以前收到的消息已经被处理了
				memcpy(msg1_buf, recv1_buf, recv1_buf[2] + 3);
				msg1_buf_valid = TRUE;
			}
		default:
			//复位
			recv1_state = FSA_INIT;
			break;
		}
		RI = 0;     //must clear by user software
	}
}

void uart1_start_trans(void)
{ 
	trans1_occupy = 1;
	trans1_chksum = 0;
	trans1_ctr = 0;
	SBUF = trans1_buf[trans1_ctr];
}