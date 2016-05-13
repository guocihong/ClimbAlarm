#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "STC12C5A60S2.h"
#include "compiler.h"
#include <intrins.h>
#include "stress.h"

/* Scheduler Tick */
#define SCHEDULER_TICK     5                        // unit is ms

#define ALARM_TEMPO        (3000/SCHEDULER_TICK)    //报警信号持续时间

/* System status */
#define SYS_PowerON        0      // 0 - 初始上电
#define SYS_B5S            1      // 1 - 基准值采样前延时(约5秒)
#define SYS_SAMP_BASE      2      // 2 - 基准值采样(10秒左右)
#define SYS_CHECK          3      // 3 - 实时监测

/* AD */
typedef struct strAD_Sample
{ //每点采样值
  Uint16   val;     //当前采样值
  Uint8    index;   //通道号，范围0 ~ 13
  Byte     valid;   //采样数据处理标志: 0 - 已处理，可以写入新值; 1 - 新值，等待处理                                    
}sAD_Sample;

typedef struct strAD_Sum
{ //采样值累加和
  Uint16   sum;     //累计和 (最多达64点,不会溢出)
  Uint8    point;   //已采样点数
}sAD_Sum;

typedef struct strAD_BASE
{ //系统运行时静态基准值对应的采样值
  Uint16   base;       //静态基准值
  Uint16   base_down;  //基准值下限(含)
  Uint16   base_up;    //基准值上限(含)
}sAD_BASE;

//for Uart
#define	FRAME_STX           0x16          // Frame header
#define	MAX_RecvFrame       50            // 接收缓存区大小
#define	MAX_TransFrame      50            // 发送缓存区大小
#define RECV_TIMEOUT        4            // 字节间的最大时间间隔, 单位为tick
                                            // 最小值可以为1, 如果为0则表示不进行超时判定                                    
/* state constant(仅用于接收) */
#define FSA_INIT            0            //等待帧头
#define FSA_ADDR_D          1            //等待目的地址
#define FSA_ADDR_S          2            //等待源地址
#define FSA_LENGTH          3            //等待长度字节
#define FSA_DATA            4            //等待命令串(包括 命令ID 及 参数)
#define FSA_CHKSUM          5            //等待校验和

/* Uart Queue */
typedef struct strUART_Q
{
  Byte  flag;                               //状态： 0 - 空闲； 1 - 等待发送； 2 - 正在发送; 3 - 已发送，等待应答
  Byte  tdata[MAX_TransFrame];              //数据包(最后一个校验字节可以不提前计算，而在发送时边发送边计算)
  Byte  len;					            //数据包有效长度(含校验字节)
}sUART_Q;

#define UART_QUEUE_NUM      4            //UART 队列数, 用于上位机

#define bDoorKeeper         P46          //高阻输入，门磁检测: 1-门磁关闭; 0-门磁打开（应报警）
#define bRelay_A1           P34		   //推挽输出，(左侧)报警输出1：1-继电器加电吸合(上电缺省)； 0-不加电

/* interrupt enable */
#define Enable_interrupt()  (EA = 1)
#define Disable_interrupt() (EA = 0)

#endif