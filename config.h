#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "STC12C5A60S2.h"
#include "compiler.h"
#include <intrins.h>
#include "stress.h"

/* Scheduler Tick */
#define SCHEDULER_TICK     5                        // unit is ms

#define ALARM_TEMPO        (3000/SCHEDULER_TICK)    //�����źų���ʱ��

/* System status */
#define SYS_PowerON        0      // 0 - ��ʼ�ϵ�
#define SYS_B5S            1      // 1 - ��׼ֵ����ǰ��ʱ(Լ5��)
#define SYS_SAMP_BASE      2      // 2 - ��׼ֵ����(10������)
#define SYS_CHECK          3      // 3 - ʵʱ���

/* AD */
typedef struct strAD_Sample
{ //ÿ�����ֵ
  Uint16   val;     //��ǰ����ֵ
  Uint8    index;   //ͨ���ţ���Χ0 ~ 13
  Byte     valid;   //�������ݴ����־: 0 - �Ѵ�������д����ֵ; 1 - ��ֵ���ȴ�����                                    
}sAD_Sample;

typedef struct strAD_Sum
{ //����ֵ�ۼӺ�
  Uint16   sum;     //�ۼƺ� (����64��,�������)
  Uint8    point;   //�Ѳ�������
}sAD_Sum;

typedef struct strAD_BASE
{ //ϵͳ����ʱ��̬��׼ֵ��Ӧ�Ĳ���ֵ
  Uint16   base;       //��̬��׼ֵ
  Uint16   base_down;  //��׼ֵ����(��)
  Uint16   base_up;    //��׼ֵ����(��)
}sAD_BASE;

//for Uart
#define	FRAME_STX           0x16          // Frame header
#define	MAX_RecvFrame       50            // ���ջ�������С
#define	MAX_TransFrame      50            // ���ͻ�������С
#define RECV_TIMEOUT        4            // �ֽڼ�����ʱ����, ��λΪtick
                                            // ��Сֵ����Ϊ1, ���Ϊ0���ʾ�����г�ʱ�ж�                                    
/* state constant(�����ڽ���) */
#define FSA_INIT            0            //�ȴ�֡ͷ
#define FSA_ADDR_D          1            //�ȴ�Ŀ�ĵ�ַ
#define FSA_ADDR_S          2            //�ȴ�Դ��ַ
#define FSA_LENGTH          3            //�ȴ������ֽ�
#define FSA_DATA            4            //�ȴ����(���� ����ID �� ����)
#define FSA_CHKSUM          5            //�ȴ�У���

/* Uart Queue */
typedef struct strUART_Q
{
  Byte  flag;                               //״̬�� 0 - ���У� 1 - �ȴ����ͣ� 2 - ���ڷ���; 3 - �ѷ��ͣ��ȴ�Ӧ��
  Byte  tdata[MAX_TransFrame];              //���ݰ�(���һ��У���ֽڿ��Բ���ǰ���㣬���ڷ���ʱ�߷��ͱ߼���)
  Byte  len;					            //���ݰ���Ч����(��У���ֽ�)
}sUART_Q;

#define UART_QUEUE_NUM      4            //UART ������, ������λ��

#define bDoorKeeper         P46          //�������룬�Ŵż��: 1-�ŴŹر�; 0-�ŴŴ򿪣�Ӧ������
#define bRelay_A1           P34		   //���������(���)�������1��1-�̵����ӵ�����(�ϵ�ȱʡ)�� 0-���ӵ�

/* interrupt enable */
#define Enable_interrupt()  (EA = 1)
#define Disable_interrupt() (EA = 0)

#endif