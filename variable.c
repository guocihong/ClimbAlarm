#include "config.h"

/* UART1 */		
xdata  Byte     msg1_buf[MAX_RecvFrame];     // received message, used for proceding
bdata  bit      msg1_buf_valid;	             // received valid flag
xdata  Byte     recv1_buf[MAX_RecvFrame];    // receiving buffer
idata  Byte     recv1_state;                 // receive state
idata  Byte     recv1_timer;                 // receive time-out, �����ֽڼ䳬ʱ�ж�
idata  Byte     recv1_chksum;                // computed checksum
idata  Byte     recv1_ctr;                   // reveiving pointer

xdata  Byte     trans1_buf[MAX_TransFrame];  // uart transfer message buffer
idata  Byte     trans1_ctr;                  // transfer pointer
idata  Byte     trans1_size;                 // transfer bytes number
idata  Byte     trans1_chksum;               // computed check-sum of already transfered message
bdata  bit      trans1_occupy;               // ��������ռ�ñ�־��1-��ռ��, 0-����

data  Byte      uart1_q_index;               // ���ڷ���ĳ���������ţ���Ϊ0xFF, ��ʾû���κ�����뷢������
xdata sUART_Q   uart1_q[UART_QUEUE_NUM];	 // ���ڶ���				 						  								   

/* AD sample */
bdata  bit         ad_sensor_mask;           //����������
 data  sAD_Sample  ad_sample;                //���浱ǰ����ֵ
idata  sAD_Sum     ad_samp_equ;              //����ȥ�������
xdata  Union16     ad_chn_sample;            //����һ�ֲ���ֵ���Ѿ���ȥ������ÿͨ��һ���㣬ѭ�����棩
idata  Uint16      ad_samp_pnum;             //��������(���㾲̬��׼ֵʱ�ܲ�������)
idata  sAD_Sum     ad_samp_sum;              //�׶����
xdata  sAD_BASE    ad_chn_base;              //��ͨ������ʱ��̬��׼ֵ/�����޷�ֵ����λ������ֵ��
 data  Byte        ad_chn_over;              //��ͨ������������(�����)�ķ�ֵ�ж��� 0 - ��Χ�ڣ� 1 - ����ֵ
xdata  Uint16      ad_still_dn;              //��̬����ֵ����
xdata  Uint16      ad_still_up;              //��̬����ֵ����
 xdata  Byte       ad_still_Dup;             //������ֵ����
idata  Uint16      ad_alarm_exts;            //����������־��δ��mask����λֵ 0 - �ޣ� 1 - ����ֵ                    

/* for system */
idata  Byte     system_status;               //ϵͳ״̬
										 	 // 0 - ��ʼ�ϵ�
											 // 1 - ��׼ֵ����ǰ��ʱ
											 // 2 - ��׼ֵ����(10������)
											 // 3 - ʵʱ���
idata  Byte     gl_comm_addr;                //��ģ��485ͨ�ŵ�ַ
bdata  bit      system_2or1;                 //˫/��������־: 0 - ˫; 1 - ��		

/* ϵͳ��ʱ */
xdata  Byte     gl_ack_tick = 0;	         //��λ��485��Ӧ����ʱ��ʱ tick
xdata  Uint16   gl_delay_tick;               //ͨ����ʱ��tick
 
/* variables for alarm output */
bdata  bit      ad_alarm_flag;               //����������־
 data  Uint16   ad_alarm_tick;               //��ͨ��������ʱtick
bdata  bit      alarm1_flag;
 data  Uint16   alarm1_timer;                //��ʱ����������1�ѱ���ʱ��,��λ:tick 

/* variables for beep */
xdata  Uint16   beep_during_temp;            //Ԥ���һ�η�������ʱ��, ��λ:tick 

/* Doorkeep(�Ŵ�) */
bdata  bit      gl_dk_status;                //�Ŵſ���״̬��ÿ1s��̬��⣩: 1 - �պ�; 0 - ��(��Ҫ����)                    
xdata  Byte     gl_dk_tick;  	             //�Ŵż���ʱtick

/* ����������ƫ�� */
xdata  Uint16   sensor_sample_offset;        //����������ƫ�û������ʱ������������ֵ��Ϊ0����Լ400���ң���Ҫ������˲������ = ����ֵ - ����ƫ��
									         