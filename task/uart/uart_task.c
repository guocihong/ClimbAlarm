#include "task/uart/uart_task.h"

#define REPLY_DLY   (10/SCHEDULER_TICK)             //�յ�PC������Ӧ����ʱ

/* ϵͳ��ʱ */
extern xdata  Byte     gl_ack_tick;	                //Ӧ����ʱ��ʱ tick

/* UART1 */
extern xdata  Byte     msg1_buf[MAX_RecvFrame];     // received message, used for proceding
extern bdata  bit      msg1_buf_valid;	            // received valid flag
extern xdata  Byte     recv1_buf[MAX_RecvFrame];    // receiving buffer
extern idata  Byte     recv1_state;                 // receive state
extern idata  Byte     recv1_timer;                 // receive time-out, �����ֽڼ䳬ʱ�ж�
extern idata  Byte     recv1_chksum;                // computed checksum
extern idata  Byte     recv1_ctr;                   // reveiving pointer

extern xdata  Byte     trans1_buf[MAX_TransFrame];  // uart transfer message buffer
extern idata  Byte     trans1_ctr;                  // transfer pointer
extern idata  Byte     trans1_size;                 // transfer bytes number
extern idata  Byte     trans1_chksum;               // computed check-sum of already transfered message
extern bdata  bit      trans1_occupy;               // ��������ռ�ñ�־��1-��ռ��, 0-����

extern  data  Byte     uart1_q_index;               // ���ڷ���ĳ���������ţ���Ϊ0xFF, ��ʾû���κ�����뷢������
extern xdata  sUART_Q  uart1_q[UART_QUEUE_NUM];	    // ���ڶ���

/* for system */
extern idata  Byte     system_status;               //ϵͳ״̬
extern bdata  bit      system_2or1;                 //˫/��������־: 0 - ˫; 1 - ��
extern idata  Byte     gl_comm_addr;                //��ģ��485ͨ�ŵ�ַ

/* variables for alarm output */
extern bdata  bit      ad_alarm_flag;               //����������־

/* variables for beep */
extern xdata  Uint16   beep_during_temp;            //Ԥ���һ�η�������ʱ��, ��λ:tick 

/* Doorkeep(�Ŵ�) */
extern bdata  bit      gl_dk_status;                //�Ŵſ���״̬��ÿ1s��̬��⣩: 1 - �պ�; 0 - ��(��Ҫ����)

/* AD sample */
extern xdata  Union16  ad_chn_sample;               //����һ�ֲ���ֵ���Ѿ���ȥ������ÿͨ��һ���㣬ѭ�����棩
extern xdata  sAD_BASE ad_chn_base;                 //��ͨ������ʱ��̬��׼ֵ/�����޷�ֵ����λ������ֵ��
extern xdata  Uint16   ad_still_dn;                 //��̬����ֵ����
extern xdata  Uint16   ad_still_up;                 //��̬����ֵ����
extern xdata  Byte     ad_still_Dup;                //������ֵ����

/* ����������ƫ�� */
extern xdata  Uint16   sensor_sample_offset;        //����������ƫ�û������ʱ������������ֵ��Ϊ0����Լ400���ң���Ҫ������˲������ = ����ֵ - ����ƫ��
                                                    

Byte check_still_stress(Byte index);

void uart_task_init(void)
{
	Byte i;

	//uart1��ر�����ʼ��
	msg1_buf_valid = FALSE;
	recv1_state = FSA_INIT;
	recv1_timer = 0;
	recv1_ctr = 0;
	recv1_chksum = 0;
	trans1_size = 0;
	trans1_ctr = 0;
	trans1_occupy = 0;       //����

	for (i = 0; i < UART_QUEUE_NUM; i++){
		uart1_q[i].flag = 0; //������
	}
	uart1_q_index = 0xFF;    //�޶�������뷢������

	//UARTӲ����ʼ��
	uart_init();             //֮���Ѿ�׼���ô����շ���ֻ�ǻ�δʹ��ȫ���ж�
}

void uart_task(void)
{
	Byte   i, j;
	Uint16 temp16;

	//1.���ղ����� UART1��������λ���������
	if (msg1_buf_valid) {
		//a.1 �Ƿ���Ҫִ�б�����
		if ((msg1_buf[0] == CMD_ADDR_BC) || (msg1_buf[0] == gl_comm_addr)) {
			//�㲥��ַ��ָ�����豸, ��Ҫִ��
			switch (msg1_buf[3])
			{
			case CMD_ZL_PRE://����/����ר�������־
				switch (msg1_buf[5])
				{
				case 0x10: //�����ò���
					//��UART�������ҿ���Buffer
					i = uart1_get_buffer();
					if (i < UART_QUEUE_NUM) {
						//�ҵ��˿���buffer, д��data
						uart1_q[i].tdata[0]  = FRAME_STX;
						uart1_q[i].tdata[1]  = msg1_buf[1];	             //Ŀ�ĵ�ַ
						uart1_q[i].tdata[2]  = gl_comm_addr;	         //Դ��ַ
						uart1_q[i].tdata[3]  = 0x1E;                     //�����
						uart1_q[i].tdata[4]  = CMD_ZL_PRE;		         //����ID
						uart1_q[i].tdata[5]  = 0x1C;                     //����1
						uart1_q[i].tdata[6]  = 0x08;                     //����2
						uart1_q[i].tdata[7]  = 0x00;  //����������
						uart1_q[i].tdata[8]  = 0x00;   //����������
						uart1_q[i].tdata[9]  = HIGH(ad_still_dn);        //��̬�����������޸�
						uart1_q[i].tdata[10] = LOW(ad_still_dn);         //��̬�����������޵�
						uart1_q[i].tdata[11] = HIGH(ad_still_up);        //��̬�����������޸�
						uart1_q[i].tdata[12] = LOW(ad_still_up);         //��̬�����������޵�
						uart1_q[i].tdata[13] = 66;                       //������ֵ�¸��������̶�Ϊ66
						for (j = 0; j < 7; j++) {                        //������ֵ�����ϸ���1~8
							uart1_q[i].tdata[14 + j] = ad_still_Dup;
						}

						uart1_q[i].tdata[21] = 0;
						for (j = 0; j < 7; j++) {                        //������ֵ�����ϸ���1~8
							uart1_q[i].tdata[22 + j] = ad_still_Dup;
						}
						uart1_q[i].tdata[29] = 0;
						uart1_q[i].tdata[30] = system_2or1;              //˫/������
						uart1_q[i].tdata[31] = gl_comm_addr;             //�����ַ
						uart1_q[i].tdata[32] = (Byte)(((Uint32)beep_during_temp * SCHEDULER_TICK) / 1000);	//���ⱨ�����ʱ��
						uart1_q[i].tdata[33] = 0;                        //�������ʱ��
						uart1_q[i].len = 35;
					} else {
						//�޿���buffer, ����������
						//���: ���ж��������ڷ���, �ȴ������
						while (uart1_q_index != 0xFF);	//������,������ WDT ��λ
					}
					break;

				case 0x12: //��������Ϣ
					//��UART�������ҿ���Buffer
					i = uart1_get_buffer();
					if (i < UART_QUEUE_NUM) {
						//�ҵ��˿���buffer, д��data
						uart1_q[i].tdata[0]  = FRAME_STX;
						uart1_q[i].tdata[1]  = msg1_buf[1];	            //Ŀ�ĵ�ַ
						uart1_q[i].tdata[2]  = gl_comm_addr;	        //Դ��ַ
						uart1_q[i].tdata[3]  = 0x0A;                    //�����
						uart1_q[i].tdata[4]  = CMD_ZL_PRE;              //����ID
						uart1_q[i].tdata[5]  = 0x08;                    //����1
						uart1_q[i].tdata[6]  = 0x1A;                    //����1
						uart1_q[i].tdata[7]  = 0x00;                    //����������
						uart1_q[i].tdata[8]  = 0x00;                    //����������

						uart1_q[i].tdata[9]  = (Byte)ad_alarm_flag;     //����������
						uart1_q[i].tdata[10] = 0x00;                    //����������

						uart1_q[i].tdata[11] = 0x00;                    //��̬����������
						uart1_q[i].tdata[12] = 0x00;                    //��̬����������
						
						uart1_q[i].tdata[13] = (Byte)(!gl_dk_status);   //�Ŵ�
						uart1_q[i].len = 15;
					} else {
						//�޿���buffer, ����������
						//���: ���ж��������ڷ���, �ȴ������
						while (uart1_q_index != 0xFF);	//������,������ WDT ��λ
					}
					break;

				case 0x14: //��˲������
					//��UART�������ҿ���Buffer
					i = uart1_get_buffer();
					if (i < UART_QUEUE_NUM) {
						//�ҵ��˿���buffer, д��data
						uart1_q[i].tdata[0] = FRAME_STX;
						uart1_q[i].tdata[1] = msg1_buf[1];	    //Ŀ�ĵ�ַ
						uart1_q[i].tdata[2] = gl_comm_addr;	    //Դ��ַ
						uart1_q[i].tdata[3] = 0x23;
						uart1_q[i].tdata[4] = CMD_ZL_PRE;
						uart1_q[i].tdata[5] = 0x21;
						uart1_q[i].tdata[6] = 0x1C;
						
						//��1
						temp16 = ad_chn_sample.w;
						uart1_q[i].tdata[7] = HIGH(temp16);
						uart1_q[i].tdata[8] = LOW(temp16);
							
						for (j = 1; j < 8; j++) { //��2~8
							uart1_q[i].tdata[7 + (j << 1)] = 0x00;
							uart1_q[i].tdata[8 + (j << 1)] = 0x00;
						}

						
						for (j = 0; j < 8; j++) { //��1~8
							uart1_q[i].tdata[23 + (j << 1)] = 0x00;
							uart1_q[i].tdata[24 + (j << 1)] = 0x00;
						}

						uart1_q[i].len = 40;
					} else {
						//�޿���buffer, ����������
						//���: ���ж��������ڷ���, �ȴ������
						while (uart1_q_index != 0xFF);	//������,������ WDT ��λ
					}
					break;

				case 0x15: //����̬������׼
					//��UART�������ҿ���Buffer
					i = uart1_get_buffer();
					if (i < UART_QUEUE_NUM) {
						//�ҵ��˿���buffer, д��data
						uart1_q[i].tdata[0] = FRAME_STX;
						uart1_q[i].tdata[1] = msg1_buf[1];	    //Ŀ�ĵ�ַ
						uart1_q[i].tdata[2] = gl_comm_addr;	    //Դ��ַ
						uart1_q[i].tdata[3] = 0x23;
						uart1_q[i].tdata[4] = CMD_ZL_PRE;
						uart1_q[i].tdata[5] = 0x21;
						uart1_q[i].tdata[6] = 0x1D;
						
						//��1
						temp16 = ad_chn_base.base;
						uart1_q[i].tdata[7] = HIGH(temp16);
						uart1_q[i].tdata[8] = LOW(temp16);
							
						for (j = 1; j < 8; j++) { //��2~8
							uart1_q[i].tdata[7 + (j << 1)] = 0x00;
							uart1_q[i].tdata[8 + (j << 1)] = 0x00;
						}

						for (j = 0; j < 8; j++) { //��1~8
							uart1_q[i].tdata[23 + (j << 1)] = 0x00;
							uart1_q[i].tdata[24 + (j << 1)] = 0x00;
						}

						uart1_q[i].len = 40;
					} else {
						//�޿���buffer, ����������
						//���: ���ж��������ڷ���, �ȴ������
						while (uart1_q_index != 0xFF);	//������,������ WDT ��λ
					}
					break;

				case 0x40: //���þ�̬����ֵ��Χ
					//1. д��flash
					flash_enable();
					flash_erase(EEPROM_SECTOR3);
					flash_write(msg1_buf[6], EEPROM_SECTOR3 + 1);
					flash_write(msg1_buf[7], EEPROM_SECTOR3 + 2);
					flash_write(msg1_buf[8], EEPROM_SECTOR3 + 3);
					flash_write(msg1_buf[9], EEPROM_SECTOR3 + 4);
					flash_write(0x5a, EEPROM_SECTOR3);
					flash_disable();

					//2. ���±���
					ad_still_dn = ((Uint16)msg1_buf[6] << 8) + msg1_buf[7];	 //����
					ad_still_up = ((Uint16)msg1_buf[8] << 8) + msg1_buf[9];	 //����

					break;
					
				case 0x50: //���ñ�����ֵ
					//1. д��flash�����±���
					flash_enable();
					flash_erase(EEPROM_SECTOR4);

					ad_still_Dup = msg1_buf[7];
					flash_write(msg1_buf[7], EEPROM_SECTOR4 + 1);

					flash_write(0x5a, EEPROM_SECTOR4);
					flash_disable();

					//���޹̶�ȡ��׼ֵ�� 1/3
					//2. ���»�����������������(����ֵ)
					if (system_status == SYS_CHECK) {
						//�ѿ�ʼ���м��
						if ((1023 - ad_chn_base.base) > ad_still_Dup)
							ad_chn_base.base_up = ad_chn_base.base + ad_still_Dup;
						else
							ad_chn_base.base_up = 1023;
					}
					break;

				case 0x60: //�������ⱨ��ʱ��
					//1. д��flash
					flash_enable();
					flash_erase(EEPROM_SECTOR5);
					flash_write(msg1_buf[6], EEPROM_SECTOR5 + 1);
					flash_write(0x5a, EEPROM_SECTOR5);
					flash_disable();

					//2. ���±���
					beep_during_temp = (Uint16)(((Uint32)msg1_buf[6] * 1000) / SCHEDULER_TICK);
					break;
					
				case 0xF1: //���ô���������ƫ��---->������·�ϵ����                  
					//1. д��flash�����±���
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

		//a.3 ����Ӧ����ʱ
		Disable_interrupt();
		gl_ack_tick = REPLY_DLY;
		Enable_interrupt();

		//a.4 ��λ��־
		msg1_buf_valid = FALSE;
	}

	//3. UART1 ���з���
	if ((uart1_q_index == 0xFF) && (recv1_state == FSA_INIT) && (gl_ack_tick == 0)) {
		//UART1�޽��뷢�����̵Ķ�����, ���Ƿ��еȴ����͵���
		for (i = 0; i < UART_QUEUE_NUM; i++) {
			if (uart1_q[i].flag == 1) {
				//�еȴ����͵�����Ŵ����
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
*         ������ֵ >= UART_QUEUE_NUM, ���ʾû�����뵽����buffer
*----------------------------------------------------------------------------
* PURPOSE: �ڴ��ڶ�����Ѱ�ҿ��ж�������ҵ������ض��������(0 ~ (UART_QUEUE_NUM-1))
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
		if (flag == 0) { //���ҵ�����Buffer
			uart1_q[i].flag = 1;
			break;
		}
	}
	return i;
}