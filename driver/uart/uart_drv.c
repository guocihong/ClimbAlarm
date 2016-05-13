#include "driver/uart/uart_drv.h"

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


void uart_init(void)    //9600bps@22.1184MHz
{
	//uart1Ӳ����ʼ��
	SCON = 0x50;		//8λ����,�ɱ䲨����,����ʹ��
	PCON &= 0x3F;		//�����ʲ�����
	AUXR &= 0xFB;		//���������ʷ�����ʱ��ΪFosc/12,��12T
	BRT = 0xFA;		    //�趨���������ʷ�������װֵ
	AUXR |= 0x01;		//����1ѡ����������ʷ�����Ϊ�����ʷ�����
	AUXR |= 0x10;		//�������������ʷ�����
	ES = 1;             //ʹ�ܴ���1�ж�
	PS = 1;             //�����ж����ȼ�Ϊ���ȼ�1
	IPH &= 0xEF;

	_nop_();
	_nop_();
	_nop_();
	_nop_();
}

//��λ��
void uart1_isr(void) interrupt SIO_VECTOR using 2
{
	Byte c;

	if (_testbit_(TI)) { //�����ж�
		trans1_ctr++;   //ȡ��һ��������index
		if (trans1_ctr < trans1_size) { //δ�������
			if (trans1_ctr == (trans1_size - 1)) { //�Ѿ�ָ��У���ֽ�
				SBUF = trans1_chksum;    //����У���ֽ�
			} else { //��У���ֽ�, ��Ҫ���Ͳ�����checksum
				SBUF = trans1_buf[trans1_ctr];
				if (trans1_ctr > 0) { //����check_sum
					trans1_chksum += trans1_buf[trans1_ctr];   //����chksum
				}
			}
		} else { //�Ѿ�ȫ���������(��У���ֽ�)�������÷���������
			//Ŀǰ��ƣ�������ȴ�Ӧ��, �����ͷŸö�����
			if (uart1_q_index < UART_QUEUE_NUM)
				uart1_q[uart1_q_index].flag = 0;   //�ö��������
			uart1_q_index = 0xFF;	//�޶������ڷ���
			trans1_occupy = 0;		//����������
		}
		TI = 0;   //must clear by user software
	}

	if (_testbit_(RI)) { //�����ж�
		c = SBUF;
		switch (recv1_state)
		{
		case FSA_INIT://�Ƿ�Ϊ֡ͷ
			if (c == FRAME_STX) { //Ϊ֡ͷ, ��ʼ�µ�һ֡
				recv1_ctr = 0;
				recv1_chksum = 0;
				recv1_timer = RECV_TIMEOUT;
				recv1_state = FSA_ADDR_D;
			}
			break;

		case FSA_ADDR_D://ΪĿ�ĵ�ַ, ��ʼ���沢����Ч���
			recv1_buf[recv1_ctr++] = c;
			recv1_chksum += c;
			recv1_timer = RECV_TIMEOUT;
			recv1_state = FSA_ADDR_S;
			break;

		case FSA_ADDR_S://ΪԴ��ַ
			recv1_buf[recv1_ctr++] = c;
			recv1_chksum += c;
			recv1_timer = RECV_TIMEOUT;
			recv1_state = FSA_LENGTH;
			break;

		case FSA_LENGTH://Ϊ�����ֽ�
			if ((c > 0) && (c < (MAX_RecvFrame - 3))) { //��Ч��
				recv1_buf[recv1_ctr++] = c;    //�������ֽڱ��泤��
				recv1_chksum += c;
				recv1_timer = RECV_TIMEOUT;
				recv1_state = FSA_DATA;
			} else {	//����Ч��
				recv1_state = FSA_INIT;
			}
			break;

		case FSA_DATA://��ȡ���
			recv1_buf[recv1_ctr] = c;
			recv1_chksum += c;   //����У���
			if (recv1_ctr == (recv1_buf[2] + 2)){ //�Ѿ��յ�ָ�����ȵ���������
				recv1_state = FSA_CHKSUM;
			}else{//��δ����
				recv1_ctr ++;
			}
			recv1_timer = RECV_TIMEOUT;
			break;

		case FSA_CHKSUM://���У���ֽ�
			if ((recv1_chksum == c) && (msg1_buf_valid == FALSE)){
				//�Ѿ��յ�����һ֡������ǰ�յ�����Ϣ�Ѿ���������
				memcpy(msg1_buf, recv1_buf, recv1_buf[2] + 3);
				msg1_buf_valid = TRUE;
			}
		default:
			//��λ
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