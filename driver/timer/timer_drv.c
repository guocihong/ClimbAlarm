#include "driver/timer/timer_drv.h"
#include "driver/adc/adc_drv.h"

/* UART1 */
extern idata  Byte        recv1_state;                 // receive state
extern idata  Byte        recv1_timer;                 // receive time-out, �����ֽڼ䳬ʱ�ж�

/* for AD */
extern data   sAD_Sample  ad_sample;                   // ���浱ǰ����ֵ

/* Doorkeep */   
extern xdata  Byte        gl_dk_tick;                  //�Ŵż���ʱtick  

/* ϵͳ��ʱ */
extern xdata  Uint16      gl_delay_tick;               //ͨ����ʱ��tick
extern xdata  Byte        gl_ack_tick;	               //Ӧ����ʱ��ʱ tick


/* variables for alarm output */ 
extern bdata  bit         alarm1_flag;
extern data   Uint16      alarm1_timer;                // ��ʱ����������1�ѱ���ʱ��,��λtick 
extern data   Uint16      ad_alarm_tick;               //��ͨ��������ʱtick
 
/* ����������ƫ�� */
extern xdata  Uint16      sensor_sample_offset;        //����������ƫ�û������ʱ������������ֵ��Ϊ0����Լ400���ң���Ҫ������˲������ = ����ֵ - ����ƫ��

void timer0_init(void)   // 5ms@22.1184MHz
{    
    // ��ʱ��0��ʼ��
	AUXR &= 0x7F;		 // ����Ϊ12Tģʽ
	TMOD &= 0xF0;		 // ����Ϊ����ģʽ1
	TMOD |= 0x01;
	TL0 = 0x00;		     // ���ö�ʱ��ֵ
	TH0 = 0xDC;		     // ���ö�ʱ��ֵ
	TF0 = 0;		     // ���TF0��־
    ET0 = 1;             // ʹ��T0�ж�����λ
    IPH |= (1 << 1);
    PT0 = 0;             // �����ж����ȼ�Ϊ���ȼ�2
	TR0 = 1;		     // ��ʱ��0��ʼ��ʱ
	
	// ����ADת��
	ADC_CONTR |= ADC_START;
}

void timer0_isr(void) interrupt TF0_VECTOR using 1
{	               
    // ��װ��ֵ
	TL0 = 0x00;		     // ���ö�ʱ��ֵ
	TH0 = 0xDC;		     // ���ö�ʱ��ֵ
	TR0 = 1;		     // ��ʱ��0��ʼ��ʱ
        
    // ADת�����,��ADC_FLAGת����ɱ�־����
    ADC_CONTR &= ~ADC_FLAG;

	// ��ADֵ
	if (ad_sample.valid == FALSE) {
		// ԭ�����Ѿ�������, ����д��������
		ad_sample.val   = ADC_RES;             // ����8λ
		ad_sample.val   = ad_sample.val << 2;
		ad_sample.val   += (ADC_RESL & 0x03);  // �õ�10bit����ֵ
		ad_sample.valid = TRUE;
		
		//����ֵ��ȥ����ƫ��
/*  		if (ad_sample.val > sensor_sample_offset) {
			ad_sample.val -= sensor_sample_offset;
		} else {
			ad_sample.val = 0;
		}  */
		
		ADC_CONTR |= ADC_START;                   // ����ת��
	}

	// increment task tick counters
	gl_dk_tick++;                                 //�Ŵż���ʱtick
	gl_delay_tick++;                              //ͨ����ʱ��tick
	if (gl_ack_tick > 0) {
		gl_ack_tick--;                            //Ӧ����ʱ��ʱ
	}

	ad_alarm_tick++;
	
	if (alarm1_flag) {
		alarm1_timer++;
	}
 
	// UART1�ֽ�֮����ճ�ʱ
	if (recv1_state != FSA_INIT) {
		//�ǳ�ʼ״̬����Ҫ����Ƿ�ʱ
		if (recv1_timer > 0) {
			recv1_timer--;
		}
		
		if (recv1_timer == 0){
			recv1_state = FSA_INIT;   //���ճ�ʱ, �ָ�����ʼ״̬
		}
	}
}