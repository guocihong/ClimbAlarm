#include "task/adc/adc_task.h"

/* for AD */
#define AD_EQU_PNUM            4                     //ÿ����˿����4����ƽ��ֵ

extern  data  sAD_Sample  ad_sample;         //���浱ǰ����ֵ
extern idata  sAD_Sum     ad_samp_equ;       //����ȥ�������
extern xdata  Union16     ad_chn_sample;     //����һ�ֲ���ֵ���Ѿ���ȥ������ÿͨ��һ���㣬ѭ�����棩
extern idata  Uint16      ad_samp_pnum;      //��������(���㾲̬��׼ֵʱ�ܲ�������)
extern idata  sAD_Sum     ad_samp_sum;       //�׶����
extern xdata  sAD_BASE    ad_chn_base;       //��ͨ������ʱ��̬��׼ֵ/�����޷�ֵ����λ������ֵ��
extern  data  Byte        ad_chn_over;       //��ͨ������������(�����)�ķ�ֵ�ж��� 0 - ��Χ�ڣ� 1 - ����ֵ
extern xdata  Byte        ad_still_Dup;      //������ֵ����
extern idata  Uint16      ad_alarm_exts;     //����������־����mask���� λֵ 0 - �ޣ� 1 - ����ֵ

/* for this task: ���ڻ�׼ֵ���� */
static idata  Byte        md_point;          //���ڻ�׼ֵ���ٵļ�������

/* for system */
extern idata  Byte        system_status;     //ϵͳ״̬

/* variables for alarm output */
extern data   Uint16      ad_alarm_tick;     //��ͨ��������ʱtick
extern bdata  bit         ad_alarm_flag;     //����������־
static xdata  Uint16      alarm_led_flag;	 //LED����ָʾ: 0 - �ޱ������𣩣�1 - ����������

void adc_task_init(void)
{
	//��ر�����ʼ��
	ad_sample.valid = 0;                     //���У�����д����ֵ
	ad_samp_pnum    = 0;                     //��������(���㾲̬��׼ֵʱ�ܲ�������)
	ad_samp_equ.sum       = 0;	             //����ȥ�������
	ad_samp_equ.point     = 0;
	ad_samp_sum.sum       = 0;	             //�׶����
	ad_samp_sum.point     = 0;
	ad_chn_sample.w       = 0;	             //����һ�ֲ���ֵ
	ad_chn_base.base      = 0;	             //��ͨ����̬��׼ֵ/�����޷�ֵ
	ad_chn_base.base_down = 0;
	ad_chn_base.base_up   = 0;
	ad_chn_over           = 0x00;	         //��ͨ������������(�����)�ķ�ֵ�ж������ڷ�Χ��
	md_point              = 0;               //���ڻ�׼ֵ���ٵļ�������
	ad_alarm_tick         = 0x00;

	ad_alarm_exts   = 0;	  				 //����������־����mask��: ��
	alarm_led_flag  = 0;	                 //���б���LEDΪ��
	ad_alarm_flag   = 0;

	//adcӲ����ʼ��
	adc_init();
}

void adc_task(void)
{
	Uint16  temp16;         //��ʱ����
	Uint16  val_temp;       //�������10bit����ֵ,  ������ʱ����
	Uint16  val;            //4������õ���ƽ������ֵ, ��Ϊһ���ɽ��г����жϵ���С��

	if (ad_sample.valid) {  //���²������ݵ���
		// 0. ���浽��ʱ����
		val_temp = ad_sample.val;

		// 1. ���浽����ȥ���������
		ad_samp_equ.sum += val_temp;
		ad_samp_equ.point++;

		// 2. ��ǰͨ���Ƿ�����ȥ��������
		if (ad_samp_equ.point == AD_EQU_PNUM) {
			// ����ȥ���������������������һ����
			// 2.a �����Ӧͨ����һ��������
			val = ad_samp_equ.sum >> 2;  //����4

			// 2.b ���㵱ǰͨ����ȥ������ͽṹ
			ad_samp_equ.sum = 0;
			ad_samp_equ.point = 0;

			// 2.c ����ʵʱ����ֵ
			ad_chn_sample.w = val;   //���浽����һ�ֲ���ֵ������

			// 2.d ��ϵͳ״̬�������ݵĴ���
			switch (system_status)
			{
			case SYS_PowerON:   //�ϵ�
			case SYS_B5S:       //5����ʱ
				break;

			case SYS_SAMP_BASE: //��ʼ�ϵ�ʱ�ľ�̬��׼ֵ����
				//����׶κ�
				ad_samp_sum.sum += val;
				ad_samp_pnum++;
				if (ad_samp_pnum == 32) {
					//�Ѿ�����׼ֵ����������ÿͨ��32��)
					//1.�����ֵ��������
					//��׼
					ad_chn_base.base = ad_samp_sum.sum >> 5;   //����32

					//���� = ��׼ * ��1 / 3��
					val_temp = ad_chn_base.base;
                    ad_chn_base.base_down = val_temp - ad_still_Dup;

					//����
					if ((1023 - ad_chn_base.base) > ad_still_Dup) {
						ad_chn_base.base_up = ad_chn_base.base + ad_still_Dup;
					} else {
						ad_chn_base.base_up = 1023;
					}

					//��λ�׶κͱ�����׼����������Ӧ��ֵ����
					ad_samp_sum.sum   = 0;
					ad_samp_sum.point = 0;

					//2.e ״̬-> ʵʱ���
					system_status = SYS_CHECK;
				}

				break;

			case SYS_CHECK: //ʵʱ���
				//2. �ж��Ƿ���������
				ad_chn_over = ad_chn_over << 1;   //Bit0��0�����ȱʡ������Χ��
				if ((val >= ad_chn_base.base_down) && (val <= ad_chn_base.base_up)) {
					//��������/��������Χ��
					//a. ���־(ȱʡ)
					//b. ������ٻ�׼ֵ�����
					ad_samp_sum.sum += val;
					ad_samp_sum.point++;

					if (ad_samp_sum.point == 2) {
						//��2��(Լ��0.6��)
						//b.0 ������2���ֵ
						val_temp = ad_samp_sum.sum >> 1;   //����2, �õ���2��ľ�ֵ
						//b.1 ���»�׼ֵ
						if (ad_chn_base.base > (val_temp + 1)) {                            
                            // ����С��2,�����ɳ�
							md_point++;
							if (md_point >= DEF_ModiBASE_PT) {
                                // ���������ɳ�ʱ��������������, ����һ�θ���
								// 1. ���ٻ�׼ֵ
                               if (ad_chn_base.base > 0) {
                                    //���Եݼ�1
                                    ad_chn_base.base--;
                                    // ͬ������������
                                    val_temp = ad_chn_base.base;
                                    if ((val_temp - ad_still_Dup) > 0) {
                                        ad_chn_base.base_down = val_temp - ad_still_Dup;
                                    }
                                    
                                    if (ad_chn_base.base_up > 0) {
                                        ad_chn_base.base_up--;
                                    }
                                }
								// 2. �建���ɳڸ��ٱ���
								md_point = 0;
							}
						} else if (val_temp > (ad_chn_base.base + 1)) {
							// ���ٴ�2, �����Ž�
							md_point++;
							if (md_point >= DEF_ModiBASE_PT) {
								// ���������Ž�ʱ��������������, ����һ�θ���
								// 1. ���ٻ�׼ֵ
								if (ad_chn_base.base < 1023) {
									//���Ե���1
									ad_chn_base.base++;
									// ͬ������������
									val_temp = ad_chn_base.base;
                                    ad_chn_base.base_down = val_temp - ad_still_Dup;
									if (ad_chn_base.base_up < 1023) {
										ad_chn_base.base_up++;
									}
								}
								// 2. �建���Ž����ٱ���
								md_point = 0;
							}
						}

						//b.2 ��λ�׶κͱ��� - ����4��4��ƽ������ͽṹ
						ad_samp_sum.sum   = 0;
						ad_samp_sum.point = 0;
					}
				} else {
					//��������, �ñ�־
					ad_chn_over |= 0x01;
				}

				//2.4 ����4�㳬��Χ����ͨ������������
				if ((ad_chn_over & 0x0F) == 0x0F) {
					//������������־
					ad_alarm_exts |= (Uint16)0x01;

					//������ʱtick����
					ad_alarm_tick = 0;

					//�������»���
					ad_chn_base.base = val;
					val_temp = ad_chn_base.base;
                    ad_chn_base.base_down = val_temp - ad_still_Dup;
					if ((1023 - ad_chn_base.base) > ad_still_Dup) {
						ad_chn_base.base_up = ad_chn_base.base + ad_still_Dup;
					} else {
						ad_chn_base.base_up = 1023;
					}

					//��λ�׶κͱ��� - ����4��4��ƽ������ͽṹ
					ad_samp_sum.sum = 0;
					ad_samp_sum.point = 0;

					//�建���Ž����ٱ���
					md_point = 0;   //���ڻ�׼ֵ���ٵļ�������
				} else if ((ad_chn_over & 0x0F) == 0x00) {
					//����������
					//��鱨��ʱ���Ƿ��ѵ�
					if (ad_alarm_tick > ALARM_TEMPO) {
						//�����Ѿ�����󱨾�ʱ��, ֹͣ����
						ad_alarm_exts &= ~((Uint16)0x01);
					}
				}

				//3 LEDָʾ(��mask)
				temp16 = ad_alarm_exts & 0x0001;
				if (alarm_led_flag != temp16) {
					//LEDָʾ��Ϣ�б仯
					alarm_led_flag = temp16;
					P33 = !P33;
				}

				//4 ���±�����־
				temp16 = ad_alarm_exts & 0x0001;
				if (temp16 == 0) {
					ad_alarm_flag = 0;   //�ޱ���
				} else {
					ad_alarm_flag = 1;   //�б���
				}

				break;
			}

			//3.��ǰ����ֵ������ϣ������µĲ���ֵ����
			ad_sample.valid = FALSE;
		}
	}
}