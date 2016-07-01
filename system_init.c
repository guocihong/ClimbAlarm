#include "system_init.h"

static void gpio_init(void);
static void get_defence_info(void);
static void get_config_info(void);

/* AD sample */
extern bdata  bit      ad_sensor_mask;              //����������
extern xdata  Uint16   ad_still_dn;                 //��̬����ֵ����
extern xdata  Uint16   ad_still_up;                 //��̬����ֵ����
extern xdata  Byte     ad_still_Dup;                //������ֵ����

/* variables for beep */
extern xdata  Uint16   beep_during_temp;            //Ԥ���һ�η�������ʱ��, ��λ:tick

/* ����������ƫ�� */
extern xdata  Uint16   sensor_sample_offset;        //����������ƫ�û������ʱ������������ֵ��Ϊ0����Լ400���ң���Ҫ������˲������ = ����ֵ - ����ƫ��

/* for system */
extern idata  Byte     gl_comm_addr;                //��ģ��485ͨ�ŵ�ַ
extern bdata  bit      system_2or1;                 //˫/��������־: 0 - ˫; 1 - ��

void system_init(void)
{
	gpio_init();

	get_config_info();

	status_task_init();
	adc_task_init();
	uart_task_init();
	doorkeep_task_init();
	alarm_task_init();

	timer0_init();
}


static void gpio_init(void)
{
	Uint32 i;
	Byte j;

	//����P10ΪADģʽ
	P1M1 = 0x01;
	P1M0 = 0x00;

	//����P31Ϊ��������,��������Ƿ��������ñ����Ҫ�������α���
	P3M1 = 0x02;
	P3M0 = 0x00;

	ad_sensor_mask = 1;//ȱʡ:����������
	do
	{
		j = 255;
		while (j > 0)  j--;
		if (ad_sensor_mask == !P31)
		{
			i++;
		}
		else
		{
			i = 0;
			ad_sensor_mask = !P31;
		}
	} while (i < 8);
	ad_sensor_mask = !ad_sensor_mask;

	//����P31,P32,P33,P34Ϊǿ�������
	P3M1 = 0x00;
	P3M0 = 0x1E;
	P32 = 1;//���𲻱���
	P33 = 1;//����������
	P34 = 1;//�������ϵ�պ�

	//����P46Ϊ��������
	P4M1 = 0x40;
	P4M0 = 0x00;
	P4SW = 0x70;

	//��һ��2��LED -- ȫ�����������ڼ������LED�Ƿ����
	P32 = 0;   //����2��LED
	P33 = 0;

	gl_comm_addr= 55;
	system_2or1 = 1;

	i = 150000;
	while (i>0)  i--;

	P32 = 1;   //Ϩ��2��LED
	P33 = 1;

	//��ʱ
	i = 150000;
	while (i>0)  i--;
}

//��ȡϵͳԤ������
static void get_config_info(void)
{
	Byte temp;

	//ʹ��Flash����
	flash_enable();

	//����̬����ֵ��Χ
	temp = flash_read(EEPROM_SECTOR3);
	if (temp == 0x5A) { //����Ч����
		//����
		temp = flash_read(EEPROM_SECTOR3 + 1);
		ad_still_dn = (Uint16)temp << 8;
		temp = flash_read(EEPROM_SECTOR3 + 2);
		ad_still_dn += temp;
		//����
		temp = flash_read(EEPROM_SECTOR3 + 3);
		ad_still_up = (Uint16)temp << 8;
		temp = flash_read(EEPROM_SECTOR3 + 4);
		ad_still_up += temp;
		//��Ч��?
		if ((ad_still_dn < STD_STILL_DN) ||
		        (ad_still_up > STD_STILL_UP) ||
		        (ad_still_dn >= ad_still_up)) { //�޺Ϸ����ݣ�ȡȱʡֵ
			ad_still_dn = STD_STILL_DN;
			ad_still_up = STD_STILL_UP;
		}
	} else {	//����Ч���ã�ȡȱʡֵ
		ad_still_dn = STD_STILL_DN;
		ad_still_up = STD_STILL_UP;
	}

	//��������ֵ����
	temp = flash_read(EEPROM_SECTOR4);
	if (temp == 0x5A) { //����Ч����
		ad_still_Dup = flash_read(EEPROM_SECTOR4 + 1);

		//�Ƿ���Ч��
		if ((ad_still_Dup < STD_ALARM_MIN) || (ad_still_Dup > STD_ALARM_MAX))
			ad_still_Dup = STD_ALARM_DEF;    //�޺Ϸ����ݣ�ȡȱʡֵ
	} else {	//����Ч���ã�ȡȱʡֵ
		ad_still_Dup = STD_ALARM_DEF;
	}

	//6. �����ⱨ��ʱ������
	temp = flash_read(EEPROM_SECTOR5);
	if (temp == 0x5A) { //����Ч����
		temp = flash_read(EEPROM_SECTOR5 + 1);
		beep_during_temp = (Uint16)(((Uint32)temp * 1000) / SCHEDULER_TICK);
	} else {	//ȡȱʡֵ
		beep_during_temp = 0;   //��λ�� tick
	}

	//������������ƫ��
	temp = flash_read(EEPROM_SECTOR6);
	if (temp == 0x5A) { //����Ч����
		temp = flash_read(EEPROM_SECTOR6 + 1);
		sensor_sample_offset = ((Uint16)temp << 8);
		temp = flash_read(EEPROM_SECTOR6 + 2);
		sensor_sample_offset += temp;
	} else {	//����Ч����
		sensor_sample_offset = 0;
	}

	//9. ��ֹFlash����
	flash_disable();
}
