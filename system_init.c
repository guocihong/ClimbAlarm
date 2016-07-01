#include "system_init.h"

static void gpio_init(void);
static void get_defence_info(void);
static void get_config_info(void);

/* AD sample */
extern bdata  bit      ad_sensor_mask;              //传感器掩码
extern xdata  Uint16   ad_still_dn;                 //静态拉力值下限
extern xdata  Uint16   ad_still_up;                 //静态拉力值上限
extern xdata  Byte     ad_still_Dup;                //报警阀值上限

/* variables for beep */
extern xdata  Uint16   beep_during_temp;            //预设的一次蜂鸣持续时间, 单位:tick

/* 传感器采样偏差 */
extern xdata  Uint16   sensor_sample_offset;        //传感器采样偏差：没有外力时，传感器采样值不为0，大约400左右，需要矫正。瞬间张力 = 采样值 - 采样偏差

/* for system */
extern idata  Byte     gl_comm_addr;                //本模块485通信地址
extern bdata  bit      system_2or1;                 //双/单防区标志: 0 - 双; 1 - 单

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

	//设置P10为AD模式
	P1M1 = 0x01;
	P1M0 = 0x00;

	//设置P31为高阻输入,用来检测是否插入跳线帽，主要用来屏蔽报警
	P3M1 = 0x02;
	P3M0 = 0x00;

	ad_sensor_mask = 1;//缺省:报警不屏蔽
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

	//设置P31,P32,P33,P34为强推挽输出
	P3M1 = 0x00;
	P3M0 = 0x1E;
	P32 = 1;//防拆不报警
	P33 = 1;//杆自身不报警
	P34 = 1;//开关量上电闭合

	//设置P46为高阻输入
	P4M1 = 0x40;
	P4M0 = 0x00;
	P4SW = 0x70;

	//闪一下2个LED -- 全部点亮，用于检测所有LED是否完好
	P32 = 0;   //点亮2个LED
	P33 = 0;

	gl_comm_addr= 55;
	system_2or1 = 1;

	i = 150000;
	while (i>0)  i--;

	P32 = 1;   //熄灭2个LED
	P33 = 1;

	//延时
	i = 150000;
	while (i>0)  i--;
}

//读取系统预设数据
static void get_config_info(void)
{
	Byte temp;

	//使能Flash访问
	flash_enable();

	//读静态张力值范围
	temp = flash_read(EEPROM_SECTOR3);
	if (temp == 0x5A) { //有有效设置
		//下限
		temp = flash_read(EEPROM_SECTOR3 + 1);
		ad_still_dn = (Uint16)temp << 8;
		temp = flash_read(EEPROM_SECTOR3 + 2);
		ad_still_dn += temp;
		//上限
		temp = flash_read(EEPROM_SECTOR3 + 3);
		ad_still_up = (Uint16)temp << 8;
		temp = flash_read(EEPROM_SECTOR3 + 4);
		ad_still_up += temp;
		//有效否?
		if ((ad_still_dn < STD_STILL_DN) ||
		        (ad_still_up > STD_STILL_UP) ||
		        (ad_still_dn >= ad_still_up)) { //无合法数据，取缺省值
			ad_still_dn = STD_STILL_DN;
			ad_still_up = STD_STILL_UP;
		}
	} else {	//无有效设置，取缺省值
		ad_still_dn = STD_STILL_DN;
		ad_still_up = STD_STILL_UP;
	}

	//读报警阀值参数
	temp = flash_read(EEPROM_SECTOR4);
	if (temp == 0x5A) { //有有效设置
		ad_still_Dup = flash_read(EEPROM_SECTOR4 + 1);

		//是否有效？
		if ((ad_still_Dup < STD_ALARM_MIN) || (ad_still_Dup > STD_ALARM_MAX))
			ad_still_Dup = STD_ALARM_DEF;    //无合法数据，取缺省值
	} else {	//无有效设置，取缺省值
		ad_still_Dup = STD_ALARM_DEF;
	}

	//6. 读声光报警时间设置
	temp = flash_read(EEPROM_SECTOR5);
	if (temp == 0x5A) { //有有效设置
		temp = flash_read(EEPROM_SECTOR5 + 1);
		beep_during_temp = (Uint16)(((Uint32)temp * 1000) / SCHEDULER_TICK);
	} else {	//取缺省值
		beep_during_temp = 0;   //单位： tick
	}

	//读传感器采样偏差
	temp = flash_read(EEPROM_SECTOR6);
	if (temp == 0x5A) { //有有效设置
		temp = flash_read(EEPROM_SECTOR6 + 1);
		sensor_sample_offset = ((Uint16)temp << 8);
		temp = flash_read(EEPROM_SECTOR6 + 2);
		sensor_sample_offset += temp;
	} else {	//无有效设置
		sensor_sample_offset = 0;
	}

	//9. 禁止Flash访问
	flash_disable();
}
