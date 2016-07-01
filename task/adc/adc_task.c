#include "task/adc/adc_task.h"

/* for AD */
#define AD_EQU_PNUM            4                     //每道钢丝采样4次求平均值

extern  data  sAD_Sample  ad_sample;         //保存当前采样值
extern idata  sAD_Sum     ad_samp_equ;       //均衡去嘈声求和
extern xdata  Union16     ad_chn_sample;     //最新一轮采样值（已均衡去噪声，每通道一个点，循环保存）
extern idata  Uint16      ad_samp_pnum;      //采样点数(计算静态基准值时总采样点数)
extern idata  sAD_Sum     ad_samp_sum;       //阶段求和
extern xdata  sAD_BASE    ad_chn_base;       //各通道运行时静态基准值/上下限阀值（单位：采样值）
extern  data  Byte        ad_chn_over;       //各通道连续采样点(均衡后)的阀值判定： 0 - 范围内； 1 - 超阀值
extern xdata  Byte        ad_still_Dup;      //报警阀值上限
extern idata  Uint16      ad_alarm_exts;     //外力报警标志（无mask）： 位值 0 - 无； 1 - 超阀值

/* for this task: 用于基准值跟踪 */
static idata  Byte        md_point;          //用于基准值跟踪的计量点数

/* for system */
extern idata  Byte        system_status;     //系统状态

/* variables for alarm output */
extern data   Uint16      ad_alarm_tick;     //各通道报警计时tick
extern bdata  bit         ad_alarm_flag;     //外力报警标志
static xdata  Uint16      alarm_led_flag;	 //LED报警指示: 0 - 无报警（灭）；1 - 报警（亮）

void adc_task_init(void)
{
	//相关变量初始化
	ad_sample.valid = 0;                     //空闲，可以写入新值
	ad_samp_pnum    = 0;                     //采样点数(计算静态基准值时总采样点数)
	ad_samp_equ.sum       = 0;	             //均衡去嘈声求和
	ad_samp_equ.point     = 0;
	ad_samp_sum.sum       = 0;	             //阶段求和
	ad_samp_sum.point     = 0;
	ad_chn_sample.w       = 0;	             //最新一轮采样值
	ad_chn_base.base      = 0;	             //各通道静态基准值/上下限阀值
	ad_chn_base.base_down = 0;
	ad_chn_base.base_up   = 0;
	ad_chn_over           = 0x00;	         //各通道连续采样点(均衡后)的阀值判定：均在范围内
	md_point              = 0;               //用于基准值跟踪的计量点数
	ad_alarm_tick         = 0x00;

	ad_alarm_exts   = 0;	  				 //外力报警标志（无mask）: 无
	alarm_led_flag  = 0;	                 //所有报警LED为灭
	ad_alarm_flag   = 0;

	//adc硬件初始化
	adc_init();
}

void adc_task(void)
{
	Uint16  temp16;         //临时变量
	Uint16  val_temp;       //新送入的10bit采样值,  后作临时变量
	Uint16  val;            //4点均衡后得到的平均采样值, 作为一个可进行超限判断的最小点

	if (ad_sample.valid) {  //有新采样数据到达
		// 0. 保存到临时变量
		val_temp = ad_sample.val;

		// 1. 保存到均衡去嘈声求和中
		ad_samp_equ.sum += val_temp;
		ad_samp_equ.point++;

		// 2. 当前通道是否满足去嘈声点数
		if (ad_samp_equ.point == AD_EQU_PNUM) {
			// 已满去嘈声点数，可求出均衡后的一个点
			// 2.a 求出对应通道的一个采样点
			val = ad_samp_equ.sum >> 2;  //除于4

			// 2.b 清零当前通道的去嘈声求和结构
			ad_samp_equ.sum = 0;
			ad_samp_equ.point = 0;

			// 2.c 保存实时采样值
			ad_chn_sample.w = val;   //保存到最新一轮采样值数组中

			// 2.d 由系统状态决定数据的处理
			switch (system_status)
			{
			case SYS_PowerON:   //上电
			case SYS_B5S:       //5秒延时
				break;

			case SYS_SAMP_BASE: //初始上电时的静态基准值采样
				//存入阶段和
				ad_samp_sum.sum += val;
				ad_samp_pnum++;
				if (ad_samp_pnum == 32) {
					//已经满基准值采样点数（每通道32点)
					//1.计算均值和上下限
					//基准
					ad_chn_base.base = ad_samp_sum.sum >> 5;   //除于32

					//下限 = 基准 * （1 / 3）
					val_temp = ad_chn_base.base;
                    ad_chn_base.base_down = val_temp - ad_still_Dup;

					//上限
					if ((1023 - ad_chn_base.base) > ad_still_Dup) {
						ad_chn_base.base_up = ad_chn_base.base + ad_still_Dup;
					} else {
						ad_chn_base.base_up = 1023;
					}

					//复位阶段和变量，准备用于自适应阀值跟踪
					ad_samp_sum.sum   = 0;
					ad_samp_sum.point = 0;

					//2.e 状态-> 实时检测
					system_status = SYS_CHECK;
				}

				break;

			case SYS_CHECK: //实时检测
				//2. 判断是否外力报警
				ad_chn_over = ad_chn_over << 1;   //Bit0填0，因此缺省在允许范围内
				if ((val >= ad_chn_base.base_down) && (val <= ad_chn_base.base_up)) {
					//在张力上/下限允许范围内
					//a. 清标志(缺省)
					//b. 计入跟踪基准值求和中
					ad_samp_sum.sum += val;
					ad_samp_sum.point++;

					if (ad_samp_sum.point == 2) {
						//满2点(约需0.6秒)
						//b.0 计算这2点均值
						val_temp = ad_samp_sum.sum >> 1;   //除于2, 得到这2点的均值
						//b.1 更新基准值
						if (ad_chn_base.base > (val_temp + 1)) {                            
                            // 至少小于2,缓慢松弛
							md_point++;
							if (md_point >= DEF_ModiBASE_PT) {
                                // 已满缓慢松弛时的连续计量点数, 进行一次跟踪
								// 1. 跟踪基准值
                               if (ad_chn_base.base > 0) {
                                    //可以递减1
                                    ad_chn_base.base--;
                                    // 同步更新上下限
                                    val_temp = ad_chn_base.base;
                                    if ((val_temp - ad_still_Dup) > 0) {
                                        ad_chn_base.base_down = val_temp - ad_still_Dup;
                                    }
                                    
                                    if (ad_chn_base.base_up > 0) {
                                        ad_chn_base.base_up--;
                                    }
                                }
								// 2. 清缓慢松弛跟踪变量
								md_point = 0;
							}
						} else if (val_temp > (ad_chn_base.base + 1)) {
							// 至少大2, 缓慢张紧
							md_point++;
							if (md_point >= DEF_ModiBASE_PT) {
								// 已满缓慢张紧时的连续计量点数, 进行一次跟踪
								// 1. 跟踪基准值
								if (ad_chn_base.base < 1023) {
									//可以递增1
									ad_chn_base.base++;
									// 同步更新上下限
									val_temp = ad_chn_base.base;
                                    ad_chn_base.base_down = val_temp - ad_still_Dup;
									if (ad_chn_base.base_up < 1023) {
										ad_chn_base.base_up++;
									}
								}
								// 2. 清缓慢张紧跟踪变量
								md_point = 0;
							}
						}

						//b.2 复位阶段和变量 - 用于4点4点平均的求和结构
						ad_samp_sum.sum   = 0;
						ad_samp_sum.point = 0;
					}
				} else {
					//外力报警, 置标志
					ad_chn_over |= 0x01;
				}

				//2.4 连续4点超范围，此通道有外力报警
				if ((ad_chn_over & 0x0F) == 0x0F) {
					//置外力报警标志
					ad_alarm_exts |= (Uint16)0x01;

					//报警计时tick清零
					ad_alarm_tick = 0;

					//立即更新机制
					ad_chn_base.base = val;
					val_temp = ad_chn_base.base;
                    ad_chn_base.base_down = val_temp - ad_still_Dup;
					if ((1023 - ad_chn_base.base) > ad_still_Dup) {
						ad_chn_base.base_up = ad_chn_base.base + ad_still_Dup;
					} else {
						ad_chn_base.base_up = 1023;
					}

					//复位阶段和变量 - 用于4点4点平均的求和结构
					ad_samp_sum.sum = 0;
					ad_samp_sum.point = 0;

					//清缓慢张紧跟踪变量
					md_point = 0;   //用于基准值跟踪的计量点数
				} else if ((ad_chn_over & 0x0F) == 0x00) {
					//无外力报警
					//检查报警时间是否已到
					if (ad_alarm_tick > ALARM_TEMPO) {
						//报警已经到最大报警时间, 停止报警
						ad_alarm_exts &= ~((Uint16)0x01);
					}
				}

				//3 LED指示(无mask)
				temp16 = ad_alarm_exts & 0x0001;
				if (alarm_led_flag != temp16) {
					//LED指示信息有变化
					alarm_led_flag = temp16;
					P33 = !P33;
				}

				//4 更新报警标志
				temp16 = ad_alarm_exts & 0x0001;
				if (temp16 == 0) {
					ad_alarm_flag = 0;   //无报警
				} else {
					ad_alarm_flag = 1;   //有报警
				}

				break;
			}

			//3.当前采样值处理完毕，允许新的采样值输入
			ad_sample.valid = FALSE;
		}
	}
}