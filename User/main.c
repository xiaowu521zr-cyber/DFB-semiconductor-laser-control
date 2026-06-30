#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include <AD.h>
#include <stdio.h>
#include <math.h>
#include <PWM.h>

#define NB -3
#define NM -2
#define NS -1
#define ZO 0
#define PS 1
#define PM 2
#define PB 3

float error = 0.0;       // 当前误差
float lastError = 0.0;   // 上一次的误差
float integral = 0.0;    // 积分项
float error_ec = 0.0;    // 微分项
float output = 0.0;      // PID输出
float setpoint = 35;     // 设定值（目标值）
float feedback = 0.0;    // 反馈值（实际值）
const float e = 2.7182;//自然常数
float Voltage;	  //AD采集到的电压

//float kp0 = 120;   // 比例系数基数
//float ki0 = 0.8;  // 积分系数基数
//float kd0 = 2000;  // 微分系数基数
//float kp0 = 80; 
//float ki0 = 0.1; 
//float kd0 = 500;
float kp0 = 50;   // 稍微降低一点动力，60 够了
float ki0 = 0.5; // 积分要极其微弱，靠时间慢慢累积消除稳态即可
float kd0 = 50;

float kp ;  // 比例系数
float ki ;  // 积分系数
float kd ;  // 微分系数

float RTH;//热敏电阻阻值
int value;//PWM波占空比

float e_min=-10,e_max=10;	   //定义误差值最大最小值
//float ec_min=-0.34,ec_max=0.02;  //定义误差值变化率最大最小值
float ec_min = -0.5, ec_max = 0.5;  // 让 0 永远映射为 0 (ZO)
//float kp_max=120,ki_max=1.2,kd_max=10000;   //定义各个系数最大值
//float kp_min=-120,ki_min=-1.2,kd_min=-10000; //定义误差值变化率最大最小值
float kp_max=40,  ki_max=0.05, kd_max=1000;   // 缩小模糊算法的干预范围
float kp_min=-40, ki_min=-0.05, kd_min=-400;

float qerror_e,qerror_ec; //误差和误差值在所设范围内量化值
float error,error_ec;//误差值和误差变化率
int i_e,i_ec;//qerror的整数部分
float m_e,m_ec;//qerror的小数部分
float affiliation_e[2];//定义误差值隶属度概率值
float affiliation_ec[2];//定义误差值变化率隶属度概率值
float detail_kp,detail_ki,detail_kd;//解模糊后pid各个系数值
int fuzzypoint_e[2]; //查表中误差值对应坐标
int fuzzypoint_ec[2];//查表中误差值变化率对应坐标

float q;
float b;
float z;

float filtered;
	
#define WINDOW_SIZE 10 // 窗口大小可调
float sliding_mean_filter(float new_data) 
{
    static float window[WINDOW_SIZE] = {0};  // 静态数组存储窗口数据
    static int index = 0;                    // 当前写入位置索引
    static float sum = 0;                    // 窗口数据总和
    static int count = 0;                    // 当前有效数据量
    
    // 更新窗口数据：减去即将被替换的旧值，加上新值
    sum = sum - window[index] + new_data;
    
    // 更新窗口内容
    window[index] = new_data;
    
    // 环形索引更新
    index = (index + 1) % WINDOW_SIZE;
    
    // 更新有效数据计数（窗口未填满时保持增长）
    count = (count < WINDOW_SIZE) ? count + 1 : WINDOW_SIZE;
    
    return sum / count;  // 返回均值（自动处理初始阶段）
}


float Quantization(float maximum,float minimum,float x)//量化函数
{
	if(x>maximum)
		x=maximum;
	if(x<minimum)
		x=minimum;
	return 6.0 *(x-minimum)/(maximum - minimum)-3;
}

float Inverse_quantization(float maximum, float minimum, float qvalues)//解模糊
{
	return (maximum - minimum) *(qvalues + 3)/6 + minimum;
}

void Getaffiliationandfuzzypoint()//获得概率值和查表坐标
{
	if(qerror_e>=0)
	{
		affiliation_e[0]=m_e;            // 隶属度1 = 当前误差的隶属度值（如正区间的隶属度）
		affiliation_e[1]=1-m_e;			 // 隶属度2 = 1 - 隶属度1（相邻模糊集合的互补隶属度）
		fuzzypoint_e[0]=i_e+4;	         // 查表坐标1 = 基础索引i_e + 4（指向正区间的规则行或列）
		fuzzypoint_e[1]=i_e+3;			 // 查表坐标2 = 基础索引i_e + 3（指向相邻模糊集合的规则）
	}		
	
	else
	{
		affiliation_e[0]=-m_e;           // 隶属度1 = -当前误差隶属度值
		affiliation_e[1]=1+m_e;          // 隶属度2 = 1 + 隶属度1（互补计算，需确保值合法）
		fuzzypoint_e[0]=i_e+2;	         // 查表坐标1 = 基础索引i_e + 2（指向负区间的规则行或列）
		fuzzypoint_e[1]=i_e+3;           // 查表坐标2 = 基础索引i_e + 3（相邻模糊集合的规则）
	}
	
	if(qerror_ec>=0)
	{
		affiliation_ec[0]=m_ec;
		affiliation_ec[1]=1-m_ec;
        fuzzypoint_ec[0]=i_ec+4;	
        fuzzypoint_ec[1]=i_ec+3;
	}
	else
	{
		affiliation_ec[0]=-m_ec;
		affiliation_ec[1]=1+m_ec;
        fuzzypoint_ec[0]=i_ec+2;	
        fuzzypoint_ec[1]=i_ec+3;
	}
}
void getFeedbackValue()//计算温度反馈值
{

	Voltage = (float)AD_GetValue()/4095 * 3.3;
//	sliding_mean_filter(Voltage);
	filtered = sliding_mean_filter(Voltage);
	RTH=55.44/(0.833*filtered+2.31)-12;     //热敏电阻阻值和AD转换的关系
	feedback=90.93-56.38*(1-powf(e,-RTH/19.6344))-45.04*(1-powf(e,-RTH/2.986));
}	
	
void PID_control() //pid输出和PWM占空比关系	
{  
	if (output >= 0) 
    {
        // 【加热模式】：温度低于目标，需要加热
        GPIO_SetBits(GPIOB, GPIO_Pin_5);
        GPIO_ResetBits(GPIOB, GPIO_Pin_6); 
        value = output; 
    } 
    else 
    {	
        // 【制冷模式】：温度超调，主动制冷拉回
        GPIO_ResetBits(GPIOB, GPIO_Pin_5);
        GPIO_SetBits(GPIOB, GPIO_Pin_6); 
        value = -output; // PWM寄存器只认正数占空比
    }

    // PWM 最大输出限幅（满载1000）
    if (value > 1000) {
        value = 1000;
    }
}

void Drive()//开放PB5与PB6作为TEC驱动信号
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_b;
	GPIO_b.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_b.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_5;
	GPIO_b.GPIO_Speed =GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_b); 
}

//void PID_calculation() //PID各项系数的计算
//{
//	
//	
//	integral += error;   // 计算积分项
//	
//	
//	output=kp*error+ki*integral+kd*error_ec;
//}


int main(void)
{
	Serial_Init();//PA9
	OLED_Init();//PB8 PB9
	AD_Init();//PA1
	PWM_Init();
	Drive();
//	GPIO_SetBits(GPIOB, GPIO_Pin_5);
//	GPIO_ResetBits(GPIOB, GPIO_Pin_6);
	
	while(1)
	{
		getFeedbackValue();//采集端正常
		error=setpoint - feedback;//误差正常

		qerror_e = Quantization(e_max, e_min, error);	   //将误差 error 映射到论域中
		i_e= (int) qerror_e;
			
		m_e=qerror_e-(int)qerror_e;
		

		//获取误差值变化率
		error_ec = error - lastError;
		lastError = error;
		
		qerror_ec = Quantization(ec_max, ec_min, error_ec);	  //将误差变化 error_ec 映射到论域中
		
		i_ec= (int)qerror_ec;
		m_ec=qerror_ec-(int)qerror_ec;
		
		Getaffiliationandfuzzypoint();
		
		// 设置e和ec范围尽量对称，不然e=0不等效于qe=0
		
		// e>0:P>0,使e向0
		// e<0:P<0,使e向0	
		int Kp_rule_list[7][7] = { 
        //-3  -2  -1   0   1   2   3  
   /*-3*/{PB, PB, PB, PB, PB, PB, PB}, // 超调太热，猛烈增加Kp进行制冷
   /*-2*/{PM, PM, PM, PM, PM, PM, PM},
   /*-1*/{PS, PS, PS, PS, PS, PS, PS},
    /*0*/{ZO, ZO, ZO, ZO, ZO, ZO, ZO}, // 完美对称：目标区绝对静止！
    /*1*/{PS, PS, PS, PS, PS, PS, PS},
    /*2*/{PM, PM, PM, PM, PM, PM, PM},
    /*3*/{PB, PB, PB, PB, PB, PB, PB}};// 没到目标，猛烈增加Kp进行加热

        int Ki_rule_list[7][7] = {
   /*-3*/{NB, NB, NB, NB, NB, NB, NB},
   /*-2*/{NM, NM, NM, NM, NM, NM, NM},
   /*-1*/{NS, NS, NS, NS, NS, NS, NS},
    /*0*/{ZO, ZO, ZO, ZO, ZO, ZO, ZO},
    /*1*/{NS, NS, NS, NS, NS, NS, NS},
    /*2*/{NM, NM, NM, NM, NM, NM, NM},
    /*3*/{NB, NB, NB, NB, NB, NB, NB}};

        int Kd_rule_list[7][7] = { 
   /*-3*/{ZO, ZO, ZO, ZO, ZO, ZO, ZO},
   /*-2*/{ZO, ZO, ZO, ZO, ZO, ZO, ZO},
   /*-1*/{ZO, ZO, ZO, ZO, ZO, ZO, ZO},
    /*0*/{ZO, ZO, ZO, ZO, ZO, ZO, ZO}, 
    /*1*/{ZO, ZO, ZO, ZO, ZO, ZO, ZO},
    /*2*/{ZO, ZO, ZO, ZO, ZO, ZO, ZO},
    /*3*/{ZO, ZO, ZO, ZO, ZO, ZO, ZO}};
//		int Kp_rule_list[7][7] = { 
//        //-3  -2  -1   0   1   2   3  误差
//   /*-3*/{ZO, ZO, ZO, ZO, ZO, ZO, ZO},
//   /*-2*/{ZO, ZO, ZO, ZO, ZO, PS, PS},
//   /*-1*/{ZO, ZO, ZO, ZO, ZO, PS, PS},
//   /*0*/ {ZO, ZO, ZO, ZO, ZO, ZO, ZO},
////    /*0*/{PS, PS, PS, PS, PS, PS, PS},  // 核心稳定区
//    /*1*/{PB, PB, PB, PB, PB, PB, PB},
//    /*2*/{PB, PB, PB, PB, PB, PB, PB},
//    /*3*/{PB, PB, PB, PB, PB, PB, PB}};
////  误差变化率 积分项基本为正，只有最初超调时可能为负

//		int Ki_rule_list[7][7] = {
//		 //-3  -2  -1   0   1   2   3	
//    /*-3*/{ZO, ZO, ZO, ZO, ZO, ZO, ZO},
//    /*-2*/{ZO, ZO, ZO, ZO, PS, PS, PS},
//    /*-1*/{ZO, ZO, ZO, ZO, PS, PS, PS},
//     /*0*/{ZO, ZO, ZO, ZO, ZO, ZO, ZO},  // 核心稳定区
//     /*1*/{ZO, ZO, ZO, ZO, ZO, ZO, ZO},
//     /*2*/{ZO, ZO, ZO, ZO, ZO, ZO, ZO},
//     /*3*/{ZO, ZO, ZO, ZO, ZO, ZO, ZO}};
//		// e大，减小d
//		// 左边ec小于0，d促进降温
//		int Kd_rule_list[7][7] = { 
//		 //-3  -2  -1   0   1   2   3	
//    /*-3*/{ZO, ZO, ZO, ZO, ZO, ZO, ZO},
//    /*-2*/{ZO, ZO, ZO, ZO, ZO, ZO, ZO},
//    /*-1*/{ZO, ZO, PS, ZO, ZO, ZO, ZO},
//     /*0*/{ZO, ZO, ZO, ZO, ZO, ZO, ZO}, // 核心稳定区
//     /*1*/{NM, NM, NM, NM, NM, NM, NM},
//     /*2*/{NB, NB, NB, NB, NB, NB, NB},
//     /*3*/{NB, NB, NB, NB, NB, NB, NB}};	
			
		float qdetail_kp=affiliation_e[0]*affiliation_ec[0]*Kp_rule_list[fuzzypoint_e[0]][fuzzypoint_ec[0]]
					+affiliation_e[0]*affiliation_ec[1]*Kp_rule_list[fuzzypoint_e[0]][fuzzypoint_ec[1]]
					+affiliation_e[1]*affiliation_ec[0]*Kp_rule_list[fuzzypoint_e[1]][fuzzypoint_ec[0]]
					+affiliation_e[1]*affiliation_ec[1]*Kp_rule_list[fuzzypoint_e[1]][fuzzypoint_ec[1]];    //去模糊化得到增量 △kp
		float qdetail_ki= affiliation_e[0]*affiliation_ec[0]*Ki_rule_list[fuzzypoint_e[0]][fuzzypoint_ec[0]]      //去模糊化得到增量 △ki
				+affiliation_e[0]*affiliation_ec[1]*Ki_rule_list[fuzzypoint_e[0]][fuzzypoint_ec[1]]
					+affiliation_e[1]*affiliation_ec[0]*Ki_rule_list[fuzzypoint_e[1]][fuzzypoint_ec[0]]
					+affiliation_e[1]*affiliation_ec[1]*Ki_rule_list[fuzzypoint_e[1]][fuzzypoint_ec[1]];   
		float qdetail_kd = affiliation_e[0]*affiliation_ec[0]*Kd_rule_list[fuzzypoint_e[0]][fuzzypoint_ec[0]]      //去模糊化得到增量 △ki
				+affiliation_e[0]*affiliation_ec[1]*Kd_rule_list[fuzzypoint_e[0]][fuzzypoint_ec[1]]
					+affiliation_e[1]*affiliation_ec[0]*Kd_rule_list[fuzzypoint_e[1]][fuzzypoint_ec[0]]
				+affiliation_e[1]*affiliation_ec[1]*Kd_rule_list[fuzzypoint_e[1]][fuzzypoint_ec[1]];   
		
		detail_kp=Inverse_quantization(kp_max, kp_min, qdetail_kp);
		detail_ki=Inverse_quantization(ki_max, ki_min, qdetail_ki);
		detail_kd=Inverse_quantization(kd_max, kd_min, qdetail_kd);

		kp=kp0+detail_kp;    //得到最终的 kp 值
		ki=ki0+detail_ki;    //得到最终的 ki 值
		kd=kd0+detail_kd;    //得到最终的 kd 值
		
		// 【修复 3】进阶绝杀：±0.3℃绝对静默区！关闭模糊跳动，享受纯线性丝滑！
        if (error > -0.3 && error < 0.3) {
            kp = kp0;
            ki = ki0;
            kd = kd0;
        }
		
		if (kp < 0)
			kp = 0;
		if (ki < 0)
			ki = 0;
		if (kd < 0)
			kd = 0;
		
		// 【修改点】积分分离与抗积分饱和 (Anti-Windup)
//        float integral_max = 800;  // 设定积分上限（防止过热冲到40多度）
//        float integral_min = -800; // 设定积分下限
		float integral_max = 2000;  // 设定积分上限
		float integral_min = -2000; // 设定积分下限	
		if(feedback>=0.8*setpoint)
		{
			float temp_output = kp * error + kd * error_ec;
			if (temp_output < 1000 && temp_output > -1000) {
                integral += error;
            }
//			integral += error;
			// 执行抗饱和限幅
            if(integral > integral_max) integral = integral_max;
            else if(integral < integral_min) integral = integral_min;
//			if((error<=0.5&&error>=0.05))     // (error<=-0.02&&error>=-0.5)
//				integral += 2.5*error;
//			else if((error<=-0.01&&error>-0.5)||(error<0.05&&error>=0.01))
//				integral+= 4*error;
			output=kp*error+ki*integral+kd*error_ec;
		}
		else
		{
			integral = 0;
			output=kp*error+kd*error_ec;
		}
		q=kp*error;
		b=ki*integral;
		z=kd*error_ec;
		PID_control();
		PWM_SetCompare1(value);//PA0输出
		
//		printf("%.3f\t%.3f\t%f\t%f\r\n",qerror_e,qerror_ec,feedback,error_ec);
		printf("%.3f\r\n", feedback);
		//printf("%f,%f,%f,%f,%f\r\n",feedback,kp,ki,kd,output);
		//printf("%f,%f,%f,%f,%f,%f,%d\r\n",feedback,q,b,z,qerror_e,qerror_ec,value);
		Delay_ms(50);
	}		
}		
		
		
