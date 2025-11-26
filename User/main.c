#include "myHeader.h"
#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846f
#define SQRT2 1.41421356237309504880f

/* ---------- 巴特沃斯滤波器结构体与实现（文件级） ---------- */
typedef struct
{
	float b0, b1, b2; /* 分子系数 */
	float a1, a2;     /* 数字滤波器的 a1, a2 （注意在实现中为负号形式）*/
	float x1, x2;     /* 输入延迟样本 x[n-1], x[n-2] */
	float y1, y2;     /* 输出延迟样本 y[n-1], y[n-2] */
} Butterworth;

/* 初始化 2 阶巴特沃斯低通双二阶（biquad）滤波器
   fc: 截止频率 (Hz)
   fs: 采样频率 (Hz)
*/
void Butterworth_Init(Butterworth *f, float fc, float fs)
{
	float omega = tanf((float)PI * fc / fs); /* 预扭曲 */
	float k1 = (float)SQRT2 * omega; /* sqrt(2) * omega */
	float k2 = omega * omega;
	float a0 = 1.0f + k1 + k2;

	f->b0 = k2 / a0;
	f->b1 = 2.0f * k2 / a0;
	f->b2 = k2 / a0;
	f->a1 = 2.0f * (k2 - 1.0f) / a0; /* 在滤波器实现时将用作减项 */
	f->a2 = (1.0f - k1 + k2) / a0;

	/* 清零历史状态 */
	f->x1 = f->x2 = 0.0f;
	f->y1 = f->y2 = 0.0f;
}

/* 对单个样本进行滤波，返回滤波后值（浮点）
   使用标准差分方程：
   y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
*/
float Butterworth_Filter(Butterworth *f, float x)
{
	float y = f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2 - f->a1 * f->y1 - f->a2 * f->y2;

	/* 更新历史 */
	f->x2 = f->x1;
	f->x1 = x;
	f->y2 = f->y1;
	f->y1 = y;

	return y;
}

/* 创建三个滤波器实例用于三个 ADC 通道（文件级变量） */
Butterworth bw1, bw2, bw3;
/* 这里选择采样率 fs = 100 Hz（采样间隔 10 ms），截止频率 fc = 10 Hz（可按需调整） */
const float sample_fs = 100.0f;
const float cutoff_fc = 10.0f;

/* 在定时器中断中使用的变量需要设为 volatile */
volatile int16_t speed1, speed2;
volatile uint16_t adcf1, adcf2, adcf3;

/* 定时器初始化: 使用 TIM2 产生 100Hz 更新中断 (周期 10ms) */
static void TIM2_Int_Init(void)
{
	/* 假设 APB1 定时器时钟为 36MHz。PSC = 36000-1 => 1kHz 计数频率 (1ms tick)，ARR = 10-1 => 10ms 周期 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_TimeBaseInitTypeDef tb;
	tb.TIM_Period = 10 - 1;           /* 自动重装载值 */
	tb.TIM_Prescaler = 36000 - 1;     /* 预分频 */
	tb.TIM_ClockDivision = TIM_CKD_DIV1;
	tb.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &tb);

	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = TIM2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1; /* 根据工程优先级需求调整 */
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	TIM_Cmd(TIM2, ENABLE);
}

/* TIM2 更新中断服务函数：执行原来循环中的周期性任务 */
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		/* 读取原始 ADC 值（整数） */
		u16 adcx1 = T_Get_Adc_Average(THRD1_ADC_CH0, 10);
		u16 adcx2 = T_Get_Adc_Average(THRD2_ADC_CH0, 10);
		u16 adcx3 = T_Get_Adc_Average(THRD3_ADC_CH0, 10);

/* 转为浮点并进行巴特沃斯滤波 */
		float f1 = Butterworth_Filter(&bw1, (float)adcx1);
		float f2 = Butterworth_Filter(&bw2, (float)adcx2);
		float f3 = Butterworth_Filter(&bw3, (float)adcx3);

		if (f1 < 0.0f) f1 = 0.0f;
		if (f2 < 0.0f) f2 = 0.0f;
		if (f3 < 0.0f) f3 = 0.0f;

		adcf1 = (u16)(f1 + 0.5f);
		adcf2 = (u16)(f2 + 0.5f);
		adcf3 = (u16)(f3 + 0.5f);
		speed1 = Motor1_getSpeed() / 4;
		speed2 = Motor2_getSpeed() / 4;
        thrd_correct();
        thrdPID();
		/* 更新显示（注意：OLED 与串口操作可能较耗时，若影响实时性可改为设置标志在主循环中处理） */
		OLED_ShowNum(1, 1, adcf1, 4, OLED_8X16);
		OLED_ShowNum(1, 18, adcf2, 4, OLED_8X16);
		OLED_ShowNum(1, 36, adcf3, 4, OLED_8X16);
		OLED_ShowSignedNum(55, 1, speed1, 4, OLED_8X16);
		OLED_ShowSignedNum(55, 17,speed2, 4, OLED_8X16);

        OLED_ShowNum(30, 1, g_thrd_correct_wip, 1,OLED_6X8);
//        OLED_ShowNum(60, 30, thrd_BLACK, 4, OLED_6X8);
        //OLED_ShowNum(60, 39, thrd_WHITE, 4, OLED_6X8);
        OLED_ShowFloatNum(60, 31, v1, 1, 1, OLED_6X8);
        OLED_ShowFloatNum(60, 40, v2, 1, 1, OLED_6X8);
        OLED_ShowFloatNum(60, 49, v3, 1, 1, OLED_6X8);
        OLED_ShowFloatNum(90, 49, offset, 1, 1, OLED_6X8);
		OLED_Update();



	//Serial_mySend(speed1, speed2, offset);



		if (Serial_RxFlag == 1)
		{
			OLED_ShowString(1, 1, Serial_RxPacket, OLED_6X8);
			OLED_Update();
			Serial_SendString(Serial_RxPacket);

            	float data = 0;
				char Cmd;
            	sscanf(Serial_RxPacket, "%c%f", &Cmd, &data);
				//OLED_ShowString(2, 5, Cmd);
                //OLED_ShowString(10, 1, Rx_buf, OLED_6X8);
				if(Cmd == 'S') {

                }
				//else if(Cmd == 'i') Motor1_Data.Ki = data;
				//else if(Cmd == 'p') Motor1_Data.Kp = data;
				//else if(Cmd == 'd') Motor1_Data.Kd = data;
				else if(Cmd == 'i') Ki = data;
				else if(Cmd == 'p') Kp = data;
				else if(Cmd == 'd') Kd = data;
				OLED_ShowFloatNum(33, 1, Kp,1, 2, OLED_6X8);
				OLED_ShowFloatNum(33, 9, Ki,1, 2, OLED_6X8);
				OLED_ShowFloatNum(33, 18, Kd,1, 2, OLED_6X8);


			Serial_RxFlag = 0;
		}
	}
}

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    T_Adc_Init();
    thrd_Init();
    OLED_Init();
    Serial_Init();
    Encoder_Init();

    /* 初始化滤波器系数 */
    Butterworth_Init(&bw1, cutoff_fc, sample_fs);
    Butterworth_Init(&bw2, cutoff_fc, sample_fs);
    Butterworth_Init(&bw3, cutoff_fc, sample_fs);

    /* 初始化定时器中断代替循环延时实现 100Hz 任务 */
    TIM2_Int_Init();

    /* 主循环不再执行周期性采样，保留用于低优先级处理（可扩展） */
    while (1)
    {
        /* 若耗时的显示/串口操作需要移出中断，可在此基于标志处理 */
        /* 当前实现全部在 TIM2_IRQHandler 中 */
    }
}

