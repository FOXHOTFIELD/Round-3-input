
#include "myHeader.h"
#include "IIC.h"
#include <math.h>

/* ---------- 巴特沃斯滤波器结构体与实现（集成到 thrd 模块） ---------- */
typedef struct
{
	float b0, b1, b2; /* 分子系数 */
	float a1, a2;     /* 数字滤波器的 a1, a2 （实现中为减项）*/
	float x1, x2;     /* 输入延迟样本 x[n-1], x[n-2] */
	float y1, y2;     /* 输出延迟样本 y[n-1], y[n-2] */
} Butterworth;

static void Butterworth_Init(Butterworth *f, float fc, float fs)
{
	const float PI = 3.14159265358979323846f;
	const float SQRT2 = 1.41421356237309504880f;
	float omega = tanf((float)PI * fc / fs); /* 预扭曲 */
	float k1 = (float)SQRT2 * omega;
	float k2 = omega * omega;
	float a0 = 1.0f + k1 + k2;

	f->b0 = k2 / a0;
	f->b1 = 2.0f * k2 / a0;
	f->b2 = k2 / a0;
	f->a1 = 2.0f * (k2 - 1.0f) / a0; /* 在实现时为减项 */
	f->a2 = (1.0f - k1 + k2) / a0;

	f->x1 = f->x2 = 0.0f;
	f->y1 = f->y2 = 0.0f;
}

static float Butterworth_Filter(Butterworth *f, float x)
{
	float y = f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2 - f->a1 * f->y1 - f->a2 * f->y2;
	f->x2 = f->x1;
	f->x1 = x;
	f->y2 = f->y1;
	f->y1 = y;
	return y;
}

/* 三个滤波器实例（对应三路 thrd） */
static Butterworth bw1, bw2, bw3;
/* 采样率与截止频率，保持与 main 以前使用的默认值一致 */
/* 采样间隔（ms）和滤波器参数：将采样间隔抽成宏，滤波器按实际采样率计算 */
#define THRD_SAMPLE_MS 200
static const float sample_fs = 1000.0f / (float)THRD_SAMPLE_MS; /* Hz */
static const float cutoff_fc = 10.0f;  /* Hz */

/*
 * 说明：
 *  - 该文件在 FreeRTOS 下启动一个周期性任务，定时采集三路 thrd(通过 ADC)
 *    和编码器增量值，并将打包的数据放入 IIC 从机发送缓冲区，供主机读取。
 *  - 数据格式（小端）：
 *      uint16_t thrd1, thrd2, thrd3; int16_t encoder;
 */

static void vThrd_EncoderTask(void *pvParameters);

void thrd_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 配置 ADC 引脚为模拟输入 */
	RCC_APB2PeriphClockCmd(THRD_GPIO_CLK,ENABLE);// 使能 PORTA 时钟
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;    // 模拟输入

	GPIO_InitStructure.GPIO_Pin=THRD1_GPIO_PIN;
	GPIO_Init(THRD_PORT,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=THRD2_GPIO_PIN;
	GPIO_Init(THRD_PORT,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=THRD3_GPIO_PIN;
	GPIO_Init(THRD_PORT,&GPIO_InitStructure);

	/* 初始化 ADC、编码器与 IIC（从机） */
	T_Adc_Init();
	Encoder_Init();
	/* 选择从机地址（例如 0x30），可按需修改 */
	IIC_Slave_Init(0x48);

	/* 初始化三路巴特沃斯滤波器（将滤波放在 thrd 模块中处理） */
	Butterworth_Init(&bw1, cutoff_fc, sample_fs);
	Butterworth_Init(&bw2, cutoff_fc, sample_fs);
	Butterworth_Init(&bw3, cutoff_fc, sample_fs);

	/* 创建 FreeRTOS 任务：周期性采集并更新 I2C 发送缓冲 */
	//xTaskCreate(vThrd_EncoderTask, "ThrdTask", 256/sizeof(StackType_t), NULL, tskIDLE_PRIORITY+2, NULL);
}

static void vThrd_EncoderTask(void *pvParameters)
{
	//(void)pvParameters;
	//const TickType_t xDelay = pdMS_TO_TICKS(THRD_SAMPLE_MS); // 采样间隔，同 THRD_SAMPLE_MS
	uint8_t txbuf[8];

	//for(;;){
		/* 使用多次平均采样获得更稳定的输入（与 main 之前行为一致，times = 10） */
		uint16_t raw1 = T_Get_Adc_Average(THRD1_ADC_CH0, 10);
		uint16_t raw2 = T_Get_Adc_Average(THRD2_ADC_CH0, 10);
		uint16_t raw3 = T_Get_Adc_Average(THRD3_ADC_CH0, 10);
		int16_t enc = Motor1_getSpeed();

		/* 对平均后的原始值进行巴特沃斯滤波（浮点） */
		float f1 = Butterworth_Filter(&bw1, (float)raw1);
		float f2 = Butterworth_Filter(&bw2, (float)raw2);
		float f3 = Butterworth_Filter(&bw3, (float)raw3);

		/* 限制滤波结果为非负并转换回无符号整数，保留与 ADC 相近的尺度 */
		if (f1 < 0.0f) f1 = 0.0f;
		if (f2 < 0.0f) f2 = 0.0f;
		if (f3 < 0.0f) f3 = 0.0f;

		uint16_t th1 = (uint16_t)(f1 + 0.5f);
		uint16_t th2 = (uint16_t)(f2 + 0.5f);
		uint16_t th3 = (uint16_t)(f3 + 0.5f);

		/* 显示三路原始 ADC 值，便于定位读数问题（若需显示滤波后值可改为 th1/th2/th3） */
		OLED_ShowNum(1, 1, raw1, 4, OLED_8X16);
		OLED_ShowNum(1, 9, raw2, 4, OLED_8X16);
		OLED_ShowNum(1, 17, raw3, 4, OLED_8X16);
		/* 显示 IIC 中断计数（用于确认 EXTI9_5_IRQHandler 是否被调用），不使用 printf */
		OLED_ShowNum(1, 25, IIC_GetIrqCount(), 5, OLED_8X16);
        OLED_Update();

		/* 小端打包： th1, th2, th3, enc （发送的是滤波后的 thrd 值） */
		txbuf[0] = (uint8_t)(th1 & 0xFF);
		txbuf[1] = (uint8_t)((th1 >> 8) & 0xFF);
		txbuf[2] = (uint8_t)(th2 & 0xFF);
		txbuf[3] = (uint8_t)((th2 >> 8) & 0xFF);
		txbuf[4] = (uint8_t)(th3 & 0xFF);
		txbuf[5] = (uint8_t)((th3 >> 8) & 0xFF);
		txbuf[6] = (uint8_t)(enc & 0xFF);
		txbuf[7] = (uint8_t)((enc >> 8) & 0xFF);

		/* 将数据拷贝到 IIC 从机发送缓冲区，主机读取时即可获得最新滤波后数据 */
		IIC_SetTxBuffer(txbuf, sizeof(txbuf));

		//vTaskDelay(xDelay);
	//}
}
void Thrd_EncoderTask(void)
{
	//(void)pvParameters;
	//const TickType_t xDelay = pdMS_TO_TICKS(THRD_SAMPLE_MS); // 采样间隔，同 THRD_SAMPLE_MS
	uint8_t txbuf[8];

	//for(;;){
		/* 使用多次平均采样获得更稳定的输入（与 main 之前行为一致，times = 10） */
		uint16_t raw1 = T_Get_Adc_Average(THRD1_ADC_CH0, 10);
		uint16_t raw2 = T_Get_Adc_Average(THRD2_ADC_CH0, 10);
		uint16_t raw3 = T_Get_Adc_Average(THRD3_ADC_CH0, 10);
		int16_t enc = Motor1_getSpeed() * 0;

		/* 对平均后的原始值进行巴特沃斯滤波（浮点） */
		float f1 = Butterworth_Filter(&bw1, (float)raw1);
		float f2 = Butterworth_Filter(&bw2, (float)raw2);
		float f3 = Butterworth_Filter(&bw3, (float)raw3);

		/* 限制滤波结果为非负并转换回无符号整数，保留与 ADC 相近的尺度 */
		if (f1 < 0.0f) f1 = 0.0f;
		if (f2 < 0.0f) f2 = 0.0f;
		if (f3 < 0.0f) f3 = 0.0f;

		uint16_t th1 = (uint16_t)(f1 + 0.5f);
		uint16_t th2 = (uint16_t)(f2 + 0.5f);
		uint16_t th3 = (uint16_t)(f3 + 0.5f);

		/* 显示三路原始 ADC 值，便于定位读数问题（若需显示滤波后值可改为 th1/th2/th3） */
		OLED_ShowNum(1, 1, th1, 4, OLED_8X16);
		OLED_ShowNum(1, 10+9, th2, 4, OLED_8X16);
		OLED_ShowNum(1, 20+17, th3, 4, OLED_8X16);
		/* 显示 IIC 中断计数（用于确认 EXTI9_5_IRQHandler 是否被调用），不使用 printf */
		OLED_ShowNum(1, 25, IIC_GetIrqCount(), 5, OLED_8X16);
        OLED_Update();

		/* 小端打包： th1, th2, th3, enc （发送的是滤波后的 thrd 值） */
        //th1 = 0;
		txbuf[0] = (uint8_t)(th1 & 0xFF)+1;
		txbuf[1] = (uint8_t)((th1 >> 8) & 0xFF)+1;
		txbuf[2] = (uint8_t)(th2 & 0xFF);
		txbuf[3] = (uint8_t)((th2 >> 8) & 0xFF);
		txbuf[4] = (uint8_t)(th3 & 0xFF);
		txbuf[5] = (uint8_t)((th3 >> 8) & 0xFF);
		txbuf[6] = (uint8_t)(enc & 0xFF);
		txbuf[7] = (uint8_t)((enc >> 8) & 0xFF);

		/* 将数据拷贝到 IIC 从机发送缓冲区，主机读取时即可获得最新滤波后数据 */
		IIC_SetTxBuffer(txbuf, sizeof(txbuf));

		//vTaskDelay(xDelay);
	//}
}
