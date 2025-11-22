#include "myHeader.h"

/* 简单示例：在 STM32F103C8 (BluePill) 上运行 FreeRTOS
   - 使用 PC13 作为 LED 输出（常见板载 LED）
   - 使用 USART1 通过 Serial_Printf 打印信息
   先复用工程中已有的 Serial_Init / Serial_Printf 接口 */

/* 更改 LED 引脚：将原来的 PC13 换为 PA5（示例） */
#define LED_PORT GPIOA
#define LED_PIN  GPIO_Pin_5

//static void LED_Init(void)
//{
	///* 开启 GPIOA 时钟（PA5） */
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//GPIO_InitTypeDef GPIO_InitStructure;
	//GPIO_InitStructure.GPIO_Pin = LED_PIN;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_Init(LED_PORT, &GPIO_InitStructure);
	///* 初始清零（按需改为 Set），便于观察变化 */
	//GPIO_ResetBits(LED_PORT, LED_PIN);
//}

//static void vTaskBlink(void *pvParameters)
//{
	//(void)pvParameters;
	//for (;;)
	//{
		///* 翻转引脚：GPIO_ReadOutputDataBit 返回 uint8_t，使用 uint8_t 消除枚举/整型混合警告 */
		//uint8_t current = GPIO_ReadOutputDataBit(LED_PORT, LED_PIN);
		//if (current == Bit_SET)
		//{
			//GPIO_WriteBit(LED_PORT, LED_PIN, Bit_RESET);
		//}
		//else
		//{
			//GPIO_WriteBit(LED_PORT, LED_PIN, Bit_SET);
		//}
		//vTaskDelay(pdMS_TO_TICKS(500));
	//}
//}

//static void vTaskPrint(void *pvParameters)
//{
	//(void)pvParameters;
	//unsigned long cnt = 0;
	//for (;;)
	//{
		//Serial_Printf("FreeRTOS running: %lu\r\n", cnt++);
		//vTaskDelay(pdMS_TO_TICKS(1000));
	//}
//}

#define DEBUG

#ifndef DEBUG
int main(void)
{
	/* 系统初始化（startup/系统初始化通常已在启动代码中完成） */
	SystemInit();

	/* 初始化串口与 LED */
	//Serial_Init();
    thrd_Init();
	//T_Adc_Init();
    OLED_Init();
	///* 创建任务：优先级 Blink>Print */
	//xTaskCreate(vTaskBlink, "Blink", 128, NULL, tskIDLE_PRIORITY + 2, NULL);
	//xTaskCreate(vTaskPrint, "Print", 256, NULL, tskIDLE_PRIORITY + 1, NULL);

	///* 启动调度 */
	vTaskStartScheduler();

	/* 若调度器返回，进入死循环 */
	while (1)
	{
	}
}
#endif

#ifdef DEBUG
/* main 不再在此处做滤波：滤波已被移到 thrd 模块处理并通过 IIC 发送 */

int main(void)
{
	//u16 adcx1,adcx2,adcx3; 
    
    
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//uart1_init(115200);	 	//���ڳ�ʼ��Ϊ115200
	T_Adc_Init();
  thrd_Init();
	OLED_Init();

	/* 初始化滤波器系数 */
	Butterworth_Init(&bw1, cutoff_fc, sample_fs);
	Butterworth_Init(&bw2, cutoff_fc, sample_fs);
	Butterworth_Init(&bw3, cutoff_fc, sample_fs);
	
	
	while(1)
 {
	/* 读取原始 ADC 值（整数） */
	adcx1 = T_Get_Adc_Average(THRD1_ADC_CH0, 10);
	adcx2 = T_Get_Adc_Average(THRD2_ADC_CH0, 10);
	adcx3 = T_Get_Adc_Average(THRD3_ADC_CH0, 10);

	/* 转为浮点并进行巴特沃斯滤波 */
	float f1 = Butterworth_Filter(&bw1, (float)adcx1);
	float f2 = Butterworth_Filter(&bw2, (float)adcx2);
	float f3 = Butterworth_Filter(&bw3, (float)adcx3);

	/* 将滤波结果限制并转换回整数显示（根据 ADC 分辨率自行调整） */
	if (f1 < 0.0f) f1 = 0.0f;
	if (f2 < 0.0f) f2 = 0.0f;
	if (f3 < 0.0f) f3 = 0.0f;

	u16 adcf1 = (u16)(f1 + 0.5f);
	u16 adcf2 = (u16)(f2 + 0.5f);
	u16 adcf3 = (u16)(f3 + 0.5f);

	/* 显示滤波后的数值 */
	OLED_ShowNum(1, 1, adcf1, 4, OLED_8X16);
	OLED_ShowNum(1, 18, adcf2, 4, OLED_8X16);
	OLED_ShowNum(1, 36, adcf3, 4, OLED_8X16);
	OLED_Update();

	/* 保证采样率近似为 sample_fs（这里延时 10 ms）
	   注意：Delay_ms 的精度受系统定时器影响，如需更精确采样请使用定时器中断触发采样。 */
	Delay_ms(10);
 }
 
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	////uart1_init(115200);	 	//���ڳ�ʼ��Ϊ115200
	//T_Adc_Init();
  //thrd_Init();
		
	
	//while(1)
	//{
		//adcx1=T_Get_Adc_Average(THRD1_ADC_CH0,10);
		//adcx2=T_Get_Adc_Average(THRD2_ADC_CH0,10);
		//adcx3=T_Get_Adc_Average(THRD3_ADC_CH0,10);

		//Delay_ms(500);
	//}
	

}
#endif////
