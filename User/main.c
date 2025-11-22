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
    Encoder_Init();




	 while(1)
	 {
//		  /* 不在 main 中做滤波：仅读取平均 ADC 原始值并显示
			  //滤波已移至 `thrd.c`，并会通过 IIC 发送滤波后的 thrd 值给主机/外设 */
		  //adcx1 = T_Get_Adc_Average(THRD1_ADC_CH0, 10);
		  //adcx2 = T_Get_Adc_Average(THRD2_ADC_CH0, 10);
		  //adcx3 = T_Get_Adc_Average(THRD3_ADC_CH0, 10);

		  //OLED_ShowNum(1, 1, adcx1, 4, OLED_8X16);
		  //OLED_ShowNum(1, 18, adcx2, 4, OLED_8X16);
		  //OLED_ShowNum(1, 36, adcx3, 4, OLED_8X16);
		  //OLED_ShowNum(50, 1, Motor1_getSpeed(), 3, OLED_8X16);
		  //OLED_Update();
	//(void)pvParameters;
        Thrd_EncoderTask();
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
