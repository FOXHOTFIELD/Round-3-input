#ifndef __LSENS_H
#define __LSENS_H	

#define THRD_PORT GPIOA
#define THRD_GPIO_CLK RCC_APB2Periph_GPIOA//GPIOB时钟
#define THRD1_GPIO_PIN GPIO_Pin_0//PA0
#define THRD2_GPIO_PIN GPIO_Pin_1//PA1
#define THRD3_GPIO_PIN GPIO_Pin_2//PA2

#define THRD1_ADC_CH0		ADC_Channel_0
#define THRD2_ADC_CH0		ADC_Channel_1
#define THRD3_ADC_CH0		ADC_Channel_2


void thrd_Init(void);
static void vThrd_EncoderTask(void *pvParameters);
void Thrd_EncoderTask(void);

#endif 
