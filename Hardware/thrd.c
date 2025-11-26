#include "thrd.h"
#include "adc.h"


void thrd_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
  RCC_APB2PeriphClockCmd(THRD_GPIO_CLK,ENABLE);//ʹ��PORTAʱ��	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	
	GPIO_InitStructure.GPIO_Pin=THRD1_GPIO_PIN;
	GPIO_Init(THRD_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=THRD2_GPIO_PIN;
	GPIO_Init(THRD_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=THRD3_GPIO_PIN;
	GPIO_Init(THRD_PORT,&GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = thrd_correct_port;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	T_Adc_Init();
	
}
