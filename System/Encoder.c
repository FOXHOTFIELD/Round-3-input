#include "myHeader.h"

void Encoder_Init(void){
    /*编码器1输入*/
        /*EncoderA -> PA6 TIM3_CH1 EncoderB -> PA7 TIM3_CH2*/
        /*同时TIM3每10ms产生中断更新转速*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_IS;
    GPIO_IS.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_IS.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;       //电机1 编码器A B TIM3
    GPIO_IS.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_IS);

    TIM_TimeBaseInitTypeDef TIM_TBIS;
    TIM_TBIS.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TBIS.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TBIS.TIM_Period = 65536-1;
    TIM_TBIS.TIM_Prescaler = 1 - 1;
    TIM_TBIS.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TBIS);

    TIM_ICInitTypeDef TIM_ICIS;
    TIM_ICStructInit(&TIM_ICIS);

    TIM_ICIS.TIM_Channel = TIM_Channel_1;
    TIM_ICIS.TIM_ICFilter = 0xF;
    TIM_ICInit(TIM3, &TIM_ICIS);
    TIM_ICIS.TIM_Channel = TIM_Channel_2;
    TIM_ICIS.TIM_ICFilter = 0xF;
    TIM_ICInit(TIM3, &TIM_ICIS);

    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_Cmd(TIM3, ENABLE);


    /*编码器2输入*/
      /*EncoderA -> PB6 TIM4_CH1 EncoderB -> PB7 TIM4_CH2*/
      /*使用 TIM4 的 CH1/CH2(PB6/PB7) 作为编码器输入，适用于板上无法使用 PA0/PA1 的情况*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_IS.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_IS.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;       //电机2 编码器A B TIM4 (PB6/PB7)
    GPIO_IS.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_IS);

    TIM_TBIS.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TBIS.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TBIS.TIM_Period = 65536-1;
    TIM_TBIS.TIM_Prescaler = 1 - 1;
    TIM_TBIS.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TBIS);

    TIM_ICStructInit(&TIM_ICIS);

    TIM_ICIS.TIM_Channel = TIM_Channel_1;
    TIM_ICIS.TIM_ICFilter = 0xF;
    TIM_ICInit(TIM4, &TIM_ICIS);
    TIM_ICIS.TIM_Channel = TIM_Channel_2;
    TIM_ICIS.TIM_ICFilter = 0xF;
    TIM_ICInit(TIM4, &TIM_ICIS);

    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_Cmd(TIM4, ENABLE);

}

/**
  * 函    数：获取编码器的增量值
  * 参    数：无
  * 返 回 值：自上此调用此函数后，编码器的增量值
  */
int16_t Motor1_getSpeed(void)
{
    uint16_t cnt = TIM_GetCounter(TIM3);
    TIM_SetCounter(TIM3, 0);
    
    // 处理正反转:如果 cnt > 32767,视为反向(负值)
    if (cnt > 32767) {
        return (int16_t)(cnt - 65536);
    }
    return (int16_t)cnt;
}

/**
  * 函    数：获取编码器的增量值
  * 参    数：无
  * 返 回 值：自上此调用此函数后，编码器的增量值
  */
int16_t Motor2_getSpeed(void)
{
    uint16_t cnt = TIM_GetCounter(TIM4);
    TIM_SetCounter(TIM4, 0);
    
    // 处理正反转:如果 cnt > 32767,视为反向(负值)
    if (cnt > 32767) {
        return (int16_t)(cnt - 65536);
    }
    return (int16_t)cnt;
}
