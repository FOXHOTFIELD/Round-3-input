 
#include "Delay.h"
#include "sys.h"
  
// ADC 模块相关函数
// 本文件对 ADC1 进行初始化并提供单通道采样与平均采样接口
// 默认使用单次规则转换，外部触发关闭，右对齐数据

void T_Adc_Init(void)  // 初始化 ADC1
{
	ADC_InitTypeDef ADC_InitStructure; 
	// 使能 GPIOA 和 ADC1 的外设时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE );
	// 配置 ADC 时钟为 PCLK2/6（72MHz/6 = 12MHz）
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);

	// 复位 ADC1 为默认状态
	ADC_DeInit(ADC1);

	// ADC 初始化：独立模式、单通道、非连续、无外部触发、右对齐
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;    // 独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;         // 不扫描多通道
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;   // 关闭连续转换（单次转换）
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 软件触发
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // 右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;               // 规则通道数为 1
	ADC_Init(ADC1, &ADC_InitStructure);

	// 使能 ADC1
	ADC_Cmd(ADC1, ENABLE);

	// 复位校准寄存器并等待复位完成
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));

	// 启动 ADC 校准并等待校准完成
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
}

// 对指定通道 ch 进行一次 ADC 转换并返回转换值
u16 T_Get_Adc(u8 ch)
{
	// 配置规则通道：指定通道、在序列中的顺序为 1、采样时间为 239.5 周期
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );

	// 软件触发一次转换并等待转换结束
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));

	// 返回转换结果
	return ADC_GetConversionValue(ADC1);
}

// 对指定通道 ch 进行 times 次采样并返回平均值
u16 T_Get_Adc_Average(u8 ch, u8 times)
{
	u32 temp_val = 0;
	u8 t;
	for(t = 0; t < times; t++)
	{
		temp_val += T_Get_Adc(ch);
		Delay_ms(5); // 每次采样间隔 5ms，避免过快连续采样
	}
	return temp_val / times;
}



