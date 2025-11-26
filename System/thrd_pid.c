#include "myHeader.h"

int16_t thrd_BLACK = 0, thrd_WHITE  = 0;
uint8_t g_thrd_correct_wip = 0; //正在进行校正
uint8_t thrd_correct_count = 0; //校正计数
uint8_t g_thrd_correct_finished = 0;

static volatile float Target, Actual, Out;			//目标值，实际值，输出值
volatile float Kp, Ki, Kd;					//比例项，积分项，微分项的权重
static volatile float Error0, Error1, ErrorInt;		//本次误差，上次误差，误差积分
static volatile float DifOut, Actual1;				//微分项输出，上次实际值

volatile float v1 = 0, v2 = 0, v3 = 0;
volatile float offset = 0;

void thrdPID(void)
{
    if(!g_thrd_correct_finished) return;

    /*归一化*/
    v1 = (adcf1 - thrd_WHITE) * 1.0 / (thrd_BLACK - thrd_WHITE) * 1.0;
    v2 = (adcf2 - thrd_WHITE) * 1.0 / (thrd_BLACK - thrd_WHITE) * 1.0;
    v3 = (adcf3 - thrd_WHITE) * 1.0 / (thrd_BLACK - thrd_WHITE) * 1.0;

    //float adc[6] = {adcf1, adcf2, adcf3, v1, v2, v3};
    //Serial_SendJustFloat(adc, 6);

    offset = v1 - v3; //v1->left v3->right. if v1 > v3, offset > 0, need to turn left.

        
        /*获取本次实际位置值和上次实际位置值*/
        /*Encoder_Get函数，可以获取两次读取编码器的计次值增量*/
        /*计次值增量进行累加，即可得到计次值本身（即实际位置）*/
        /*这里先获取增量，再进行累加，实际上是绕了个弯子*/
        /*如果只需要得到编码器的位置，而不需要得到速度*/
        /*则Encode_Get函数内部的代码可以修改为return TIM_GetCounter(TIM3);直接返回CNT计数器的值*/
        /*修改后，此处代码改为Actual = Encoder_Get();直接得到位置，就不再需要累加了，这样更直接*/
        Actual1 = Actual;			//获取上次实际值
        Actual += offset;           //获取本次实际值
        
        /*获取本次误差和上次误差*/
        Error1 = Error0;			//获取上次误差
        Error0 = Target - Actual;	//获取本次误差，目标值减实际值，即为误差值
        
        /*误差积分（累加）*/
        /*如果Ki不为0，才进行误差积分，这样做的目的是便于调试*/
        /*因为在调试时，我们可能先把Ki设置为0，这时积分项无作用，误差消除不了，误差积分会积累到很大的值*/
        /*后续一旦Ki不为0，那么因为误差积分已经积累到很大的值了，这就导致积分项疯狂输出，不利于调试*/
        if (Ki != 0)				//如果Ki不为0
        {
            ErrorInt += Error0;		//进行误差积分
        }
        else						//否则
        {
            ErrorInt = 0;			//误差积分直接归0
        }
        
        /*PID计算，先得到微分项*/
//			DifOut = Kd * (Error0 - Error1);		//这一句是普通PID的微分项计算公式
        DifOut = - Kd * (Actual - Actual1);		//这一句是微分先行的微分项计算公式
                                                //计算结果要取负，因为实际值的变化趋势和误差变化趋势相反
        
        /*PID计算*/
        /*使用位置式PID公式，计算得到输出值*/
        Out = Kp * Error0 + Ki * ErrorInt + DifOut;
        
        /*输出限幅*/
        if (Out > 20) {Out = 20;}		//限制输出值最大为20
        if (Out < -20) {Out = -20;}	//限制输出值最小为20
        
        /*执行控制*/
        /*输出值给到电机PWM*/
        /*因为此函数的输入范围是-100~100，所以上面输出限幅，需要给Out值限定在-100~100*/
        //Motor_SetPWM(Out);

    float arr[2] = {offset, Out};
    Serial_SendJustFloat(arr, 2);

}

void thrd_correct(void)
{
    if(!g_thrd_correct_wip){
        if(GPIO_ReadInputDataBit(GPIOB, thrd_correct_port) == Bit_SET){
            g_thrd_correct_wip = 1;
        }
    }

    if(g_thrd_correct_wip){
       if(thrd_correct_count == 0) thrd_BLACK = 0, thrd_WHITE = 0;
        thrd_BLACK += adcf2;
        thrd_WHITE += (adcf1 + adcf3) / 2;
        thrd_correct_count++;

        if(thrd_correct_count == 10){
            thrd_BLACK /= 10;
            thrd_WHITE /= 10;
            thrd_correct_count = 0;
            g_thrd_correct_wip = 0;
            g_thrd_correct_finished = 1;
        }

    }
}
