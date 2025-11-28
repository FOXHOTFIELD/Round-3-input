#include "myHeader.h"
#include <math.h>

int16_t thrd_BLACK = 0, thrd_WHITE  = 0;
uint8_t g_thrd_correct_wip = 0; //正在进行校正
uint8_t thrd_correct_count = 0; //校正计数
uint8_t g_thrd_correct_finished = 0;

static volatile float Target = 0, Actual = 0, Out = 0;			//目标值，实际值，输出值
volatile float Kp = 20 , Ki = 0.5, Kd = 0.35;					//比例项，积分项，微分项的权重
static volatile float Error0 = 0, Error1 = 0, ErrorInt = 0;		//本次误差，上次误差，误差积分
static volatile float DifOut = 0, Actual1 = 0;				//微分项输出，上次实际值

#define INTEGRAL_MAX 10.0f		//积分限幅
#define OUTPUT_MAX 20.0f		//输出限幅

volatile float v1 = 0, v2 = 0, v3 = 0;
volatile float offset = 0;

// 状态机结构
typedef struct {
    uint8_t curStatus;   // 当前状态: 1-极左 2-左偏 3-中心 4-右偏 5-极右
    uint8_t lstStatus;   // 上一个状态
    uint8_t tempStatus;  // 临时状态
    uint8_t sameCount;   // 相同状态计数
} LineStatus_t;

static LineStatus_t Status = {3, 3, 3, 0}; // 初始化为中心状态

void thrdPID(void)
{
    if(!g_thrd_correct_finished) return;

    /*归一化: 先进行范围归一化,再除以总和保证相加为1*/
    v1 = (adcf1 - thrd_WHITE) * 1.0 / (thrd_BLACK - thrd_WHITE);
    v2 = (adcf2 - thrd_WHITE) * 1.0 / (thrd_BLACK - thrd_WHITE);
    v3 = (adcf3 - thrd_WHITE) * 1.0 / (thrd_BLACK - thrd_WHITE);
    
    // 计算总和并归一化,使三值相加为1
    float sum = v1 + v2 + v3;
    if(sum > 0.001f) { // 避免除零
        v1 /= sum;
        v2 /= sum;
        v3 /= sum;
    }

    //float adc[6] = {adcf1, adcf2, adcf3, v1, v2, v3};
    //Serial_SendJustFloat(adc, 6);

    /*二值化判断: 以1000为界*/
    uint8_t b1 = (adcf1 > 1000) ? 1 : 0; // 左侧传感器
    uint8_t b2 = (adcf2 > 1000) ? 1 : 0; // 中间传感器
    uint8_t b3 = (adcf3 > 1000) ? 1 : 0; // 右侧传感器
    
    // 临时状态变量用于存储本次读取的状态
    uint8_t newStatus = 3; // 默认为中心状态
    
    // 使用位模式匹配进行状态判断(二值化传感器)
    uint8_t pattern = (b1 << 2) | (b2 << 1) | b3;
    
    if(pattern == 2){ // 010 - 中心
        newStatus = 3;
    }else if(pattern == 6 || pattern == 4){ // 110/100 - 左偏
        newStatus = 2;
    }else if(pattern == 3 || pattern == 1){ // 011/001 - 右偏
        newStatus = 4;
    }else if(pattern == 7){ // 111 - 全黑(交叉路口或宽线)
        newStatus = 3;
    }else if(pattern == 0){ // 000 - 全白(脱线)
        // 根据上一次状态推断方向
        if(Status.curStatus == 2 || Status.curStatus == 1){
            newStatus = 1; // 极左
        }else if(Status.curStatus == 4 || Status.curStatus == 5){
            newStatus = 5; // 极右
        }else{
            newStatus = 3; // 默认中心
        }
    }else{ // 其他情况保持当前状态
        newStatus = Status.curStatus;
    }
    
    // 状态机逻辑: 检查新状态是否与临时状态相同
    if (newStatus == Status.tempStatus)
    {
        // 如果新状态与临时状态相同,计数器加1
        Status.sameCount++;
        
        // 连续2次相同就更新当前状态,提升反应速度
        if (Status.sameCount >= 2)
        {
            // 更新上一个状态
            Status.lstStatus = Status.curStatus;
            Status.curStatus = newStatus;
            Status.sameCount = 2; // 保持在2,避免溢出
        }
    }
    else
    {
        // 如果新状态与临时状态不同,重置计数器并更新临时状态
        Status.sameCount = 1;
        Status.tempStatus = newStatus;
    }

    OLED_ShowNum(1, 56, Status.curStatus, 1, OLED_6X8);

    

        offset = v1 - v3; //v1->left v3->right. if v1 > v3, offset > 0, need to turn left.
        //if(Status.curStatus == 3) offset = 0;       //如果在状态3 则不希
        
        /*获取本次实际位置值和上次实际位置值*/
        /*Encoder_Get函数，可以获取两次读取编码器的计次值增量*/
        /*计次值增量进行累加，即可得到计次值本身（即实际位置）*/
        /*这里先获取增量，再进行累加，实际上是绕了个弯子*/
        /*如果只需要得到编码器的位置，而不需要得到速度*/
        /*则Encode_Get函数内部的代码可以修改为return TIM_GetCounter(TIM3);直接返回CNT计数器的值*/
        /*修改后，此处代码改为Actual = Encoder_Get();直接得到位置，就不再需要累加了，这样更直接*/
        Actual1 = Actual;			//获取上次实际值
        Actual = offset;           //获取本次实际值
        
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
            
            /*积分限幅，防止积分饱和*/
            if (ErrorInt > INTEGRAL_MAX) {ErrorInt = INTEGRAL_MAX;}
            if (ErrorInt < -INTEGRAL_MAX) {ErrorInt = -INTEGRAL_MAX;}
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
        //if (Out > OUTPUT_MAX) {Out = OUTPUT_MAX;}		//限制输出值最大
        //if (Out < -OUTPUT_MAX) {Out = -OUTPUT_MAX;}	//限制输出值最小

        //if(Status.curStatus == 3) Out = 0;
        
        OLED_ShowFloatNum(15, 56, Out, 2, 2, OLED_6X8);
        
        /*执行控制*/
        /*输出值给到电机PWM*/
        /*因为此函数的输入范围是-100~100，所以上面输出限幅，需要给Out值限定在-100~100*/
        //Motor_SetPWM(Out);
        
    //float arr[2] = {offset, Out};
    //Serial_SendJustFloat(arr, 2);
    Serial_mySend(speed1, speed2, Out, Status.curStatus);

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
