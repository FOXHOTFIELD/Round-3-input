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
    uint8_t curStatus;   // 当前状态: 1-极左 2-左 3-偏左 4-中心 5-偏右 6-右 7-极右
    uint8_t lstStatus;   // 上一个状态
    uint8_t tempStatus;  // 临时状态
    uint8_t sameCount;   // 相同状态计数
} LineStatus_t;

static LineStatus_t Status = {4, 4, 4, 0}; // 初始化为中心状态(扩展为7态, 中心为4)

void thrdPID(void)
{
    if(!g_thrd_correct_finished) return;

    /*归一化: 先进行范围归一化,再除以总和保证相加为1 (3个模拟传感器: adcf1..adcf3)
      注意: adcf0 与 adcf4 为数字量(0/1), 不参与模拟归一化，仅用作边界指示*/
    v1 = (adcf1 - thrd_WHITE) * 1.0f / (thrd_BLACK - thrd_WHITE);
    v2 = (adcf2 - thrd_WHITE) * 1.0f / (thrd_BLACK - thrd_WHITE);
    v3 = (adcf3 - thrd_WHITE) * 1.0f / (thrd_BLACK - thrd_WHITE);

    // 计算总和并归一化,使三值相加为1
    float sum = v1 + v2 + v3;
    if (sum > 0.001f) { // 避免除零
        v1 /= sum; v2 /= sum; v3 /= sum;
    }

    //float adc[6] = {adcf1, adcf2, adcf3, v1, v2, v3};
    //Serial_SendJustFloat(adc, 6);

    /*二值化判断: adcf0 与 adcf4 实为数字量(0/1)，adcf1..3 为 ADC 值，仍用阈值区分（兼容旧代码）*/
    uint8_t b0 = (adcf0) ? 1 : 0; // 最左侧数字传感器 (真实为 0/1)
    uint8_t b1 = (adcf1 > 1000) ? 1 : 0; // 左侧传感器
    uint8_t b2 = (adcf2 > 1000) ? 1 : 0; // 中间传感器
    uint8_t b3 = (adcf3 > 1000) ? 1 : 0; // 右侧传感器
    uint8_t b4 = (adcf4) ? 1 : 0; // 最右侧数字传感器 (真实为 0/1)

    // 临时状态变量用于存储本次读取的状态
    uint8_t newStatus = 4; // 默认为中心状态(扩展后中心为4)

    // 仅使用二值化模式判断（不计算质心）
    uint8_t pattern5 = (b0 << 4) | (b1 << 3) | (b2 << 2) | (b3 << 1) | b4;

    if (pattern5 == 0)
    {
        // 全白(脱线): 根据上一次状态推断方向
        if (Status.curStatus <= 3) newStatus = 1; // 偏左->极左
        else if (Status.curStatus >= 5) newStatus = 7; // 偏右->极右
        else newStatus = 4; // 中心 -> 仍认为中心
    }
    else if (pattern5 == 0x1F)
    {
        // 全黑(宽线或交叉) -> 中心
        newStatus = 4;
    }
    else
    {
        // 按常见位组合优先级匹配到 1..7
        // 优先匹配明显单/相邻组合，其他组合保持当前状态以减少抖动

        // 极左/极右优先
        if (b0 && !b1 && !b2 && !b3 && !b4) newStatus = 1;
        else if (b4 && !b3 && !b2 && !b1 && !b0) newStatus = 7;
        // 左侧组合
        else if (b0 && b1 && !b2) newStatus = 2;
        else if (b1 && !b0 && !b2 && !b3) newStatus = 3;
        else if (b1 && b2 && !b3) newStatus = 3;
        // 中间
        else if (b2 && !b1 && !b3) newStatus = 4;
        // 右侧组合
        else if (b2 && b3 && !b4) newStatus = 5;
        else if (b3 && !b2 && !b4 && !b1) newStatus = 5;
        else if (b3 && b4 && !b2) newStatus = 6;
        // 两端同时检测，认为在中间
        else if (b0 && b4) newStatus = 4;
        else
        {
            // 其它复杂组合：保持当前状态以避免抖动
            newStatus = Status.curStatus;
        }
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

    

        if((adcf0 && adcf4) || (!adcf0 && !adcf4)) offset = v1-v3;//v1->left v3->right. if v1 > v3, offset > 0, need to turn left.
        else offset =(adcf0 * 1.5) + (adcf4 * (-1.5));
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
