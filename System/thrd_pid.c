#include "myHeader.h"
#include <math.h>

int16_t thrd_BLACK = 0, thrd_WHITE  = 0;
uint8_t g_thrd_correct_wip = 0; //正在进行校正
uint8_t thrd_correct_count = 0; //校正计数
uint8_t g_thrd_correct_finished = 0;

static volatile float Target = 0, Actual = 0, Out = 0;			//目标值，实际值，输出值
volatile float Kp = 22 , Ki = 0, Kd = 1.1;					//比例项，积分项，微分项的权重
static volatile float Error0 = 0, Error1 = 0, ErrorInt = 0;		//本次误差，上次误差，误差积分
static volatile float DifOut = 0, Actual1 = 0;				//微分项输出，上次实际值

#define INTEGRAL_MAX 10.0f		//积分限幅
#define OUTPUT_MAX 20.0f		//输出限幅

/* 边缘数字传感器权重: adcf0/adcf4 为二值量(0/1)。
    通过宏可调节它们在质心计算中的影响力。默认与旧代码近似值 1.5f。 */
#define THRD_ADC0_WEIGHT 1.5f
#define THRD_ADC4_WEIGHT 1.5f
/* 中间模拟传感器的整体权重（可调整），1.0 表示按归一化后直接参与质心计算 */
#define THRD_ANALOG_WEIGHT 1.0f
/* 丢线时寻找用的固定偏移，正值表示向左搜索（与 offset 符号约定一致） */
#define THRD_SEARCH_OFFSET 2.0f
/* 用于周期摆动搜线的常量 */
#define THRD_PI 3.14159265f
/* 摆动振幅（与 THRD_SEARCH_OFFSET 保持一致） */
#define THRD_SEARCH_AMPLITUDE THRD_SEARCH_OFFSET
/* 每次调用相位增量，单位为弧度。可调节搜索速度：值越大摆动越快 */
#define THRD_SEARCH_STEP 0.0f

/* 当脱线且计算出的 offset 很小时，基于状态强制一个最小偏移，避免无法转向。
    可根据车辆调参。正值表示向左搜索/转向，负值表示向右。 */
#define THRD_MIN_OVERRIDE_OFFSET 0.8f

/* 周期摆动搜线状态变量（文件级静态） */
static float thrd_search_phase = 0.0f;
static uint8_t thrd_searching = 0;

/* 发卡弯（hairpin）检测与应对：仅调整 offset 来实现转向 */
#define THRD_HAIRPIN_DETECT_CNT_THRESH 4
#define THRD_HAIRPIN_EXEC_MAXCNT 120
#define THRD_HAIRPIN_OFFSET_AMPLITUDE (THRD_SEARCH_AMPLITUDE * 1.2f)
#define THRD_HAIRPIN_SWEEP_STEP 0.25f

static uint8_t hairpin_state = 0;         // 0=正常,1=执行发卡弯策略
static uint8_t hairpin_detect_cnt = 0;    // 连续检测计数
static uint16_t hairpin_exec_cnt = 0;     // 执行周期计数
static float hairpin_phase = 0.0f;        // 发卡弯摆动相位
static int8_t hairpin_prefer_sign = 1;    // 1=优先向左, -1=优先向右

/* PID 切换：当偏移量较大时使用更激进的 PID 参数；使用滞回防止频繁切换 */
#define THRD_PID_SWITCH_THRESHOLD 1.80f    /* 绝对 offset 超过此值使用 large PID */
#define THRD_PID_SWITCH_HYSTERESIS 0.15f   /* 滞回量 */
/* large PID 参数（偏移较大时使用）——建议：增大 P 和 D，减小 I */
#define THRD_PID_LARGE_KP 35.0f
#define THRD_PID_LARGE_KI 0.10f
#define THRD_PID_LARGE_KD 1.0f

/* PID 过渡（平滑线性插值）设置: 过渡分成若干步，默认 20 步 */
#define THRD_PID_TRANSITION_STEPS 15
/*
 * 曲线插值（缓动）: 为了比线性插值更平滑的模式切换，使用余弦缓入/缓出（ease-in-out）:
 * eased = 0.5 * (1 - cos(pi * t)), t in [0,1]
 * 这样在切换开始与结束时会更平滑，减少突变。
 */
static inline float thrd_pid_ease(float t)
{
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return 0.5f * (1.0f - cosf(THRD_PI * t));
}

/* small PID 使用文件顶部的 Kp/Ki/Kd 变量（默认值保留）
    运行时用 thrd_pid_mode 切换生效参数 */
static uint8_t thrd_pid_mode = 0; /* 0 = small(default), 1 = large */
static uint8_t thrd_pid_target_mode = 0; /* 目标模式, 与 thrd_pid_mode 协作用于平滑过渡 */
static uint8_t thrd_pid_transition = 0; /* 过渡进行中标志 */
static uint16_t thrd_pid_transition_cnt = 0; /* 已过渡步数 */

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

// pattern5 映射表: index = (b0<<4)|(b1<<3)|(b2<<2)|(b3<<1)|b4 -> 状态 1..7
// 生成规则（用于参考）: score = -3*b0 -1*b1 +0*b2 +1*b3 +3*b4; state = clamp(score+4,1,7)
static uint8_t pattern_map[32] = {
    /*  0..7  */ 4,7,5,7, 4,7,5,7,
    /*  8..15 */ 6,6,6,7, 6,6,6,7,   // 扩宽右侧靠近状态6的映射
    /* 16..23 */ 1,2,2,5, 1,2,2,5,   // 扩宽左侧靠近状态2的映射
    /* 24..31 */ 1,2,1,2, 1,2,1,4    // 更多左偏组合映射为2
};

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
        // 全白(脱线): 保留原有策略，根据上一次状态推断方向
        if (Status.curStatus <= 3) newStatus = 1; // 偏左->极左
        else if (Status.curStatus >= 5) newStatus = 7; // 偏右->极右
        else newStatus = 4; // 中心 -> 仍认为中心
    }
    else
    {
        // 使用查表映射（包含全黑的映射），由 pattern5 直接索引到状态 1..7
        newStatus = pattern_map[pattern5];
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

    

        /* 使用质心法计算 offset：
           - adcf1..adcf3 已归一化(v1..v3)，按 THRD_ANALOG_WEIGHT 权重参与
           - adcf0/adcf4 为二值量，按宏 THRD_ADC0_WEIGHT/THRD_ADC4_WEIGHT 参与
           - 位置坐标取：adcf0=-2, adcf1=-1, adcf2=0, adcf3=1, adcf4=2
           - 为保持与旧代码语义一致（v1>v3 时 offset>0 表示需要向左转），
             我们令 offset = -centroid（质心取负）
        */
        {
            float s0 = adcf0 ? THRD_ADC0_WEIGHT : 0.0f;
            float s1 = v1 * THRD_ANALOG_WEIGHT;
            float s2 = v2 * THRD_ANALOG_WEIGHT;
            float s3 = v3 * THRD_ANALOG_WEIGHT;
            float s4 = adcf4 ? THRD_ADC4_WEIGHT : 0.0f;

            float numerator = s0 * (-2.0f) + s1 * (-1.0f) + s2 * 0.0f + s3 * 1.0f + s4 * 2.0f;
            float denom = s0 + s1 + s2 + s3 + s4;

            if (denom > 1e-6f) {
                float centroid = numerator / denom;
                offset = -centroid; // 左偏为正
                /* 如果找回线，停止搜索模式并重置相位（可保留相位以连续搜索） */
                thrd_searching = 0;
                /* 找到线时若正在执行发卡弯策略，结束该策略 */
                if (hairpin_state == 1) {
                    hairpin_state = 0;
                    hairpin_detect_cnt = 0;
                    hairpin_exec_cnt = 0;
                }
                /*
                 * 修复：某些情况下二值/状态判定已经表明车在极左/极右（Status=1或7），
                 * 但质心计算由于噪声等原因得到接近 0 的 offset，导致转向无响应。
                 * 这里基于状态在 offset 很小的时候强制一个最小偏移量，确保向检测到的方向转向。
                 */
                /* 脱线判断：使用原始 ADC 值判断是否脱线（阈值 310），而不是依赖计算出的 offset 很小。
                   当三路中间传感器都很小（都 < 310）时认为脱线，根据状态强制偏移以转向。 */
                if (adcf1 < thrd_WHITE + 10 && adcf2 < thrd_WHITE + 10 && adcf3 < thrd_WHITE + 10) {
                    if (Status.curStatus <= 3) {
                        /* 偏左，强制向左偏移 */
                        offset = THRD_SEARCH_OFFSET;
                    } else if (Status.curStatus >= 5) {
                        /* 偏右，强制向右偏移（注意符号：右转为负） */
                        offset = -THRD_SEARCH_OFFSET;
                    }
                }
            } else {
                /* 先进行发卡弯检测：当左右边缘同时感应到黑且中间很弱时，累加检测计数 */
                if (adcf0 && adcf4 && (v2 < 0.12f || b2 == 0)) {
                    if (hairpin_detect_cnt < 255) hairpin_detect_cnt++;
                } else {
                    if (hairpin_detect_cnt) hairpin_detect_cnt = 0;
                }

                /* 若检测到发卡弯，进入发卡弯策略（优先使用发卡弯策略覆盖普通搜线） */
                if (hairpin_state == 0 && hairpin_detect_cnt >= THRD_HAIRPIN_DETECT_CNT_THRESH) {
                    hairpin_state = 1;
                    hairpin_exec_cnt = 0;
                    /* 优先朝上次偏向的方向搜索 */
                    if (Status.curStatus <= 3) hairpin_prefer_sign = 1;
                    else if (Status.curStatus >= 5) hairpin_prefer_sign = -1;
                    else if (Status.lstStatus <= 3) hairpin_prefer_sign = 1;
                    else if (Status.lstStatus >= 5) hairpin_prefer_sign = -1;
                    else hairpin_prefer_sign = 1;
                    /* 初始化相位，使第一次输出与优先方向一致 */
                    hairpin_phase = (hairpin_prefer_sign > 0) ? (THRD_PI/2.0f) : (-THRD_PI/2.0f);
                }

                if (hairpin_state == 1) {
                    /* 执行发卡弯策略：用更激进的周期摆动覆盖 offset，促使车快速掉头 */
                    hairpin_phase += THRD_HAIRPIN_SWEEP_STEP;
                    if (hairpin_phase > 2.0f * THRD_PI) hairpin_phase -= 2.0f * THRD_PI;
                    offset = hairpin_prefer_sign * THRD_HAIRPIN_OFFSET_AMPLITUDE * sinf(hairpin_phase);
                    hairpin_exec_cnt++;

                    /* 在执行过程中检测到回到线路（中传感器触线或质心恢复）则结束策略 */
                    if (b2 || (denom > 1e-6f)) {
                        hairpin_state = 0;
                        hairpin_detect_cnt = 0;
                        hairpin_exec_cnt = 0;
                    } else if (hairpin_exec_cnt > THRD_HAIRPIN_EXEC_MAXCNT) {
                        /* 超时后退回到常规摆动搜线（并且扩大摆幅以继续寻找） */
                        hairpin_state = 0;
                        hairpin_detect_cnt = 0;
                        hairpin_exec_cnt = 0;
                        thrd_searching = 1;
                        thrd_search_phase = hairpin_phase; /* 保持相位连续性 */
                    }
                } else {
                    /* 常规周期摆动搜线（当未进入发卡弯策略时） */
                    if (!thrd_searching) {
                        thrd_searching = 1;
                        if (Status.curStatus <= 3) thrd_search_phase = THRD_PI / 2.0f;        // sin = +1 -> 向左
                        else if (Status.curStatus >= 5) thrd_search_phase = -THRD_PI / 2.0f; // sin = -1 -> 向右
                        else if (Status.lstStatus <= 3) thrd_search_phase = THRD_PI / 2.0f;
                        else if (Status.lstStatus >= 5) thrd_search_phase = -THRD_PI / 2.0f;
                        else thrd_search_phase = THRD_PI / 2.0f; // 兜底向左
                    }

                    thrd_search_phase += THRD_SEARCH_STEP;
                    /* 归一相位到 0..2PI 范围，避免过大 */
                    if (thrd_search_phase > 2.0f * THRD_PI) thrd_search_phase -= 2.0f * THRD_PI;

                    offset = THRD_SEARCH_AMPLITUDE * sinf(thrd_search_phase);
                }
            }
        }
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
        
        /*PID计算，先根据 offset 选择有效 PID 参数（含滞回防抖）*/
//			DifOut = Kd * (Error0 - Error1); // 普通 PID 微分项（注释保留）
        {
            float abs_off = fabsf(offset);
            /* 切换逻辑：当处于 small 模式且偏移过大，切换到 large；
               当处于 large 且偏移小于阈值减滞回时切回 small */
            /* 切换逻辑：当处于 small 模式且偏移过大，开始过渡到 large；
               当处于 large 且偏移小于阈值减滞回时，开始过渡回 small
               这里不再立即修改 ErrorInt，而是使用线性插值在若干步内平滑过渡参数和积分缩放。 */
            if (thrd_pid_transition == 0) {
                if (thrd_pid_mode == 0) {
                    if (abs_off > THRD_PID_SWITCH_THRESHOLD) {
                        thrd_pid_target_mode = 1;
                        thrd_pid_transition = 1;
                        thrd_pid_transition_cnt = 0;
                    }
                } else {
                    if (abs_off < (THRD_PID_SWITCH_THRESHOLD - THRD_PID_SWITCH_HYSTERESIS)) {
                        thrd_pid_target_mode = 0;
                        thrd_pid_transition = 1;
                        thrd_pid_transition_cnt = 0;
                    }
                }
            }

            float Kp_eff = Kp;
            float Ki_eff = Ki;
            float Kd_eff = Kd;
            float ErrorInt_used = ErrorInt; /* 用于 PID 计算的积分项（可能被平滑缩放） */

            if (thrd_pid_transition) {
                /* 计算插值因子 alpha: 0..1 （用缓动函数平滑过渡） */
                float alpha = (THRD_PID_TRANSITION_STEPS > 0) ? ((float)thrd_pid_transition_cnt / (float)THRD_PID_TRANSITION_STEPS) : 1.0f;
                if (alpha > 1.0f) alpha = 1.0f;
                float eased = thrd_pid_ease(alpha);

                if (thrd_pid_mode == 0 && thrd_pid_target_mode == 1) {
                    /* small -> large */
                    Kp_eff = Kp + (THRD_PID_LARGE_KP - Kp) * eased;
                    Ki_eff = Ki + (THRD_PID_LARGE_KI - Ki) * eased;
                    Kd_eff = Kd + (THRD_PID_LARGE_KD - Kd) * eased;
                    /* ErrorInt 从 1.0 缩放到 0.5 (使用 eased) */
                    float scale = 1.0f - 0.5f * eased;
                    ErrorInt_used = ErrorInt * scale;
                } else if (thrd_pid_mode == 1 && thrd_pid_target_mode == 0) {
                    /* large -> small */
                    Kp_eff = THRD_PID_LARGE_KP + (Kp - THRD_PID_LARGE_KP) * eased;
                    Ki_eff = THRD_PID_LARGE_KI + (Ki - THRD_PID_LARGE_KI) * eased;
                    Kd_eff = THRD_PID_LARGE_KD + (Kd - THRD_PID_LARGE_KD) * eased;
                    /* ErrorInt 从 0.5 平滑恢复到 1.0 (使用 eased) */
                    float scale = 0.5f + 0.5f * eased;
                    ErrorInt_used = ErrorInt * scale;
                } else {
                    /* 如果目标与当前一致，直接结束过渡 */
                    thrd_pid_transition = 0;
                }

                thrd_pid_transition_cnt++;
                if (thrd_pid_transition_cnt >= THRD_PID_TRANSITION_STEPS) {
                    /* 结束过渡，正式切换模式 */
                    thrd_pid_transition = 0;
                    thrd_pid_mode = thrd_pid_target_mode;
                }
            } else {
                /* 非过渡期直接按当前模式选择参数 */
                if (thrd_pid_mode) {
                    Kp_eff = THRD_PID_LARGE_KP;
                    Ki_eff = THRD_PID_LARGE_KI;
                    Kd_eff = THRD_PID_LARGE_KD;
                    ErrorInt_used = ErrorInt; /* large 模式下不再立即缩放（已通过过渡处理） */
                } else {
                    Kp_eff = Kp;
                    Ki_eff = Ki;
                    Kd_eff = Kd;
                    ErrorInt_used = ErrorInt;
                }
            }

            /* 微分先行公式，使用有效 Kd */
            DifOut = - Kd_eff * (Actual - Actual1);

            /* 位置式PID计算，使用有效 Kp/Ki 和可能被平滑缩放的 ErrorInt_used */
            Out = Kp_eff * Error0 + Ki_eff * ErrorInt_used + DifOut;
        }
        
        /*输出限幅*/
        if (Out > OUTPUT_MAX) {Out = OUTPUT_MAX;}		//限制输出值最大
        if (Out < -OUTPUT_MAX) {Out = -OUTPUT_MAX;}	//限制输出值最小

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
