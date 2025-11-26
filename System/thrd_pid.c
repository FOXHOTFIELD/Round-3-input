#include "myHeader.h"

int16_t thrd_BLACK = 0, thrd_WHITE  = 0;
uint8_t g_thrd_correct_wip = 0; //正在继续校正
uint8_t thrd_correct_count = 0; //校正计数

void thrdPID(void)
{
    /*归一化*/
    float v1 = (adcf1 - thrd_WHITE) * 1.0 / (thrd_BLACK - thrd_WHITE) * 1.0;
    float v2 = (adcf2 - thrd_WHITE) * 1.0 / (thrd_BLACK - thrd_WHITE) * 1.0;
    float v3 = (adcf3 - thrd_WHITE) * 1.0 / (thrd_BLACK - thrd_WHITE) * 1.0;

    float adc[6] = {adcf1, adcf2, adcf3, v1, v2, v3};
    Serial_SendJustFloat(adc, 6);

}

void thrd_correct(void)
{
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
        }

    }
}
