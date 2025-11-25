#include "myHeader.h"

int16_t BLACK = 2222, WHITE = 220;
extern volatile uint16_t adcf1, adcf2, adcf3;

void thrdPID(void)
{
    float v1 = (adcf1 - WHITE) * 1.0 / (BLACK - WHITE) * 1.0;
    float v2 = (adcf2 - WHITE) * 1.0 / (BLACK - WHITE) * 1.0;
    float v3 = (adcf3 - WHITE) * 1.0 / (BLACK - WHITE) * 1.0;

    float adc[6] = {adcf1, adcf2, adcf3, v1, v2, v3};
    Serial_SendJustFloat(adc, 6);
}
