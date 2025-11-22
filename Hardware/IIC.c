/* 简单的 软件 I2C 从机实现（基于 STM32F10x 标准外设库）
 * - 使用 PB6 = SCL, PB7 = SDA
 * - 以“主机先发送数据（主写，从机接收）”为主要使用场景：
 *   主机向从机写入数据，从机在内部接收缓冲区保存收到的数据。
 * - 同时保留主机读取（主读，从机返回 txBuf）兼容路径。
 *
 * 注意：本实现为教学/工程示例，适用于短数据包和低速场景（典型 <100kHz）。
 * 软件从机对时序敏感，使用前请在目标硬件上验证并根据需要微调时序。
 */

#include "IIC.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "OLED.h"
#include <stddef.h>

/*
 * 说明：软件 I2C 从机实现（Bit-banged slave）
 * - 使用 PB6 作为 SCL，PB7 作为 SDA（与原来硬件引脚保持一致）
 * - 仅实现主机读取从机（Master Read -> Slave Transmit）的场景，
 *   这是原实现主要使用的功能（IIC_SetTxBuffer 提供待发送缓冲区）
 * - 通过 EXTI 中断监听 SCL/SDA 的变化并在时钟沿驱动/采样 SDA
 * - 所有注释均为中文，接口与原文件保持兼容：`IIC_Slave_Init` / `IIC_SetTxBuffer`
 *
 * 注意：软件从机实现对时序敏感，适合低速主机（典型 <100kHz）。
 * 在实际使用前建议在目标硬件上做验证和必要的时序微调。
 */

/* 软件 I2C 从机只需对主机时钟做采样，用户可按需在外部限制主机速率 */

/* 发送/接收缓冲区：
 * - txBuf: 从机在被主机读时用于返回数据（保留以兼容读场景）
 * - rxBuf: 主机写时从机接收到的数据（主写从读的主要数据路径）
 */
static uint8_t txBuf[32];
static volatile uint8_t txLen = 0;
static volatile uint8_t txIdx = 0;
static uint8_t rxBuf[32];
static volatile uint8_t rxLen = 0;

/* I/O 定义（保持 PB6=SCL, PB7=SDA） */
#define IIC_SCL_PIN    GPIO_Pin_6
#define IIC_SDA_PIN    GPIO_Pin_7
#define IIC_PORT       GPIOB

/* 软件从机内部状态 */
typedef enum { IIC_STATE_IDLE = 0, IIC_STATE_ADDR, IIC_STATE_RECEIVE, IIC_STATE_TRANSMIT, IIC_STATE_WAIT_ACK } IIC_State_t;
static volatile IIC_State_t iic_state = IIC_STATE_IDLE;

/* 接收/发送临时变量 */
static volatile uint8_t bit_cnt = 0;       // 位计数（0-7）
static volatile uint8_t curr_byte = 0;     // 接收/发送当前字节
static volatile uint8_t tx_byte = 0;       // 正在发送的字节
static volatile uint8_t tx_bit_idx = 0;    // 发送位索引（0-7）
static volatile uint8_t own_addr = 0;      // 从机地址（7-bit）
static volatile uint8_t rxIdx = 0;         // 接收缓冲索引（用于内部）

/* ACK 辅助标志 */
static volatile uint8_t ack_pending = 0;   // 需要在下一时钟周期驱动 ACK
static volatile uint8_t release_after_ack = 0; // 在 ACK 后释放 SDA

/* 调试：中断计数器，用于确认 EXTI9_5_IRQHandler 是否被触发 */
static volatile uint32_t iic_irq_count = 0;

/* 辅助 I/O 操作：拉低 SDA（驱动 0），释放 SDA（输入上拉），读取 SDA/SCL */
static void SDA_drive_low(void)
{iic_irq_count++;
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = IIC_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; // 开漏输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(IIC_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(IIC_PORT, IIC_SDA_PIN);

}

static void SDA_release(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = IIC_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 输入上拉，释放总线
    GPIO_Init(IIC_PORT, &GPIO_InitStructure);
}

static uint8_t SDA_read(void)
{
    return GPIO_ReadInputDataBit(IIC_PORT, IIC_SDA_PIN) ? 1 : 0;
}

static uint8_t SCL_read(void)
{
    return GPIO_ReadInputDataBit(IIC_PORT, IIC_SCL_PIN) ? 1 : 0;
}

void IIC_Slave_Init(uint8_t ownAddress)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 使能 GPIOB 和 AFIO 时钟（用于 EXTI） */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    /* PB6 (SCL)、PB7 (SDA) 初始为输入上拉（释放总线） */
    GPIO_InitStructure.GPIO_Pin = IIC_SCL_PIN | IIC_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 输入，上拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(IIC_PORT, &GPIO_InitStructure);

    /* 保存从机地址 */
    own_addr = ownAddress & 0x7F;

    /* 配置 EXTI：PB6 -> EXTI_Line6, PB7 -> EXTI_Line7，触发上升/下降沿 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource6);
    EXTI_InitStructure.EXTI_Line = EXTI_Line6;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource7);
    EXTI_InitStructure.EXTI_Line = EXTI_Line7;
    EXTI_Init(&EXTI_InitStructure);

    /* EXTI9_5 IRQ（包含 PB6/PB7） */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* 初始状态 */
    iic_state = IIC_STATE_IDLE;
    bit_cnt = 0;
    curr_byte = 0;
    txLen = 0;
    txIdx = 0;
    rxLen = 0;
}

void IIC_SetTxBuffer(uint8_t *buf, uint8_t len)
{
    uint8_t i;
    if(len > sizeof(txBuf)) len = sizeof(txBuf);
    /* 禁用 I2C 中断以避免与 I2C1_EV_IRQHandler 的并发访问（竞态）
       采用短时禁用 NVIC 中断的方式，拷贝完成后立即恢复。
       这样比在拷贝时使用较长的临界区更安全且对系统影响小。 */
    NVIC_DisableIRQ(EXTI9_5_IRQn);
    for(i=0;i<len;i++) txBuf[i] = buf[i];
    txLen = len;
    txIdx = 0;
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* 获取接收缓冲区数据：将内部接收数据拷贝到 outBuf，
 * 并通过 maxLen 返回实际字节数（用户在调用前应把 *maxLen
 * 设置为 outBuf 的最大长度，返回时写入实际拷贝长度）
 */
void IIC_GetRxBuffer(uint8_t *outBuf, uint8_t *maxLen)
{
    uint8_t i;
    uint8_t toCopy = rxLen;
    if(outBuf == NULL || maxLen == NULL) return;
    if(*maxLen < toCopy) toCopy = *maxLen;
    NVIC_DisableIRQ(EXTI9_5_IRQn);
    for(i=0;i<toCopy;i++) outBuf[i] = rxBuf[i];
    *maxLen = toCopy;
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* 清空接收缓冲区 */
void IIC_ClearRxBuffer(void)
{
    NVIC_DisableIRQ(EXTI9_5_IRQn);
    rxLen = 0;
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}
/*
 * EXTI9_5_IRQHandler
 * - 处理 PB6(SCL) 上的时钟沿：在上升沿采样，在下降沿驱动数据（从机发送）
 * - 处理 PB7(SDA) 上的电平变化：在 SCL 高电平时判定 START/STOP
 */
void EXTI9_5_IRQHandler(void)
{
    /* 记录一次中断（用于调试，避免在 ISR 中做耗时的 OLED 操作） */
    

    /* 处理 SCL (PB6) 变化 */
    if(EXTI_GetITStatus(EXTI_Line6) != RESET)
    {
        uint8_t scl = SCL_read();

        if(scl) // SCL 上升沿：主机数据有效期，采样 SDA
        {
            if(iic_state == IIC_STATE_ADDR)
            {
                /* 接收地址和 R/W 位 */
                curr_byte = (uint8_t)((curr_byte << 1) | (SDA_read() & 0x1));
                bit_cnt++;
                if(bit_cnt >= 8)
                {
                    uint8_t addr_rw = curr_byte;
                    if(((addr_rw >> 1) & 0x7F) == own_addr)
                    {
                        /* 地址匹配：区分主写(0)与主读(1) */
                        if((addr_rw & 0x01) == 0)
                        {
                            /* 主机写 -> 从机接收数据（主写从读） */
                            rxLen = 0;
                            rxIdx = 0;
                            ack_pending = 1; // 对地址字节应答 ACK
                            iic_state = IIC_STATE_RECEIVE; // 进入接收态
                        }
                        else if((addr_rw & 0x01) == 1)
                        {
                            /* 主机读 -> 从机发送（保留旧行为） */
                            txIdx = 0;
                            tx_byte = (txLen>0)? txBuf[0] : 0xFF;
                            tx_bit_idx = 0;
                            ack_pending = 1; // 对地址字 ACK
                            iic_state = IIC_STATE_TRANSMIT;
                        }
                    }
                    else
                    {
                        /* 地址不匹配，进入空闲 */
                        iic_state = IIC_STATE_IDLE;
                    }
                    bit_cnt = 0;
                    curr_byte = 0;
                }
            }
            else if(iic_state == IIC_STATE_WAIT_ACK)
            {
                /* 采样主机对刚发字节的 ACK（SDA=0 为 ACK） */
                uint8_t ack = SDA_read();
                if(ack == 0)
                {
                    /* ACK：如果之前是发送路径，继续发送下一字节 */
                    txIdx++;
                    if(txIdx < txLen)
                    {
                        tx_byte = txBuf[txIdx];
                    }
                    else
                    {
                        tx_byte = 0xFF; // 占位
                    }
                    tx_bit_idx = 0;
                    iic_state = IIC_STATE_TRANSMIT;
                }
                else
                {
                    /* NACK：主机结束读取或未 ACK，从机回到空闲 */
                    iic_state = IIC_STATE_IDLE;
                }
            }
            else if(iic_state == IIC_STATE_RECEIVE)
            {
                /* 接收数据字节：上升沿采样 SDA，累积 8 位后存入 rxBuf 并准备 ACK */
                curr_byte = (uint8_t)((curr_byte << 1) | (SDA_read() & 0x1));
                bit_cnt++;
                if(bit_cnt >= 8)
                {
                    /* 存储接收到的字节（有边界检查） */
                    if(rxLen < sizeof(rxBuf))
                    {
                        rxBuf[rxLen++] = curr_byte;
                    }
                    /* 接收完一个字节，下一位由从机 ACK（由下降沿驱动） */
                    ack_pending = 1;
                    bit_cnt = 0;
                    curr_byte = 0;
                }
            }
            else if(iic_state == IIC_STATE_TRANSMIT)
            {
                /* 如果当前处于接收（主写）流程的中间（我们使用 TRANSMIT
                 * 状态来统一处理 ACK 驱动），在上升沿仅用于采样阶段
                 * 真正字节接收在上升沿采样 8 位后完成（如下逻辑所示）。
                 */
                /* 对于接收流程，实际采样由上面的 ADDR 分支或下面的
                 * 单字节接收逻辑处理（当进入接收后我们在上升沿累积位）。
                 */
            }
            else if(iic_state == IIC_STATE_IDLE)
            {
                /* 空闲时忽略采样 */
            }
            /* 额外处理已合并到各状态分支中，移除无效比较以消除编译警告 */
        }
        else // SCL 下降沿：在下降沿切换 SDA 电平以供下一上升沿采样（驱动 ACK 或发送位）
        {
            if(ack_pending)
            {
                /* 在 ACK 位期间由从机拉低 SDA 以应答主机（或地址） */
                SDA_drive_low();
                ack_pending = 0;
                release_after_ack = 1;
            }
            else if(iic_state == IIC_STATE_TRANSMIT && txLen>0)
            {
                /* 发送路径：根据当前发送位驱动 SDA（高电平->释放，总线被上拉） */
                uint8_t bit = (tx_byte & 0x80) ? 1 : 0;
                tx_byte <<= 1;
                tx_bit_idx++;
                if(bit)
                {
                    SDA_release();
                }
                else
                {
                    SDA_drive_low();
                }
                if(tx_bit_idx >= 8)
                {
                    /* 发送完一个字节，下一位为主机 ACK（释放 SDA 以便主机驱动 ACK） */
                    SDA_release();
                    iic_state = IIC_STATE_WAIT_ACK;
                }
            }
            else if(iic_state == IIC_STATE_TRANSMIT && txLen==0)
            {
                /* 接收路径：当我们处于接收流程（主写从机读）时，下降沿用于准备下
                 * 一位采样/响应（ACK 已由 ack_pending 处理）。在接收字节时，实际
                 * 的位采样发生在上升沿，下面在 SDA 上升/下降的 START/STOP 检测
                 * 中会把组帧处理完成。这里无需额外处理，只确保在非驱动期释放 SDA。
                 */
                SDA_release();
            }
            else if(release_after_ack)
            {
                /* ACK 期过后释放 SDA */
                SDA_release();
                release_after_ack = 0;
            }
        }

        EXTI_ClearITPendingBit(EXTI_Line6);
    }

    /* 处理 SDA (PB7) 变化，用于检测 START/STOP（SCL 高时有效） */
    if(EXTI_GetITStatus(EXTI_Line7) != RESET)
    {
        uint8_t sda = SDA_read();
        if(SCL_read())
        {
            if(sda == 0)
            {
                /* START：SDA 从 1 -> 0 且 SCL 为高 */
                iic_state = IIC_STATE_ADDR;
                bit_cnt = 0;
                curr_byte = 0;
            }
            else
            {
                /* STOP：SDA 从 0 -> 1 且 SCL 为高，结束会话 */
                iic_state = IIC_STATE_IDLE;
                bit_cnt = 0;
                curr_byte = 0;
                txIdx = 0;
                SDA_release();
            }
        }
        EXTI_ClearITPendingBit(EXTI_Line7);
    }
}

/* 调试接口：读取/清除中断计数器（非阻塞） */
uint32_t IIC_GetIrqCount(void)
{
    return iic_irq_count;
}

void IIC_ClearIrqCount(void)
{
    iic_irq_count = 0;
}
