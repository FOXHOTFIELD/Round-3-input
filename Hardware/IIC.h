/*
 * IIC.h
 * 简单 I2C 从机接口（针对 STM32F1 + 标准外设库）
 * 提供：初始化从机地址与设置发送缓冲区
 */

#ifndef __IIC_H
#define __IIC_H

#include "stm32f10x.h"
#include <stdint.h>

/* 初始化 I2C1 作为从机，ownAddress 为 7 位从地址（例如 0x30）
 * 本实现为软件 I2C 从机（bit-banged slave），以“主机先发送数据（主写从读）”
 * 为主要使用场景：主机向从机写数据，从机接收并保存到接收缓冲区。
 */
void IIC_Slave_Init(uint8_t ownAddress);

/* 设置从机的发送缓冲区（用于主机发起读取时从机返回数据），
 * 仍保留此函数以兼容可能的主读场景。buf: 数据缓冲，len: 字节数（<=32）
 */
void IIC_SetTxBuffer(uint8_t *buf, uint8_t len);

/* 读取从机接收到的数据（主写到从机时保存的数据）
 * outBuf: 用户提供的缓冲区，maxLen: 指向缓冲区最大长度，返回时写入实际接收字节数
 */
void IIC_GetRxBuffer(uint8_t *outBuf, uint8_t *maxLen);

/* 清空接收缓冲区（在处理完接收数据后调用） */
void IIC_ClearRxBuffer(void);

/* 调试接口：读取/清除 ISR 调用计数，方便确认中断是否触发 */
uint32_t IIC_GetIrqCount(void);
void IIC_ClearIrqCount(void);

#endif
