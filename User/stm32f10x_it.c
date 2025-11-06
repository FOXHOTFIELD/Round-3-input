/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @brief   Main Interrupt Service Routines.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

/* include FreeRTOS headers so we can call the port systick handler */
#include "FreeRTOS.h"
#include "task.h"

/* FreeRTOS port systick handler symbol - 确认你拷贝的 portable/ARM_CM3/port.c 中导出的名称 */
extern void xPortSysTickHandler(void);

/* If your port provides different names for SVC / PendSV handlers, they are
   normally mapped using macros in FreeRTOSConfig.h:
     #define vPortSVCHandler    SVC_Handler
     #define xPortPendSVHandler PendSV_Handler
   In that case the port's implementation will provide SVC_Handler/PendSV_Handler.
   Do NOT provide SVC_Handler or PendSV_Handler implementations here to avoid
   duplicate symbols. */

/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

void NMI_Handler(void)
{
}

/* HardFault/Memory/Bus/Usage handlers left as template - you can add logging */
void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

/* Debug monitor left as template */
void DebugMon_Handler(void)
{
}

/* SysTick handler is implemented by the FreeRTOS port (portable/.../port.c)
   and therefore must NOT be defined here to avoid duplicate symbol when the
   FreeRTOS portable layer provides SysTick_Handler.

   If you need to perform MCU-specific SysTick work (e.g. HAL_IncTick), do it
   from the FreeRTOS port's SysTick implementation or adapt the port so that
   the port exposes a separate xPortSysTickHandler symbol and this file
   calls that symbol instead. */

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/******************************************************************************/

/* 示例：外设中断处理模板（按需实现） */
void EXTI0_IRQHandler(void)
{
    /* 处理 EXTI0 中断（示例）：
       - 检查并清中断挂起位（按寄存器/StdPeriph 库方式）
       - 如果在 ISR 中使用 FreeRTOS FromISR API（如 xQueueSendFromISR），
         请确保中断优先级符合 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 的限制，
         并在必要时调用 portYIELD_FROM_ISR( xHigherPriorityTaskWoken ); */
}

/* 其他外设中断按需添加 */
