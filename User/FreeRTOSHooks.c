/*
 * FreeRTOSHooks.c
 * Minimal application hook implementations required by FreeRTOS config.
 *
 * Provide vApplicationStackOverflowHook because configCHECK_FOR_STACK_OVERFLOW
 * is non-zero in FreeRTOSConfig.h. Adjust behavior (logging, reset, etc.) as
 * appropriate for your application.
 */

#include "FreeRTOS.h"
#include "task.h"

/*
 * Called if a stack overflow is detected. The parameters are the handle of
 * the task that overflowed and the task name (for convenience). The default
 * behaviour here is to disable interrupts and loop forever so the condition
 * can be noticed in a debugger. Change to a system reset or other recovery
 * strategy if desired.
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

    /* Disable interrupts and halt here so the fault can be investigated. */
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}
