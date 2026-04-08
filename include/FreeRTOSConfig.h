#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*
 * FreeRTOSConfig.h — Raspberry Pi Pico (RP2040)
 * SDK 2.x + FreeRTOS Kernel V11.x
 *
 * ATENÇÃO: NÃO incluas pico/stdlib.h ou qualquer header do SDK aqui.
 * Causa inclusão circular com o portmacro.h do RP2040.
 */

/* ================================================================
 * Scheduler
 * ================================================================ */
#define configUSE_PREEMPTION                        1
#define configUSE_TICKLESS_IDLE                     0
#define configCPU_CLOCK_HZ                          133000000UL
#define configTICK_RATE_HZ                          1000
#define configMAX_PRIORITIES                        32
#define configMINIMAL_STACK_SIZE                    256
#define configTOTAL_HEAP_SIZE                       ( 192 * 1024 )
#define configMAX_TASK_NAME_LEN                     16
#define configUSE_16_BIT_TICKS                      0
#define configIDLE_SHOULD_YIELD                     1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION     0

/* ================================================================
 * Sincronização
 * ================================================================ */
#define configUSE_TASK_NOTIFICATIONS                1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES       3
#define configUSE_MUTEXES                           1
#define configUSE_RECURSIVE_MUTEXES                 1
#define configUSE_COUNTING_SEMAPHORES               1
#define configUSE_QUEUE_SETS                        1
#define configQUEUE_REGISTRY_SIZE                   8

/* ================================================================
 * Event Groups — OBRIGATÓRIO para o port.c do RP2040
 * O portmacro.h usa xEventGroupSetBitsFromISR internamente.
 * ================================================================ */
#define configUSE_EVENT_GROUPS                      1

/* ================================================================
 * Memória
 * Static allocation desligado para evitar implementar
 * vApplicationGetIdleTaskMemory e vApplicationGetTimerTaskMemory.
 * ================================================================ */
#define configSUPPORT_STATIC_ALLOCATION             0
#define configSUPPORT_DYNAMIC_ALLOCATION            1
#define configSTACK_DEPTH_TYPE                      uint32_t
#define configMESSAGE_BUFFER_LENGTH_TYPE            size_t

/* ================================================================
 * Debug / Trace
 * ================================================================ */
#define configUSE_TRACE_FACILITY                    1
#define configUSE_STATS_FORMATTING_FUNCTIONS        1
#define configGENERATE_RUN_TIME_STATS               0

/* ================================================================
 * Hook functions — desligados por padrão
 * ================================================================ */
#define configUSE_IDLE_HOOK                         0
#define configUSE_TICK_HOOK                         0
#define configCHECK_FOR_STACK_OVERFLOW              0
#define configUSE_MALLOC_FAILED_HOOK                0
#define configUSE_DAEMON_TASK_STARTUP_HOOK          0

/* ================================================================
 * Timers de software
 * ================================================================ */
#define configUSE_TIMERS                            1
#define configTIMER_TASK_PRIORITY                   ( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH                    10
#define configTIMER_TASK_STACK_DEPTH                1024

/* ================================================================
 * Cortex-M0+ (RP2040) — prioridades de interrupção
 * ================================================================ */
#define configPRIO_BITS                             2
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY             3
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY        1
#define configKERNEL_INTERRUPT_PRIORITY \
    ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )

/* ================================================================
 * Assert — NÃO usa taskDISABLE_INTERRUPTS() aqui!
 * Causa "implicit declaration" no contexto do portmacro.h.
 * ================================================================ */
#define configASSERT( x )   if( ( x ) == 0 ) { for( ;; ); }

/* ================================================================
 * API incluída
 * ================================================================ */
#define INCLUDE_vTaskPrioritySet                    1
#define INCLUDE_uxTaskPriorityGet                   1
#define INCLUDE_vTaskDelete                         1
#define INCLUDE_vTaskSuspend                        1
#define INCLUDE_xResumeFromISR                      1
#define INCLUDE_vTaskDelayUntil                     1
#define INCLUDE_vTaskDelay                          1
#define INCLUDE_xTaskGetSchedulerState              1
#define INCLUDE_xTaskGetCurrentTaskHandle           1
#define INCLUDE_uxTaskGetStackHighWaterMark         1
#define INCLUDE_xTaskGetIdleTaskHandle              1
#define INCLUDE_eTaskGetState                       1
#define INCLUDE_xEventGroupSetBitFromISR            1
#define INCLUDE_xTimerPendFunctionCall              1
#define INCLUDE_xTaskAbortDelay                     1
#define INCLUDE_xTaskGetHandle                      1

#endif /* FREERTOS_CONFIG_H */