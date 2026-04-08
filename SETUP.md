# Guia de Configuração — FreeRTOS na Raspberry Pi Pico W

Documentação completa para ambientar e rodar o projeto de robô seguidor de linha com FreeRTOS na Raspberry Pi Pico W.

---

## Requisitos de Hardware

| Componente | Detalhe |
|---|---|
| Microcontrolador | Raspberry Pi **Pico W** (RP2040 + CYW43) |
| Driver de motores | Compatível com controlo PWM duplo por motor |
| Sensores IR | 5 sensores via multiplexador analógico |
| Multiplexador | MUX analógico ligado ao GPIO 18/19/20 + ADC GPIO 28 |
| Botões | START (GPIO 7) e RESET (GPIO 27) com pull-up |

### Pinagem

| Função | GPIO |
|---|---|
| Motor 1 — Pino A | 16 |
| Motor 1 — Pino B | 17 |
| Motor 2 — Pino A | 14 |
| Motor 2 — Pino B | 15 |
| Botão START | 7 |
| Botão RESET | 27 |
| ADC (MUX out) | 28 |
| MUX — Bit A | 18 |
| MUX — Bit B | 19 |
| MUX — Bit C | 20 |

---

## Requisitos de Software

### Ferramentas obrigatórias

Instala tudo via **winget** no PowerShell:

```powershell
winget install Git.Git
winget install Kitware.CMake
winget install Ninja-build.Ninja
winget install Arm.GnuArmEmbeddedToolchain
```

### Pico SDK

Clona o SDK manualmente:

```powershell
cd C:\Users\<user>
git clone https://github.com/raspberrypi/pico-sdk.git --branch 2.2.0 --depth 1
cd pico-sdk
git submodule update --init
```

Define a variável de ambiente (fecha e reabre o PowerShell depois):

```powershell
[System.Environment]::SetEnvironmentVariable("PICO_SDK_PATH", "C:\Users\<user>\pico-sdk", "User")
```

### FreeRTOS Kernel

Clona dentro da pasta do projeto:

```powershell
cd <pasta-do-projeto>
git clone https://github.com/FreeRTOS/FreeRTOS-Kernel.git
cd FreeRTOS-Kernel
git submodule update --init
```

---

## Estrutura do Projeto

```
meu-projeto/
├── CMakeLists.txt
├── pico_sdk_import.cmake        ← copiado de pico-sdk/external/
├── FreeRTOS_Kernel_import.cmake ← copiado de FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/
├── FreeRTOS-Kernel/             ← repositório clonado
├── include/
│   └── FreeRTOSConfig.h
└── src/
    └── main.c
```

O `pico_sdk_import.cmake` encontra-se em:
```
C:\Users\<user>\pico-sdk\external\pico_sdk_import.cmake
```

O `FreeRTOS_Kernel_import.cmake` encontra-se em:
```
FreeRTOS-Kernel\portable\ThirdParty\GCC\RP2040\FreeRTOS_Kernel_import.cmake
```

---

## CMakeLists.txt

```cmake
# == Bloco gerado pela extensão VS Code da Raspberry Pi — não editar ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# =====================================================================

set(PICO_BOARD pico_w CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)
include(FreeRTOS_Kernel_import.cmake)

project(meu_projeto C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Declara onde está o FreeRTOSConfig.h
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

add_executable(meu_projeto
    src/main.c
)

target_include_directories(meu_projeto PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(meu_projeto
    pico_stdlib
    pico_multicore
    pico_cyw43_arch_threadsafe_background
    hardware_pwm
    hardware_adc
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_add_extra_outputs(meu_projeto)
pico_enable_stdio_usb(meu_projeto 1)
pico_enable_stdio_uart(meu_projeto 0)
```

> **Atenção:** usa `pico_cyw43_arch_threadsafe_background` e **não** `pico_cyw43_arch_none`.
> O `none` não é thread-safe e causa crashes com o FreeRTOS.

---

## FreeRTOSConfig.h

```c
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*
 * NÃO incluas pico/stdlib.h aqui.
 * Causa inclusão circular com o portmacro.h do RP2040.
 */

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

/* SMP — número de cores (desligado para compatibilidade com CYW43) */
#define configNUMBER_OF_CORES                       1
#define configUSE_CORE_AFFINITY                     0
#define configRUN_MULTIPLE_PRIORITIES               0
#define configUSE_TASK_PREEMPTION_DISABLE           0

/* Sincronização */
#define configUSE_TASK_NOTIFICATIONS                1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES       3
#define configUSE_MUTEXES                           1
#define configUSE_RECURSIVE_MUTEXES                 1
#define configUSE_COUNTING_SEMAPHORES               1
#define configUSE_QUEUE_SETS                        1
#define configQUEUE_REGISTRY_SIZE                   8

/* Event Groups — obrigatório para o port.c do RP2040 */
#define configUSE_EVENT_GROUPS                      1

/* Memória — static allocation desligado evita implementar funções obrigatórias */
#define configSUPPORT_STATIC_ALLOCATION             0
#define configSUPPORT_DYNAMIC_ALLOCATION            1
#define configSTACK_DEPTH_TYPE                      uint32_t
#define configMESSAGE_BUFFER_LENGTH_TYPE            size_t

/* Debug */
#define configUSE_TRACE_FACILITY                    1
#define configUSE_STATS_FORMATTING_FUNCTIONS        1
#define configGENERATE_RUN_TIME_STATS               0

/* Hooks — desligados por padrão */
#define configUSE_IDLE_HOOK                         0
#define configUSE_TICK_HOOK                         0
#define configCHECK_FOR_STACK_OVERFLOW              0
#define configUSE_MALLOC_FAILED_HOOK                0
#define configUSE_DAEMON_TASK_STARTUP_HOOK          0

/* Timers de software */
#define configUSE_TIMERS                            1
#define configTIMER_TASK_PRIORITY                   ( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH                    10
#define configTIMER_TASK_STACK_DEPTH                1024

/* Cortex-M0+ (RP2040) */
#define configPRIO_BITS                             2
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY             3
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY        1
#define configKERNEL_INTERRUPT_PRIORITY \
    ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )

/*
 * Assert simplificado — NÃO usar taskDISABLE_INTERRUPTS() aqui.
 * Causa "implicit declaration" no contexto do portmacro.h.
 */
#define configASSERT( x )   if( ( x ) == 0 ) { for( ;; ); }

/* API incluída */
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
```

---

## Build

### Primeira vez (ou após mudar o CMakeLists.txt)

```powershell
cd <pasta-do-projeto>
mkdir build
cd build
cmake .. -G Ninja -DPICO_SDK_PATH="C:\Users\<user>\pico-sdk"
ninja
```

### Builds seguintes (apenas mudanças em .c / .h)

```powershell
cd <pasta-do-projeto>\build
ninja
```

### Limpar e recompilar tudo (erros inexplicáveis)

```powershell
cd <pasta-do-projeto>\build
Remove-Item -Recurse -Force *
cmake .. -G Ninja -DPICO_SDK_PATH="C:\Users\<user>\pico-sdk"
ninja
```

---

## Upload para o Pico W

1. Mantém o botão **BOOTSEL** pressionado
2. Liga o cabo USB ao computador
3. Solta o **BOOTSEL** — a Pico aparece como drive USB `RPI-RP2`
4. Arrasta o ficheiro `build/meu_projeto.uf2` para essa drive
5. A Pico reinicia automaticamente e o programa começa a correr

---

## Monitor Serial

### VS Code (recomendado)

1. Instala a extensão **Serial Monitor** da Microsoft
2. `Ctrl+Shift+P` → **Serial Monitor: Open Serial Monitor**
3. Seleciona a porta COM da Pico e velocidade **115200**

### Descobrir a porta COM

```powershell
Get-PnpDevice -Class Ports | Where-Object {$_.FriendlyName -like "*USB*"}
```

Ou: **Gestor de Dispositivos → Portas (COM e LPT)**

---

## Erros Comuns e Soluções

| Erro | Causa | Solução |
|---|---|---|
| `SDK location was not specified` | `PICO_SDK_PATH` não definido | Passa `-DPICO_SDK_PATH=...` no cmake |
| `FreeRTOS.h: No such file or directory` | `FreeRTOSConfig.h` não encontrado | Confirma `target_include_directories` no CMake |
| `xEventGroupSetBitsFromISR` não declarado | Event Groups não ativados | Adiciona `configUSE_EVENT_GROUPS 1` ao config |
| `cannot find -lFreeRTOS-Kernel-EventGroups` | Target inexistente | Remove a linha do `target_link_libraries` — o config já é suficiente |
| `implicit declaration of taskDISABLE_INTERRUPTS` | `configASSERT` complexo | Usa `#define configASSERT(x) if((x)==0){for(;;);}` |
| LED apagado / programa não arranca | `cyw43_arch_init()` dentro de task | Chama `cyw43_arch_init()` no `main()` antes do scheduler |
| Motor em velocidade máxima | PWM wrap mal configurado | Usa `pwm_config_set_wrap(&cfg, 255)` e garante `PWM_MAX ≤ 200` |
| Programa crasha após 2 ciclos | `pico_cyw43_arch_none` usado | Troca para `pico_cyw43_arch_threadsafe_background` |

---

## Notas Importantes

- O **Pico W** controla o LED via chip CYW43 — não é possível usar `gpio_put` para o LED. Usa sempre `cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, ...)`.
- O `cyw43_arch_init()` **tem de ser chamado antes** do `vTaskStartScheduler()`, nunca dentro de uma task.
- O PWM **nunca deve ultrapassar 200** nesta placa — a corrente derruba a alimentação.
- Quando usas `taskENTER_CRITICAL()` / `taskEXIT_CRITICAL()`, mantém a secção crítica o mais curta possível.
- O `configSUPPORT_STATIC_ALLOCATION 0` evita ter de implementar `vApplicationGetIdleTaskMemory` e `vApplicationGetTimerTaskMemory` — se o mudares para `1`, terás de as implementar.
