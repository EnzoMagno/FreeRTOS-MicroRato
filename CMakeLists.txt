# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
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
# ====================================================================================
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
    FreeRTOS-Kernel
    pico_cyw43_arch_none
    pico_cyw43_arch_threadsafe_background
    hardware_pwm
    pico_multicore
    FreeRTOS-Kernel-Heap4
    hardware_adc 
)

pico_add_extra_outputs(meu_projeto)
pico_enable_stdio_usb(meu_projeto 1)
pico_enable_stdio_uart(meu_projeto 0)