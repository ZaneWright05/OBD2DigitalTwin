# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

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
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(freeRTOStest C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS-Kernel)

add_executable(freeRTOStest freeRTOStest.c

    ${FREERTOS_KERNEL_PATH}/tasks.c
    ${FREERTOS_KERNEL_PATH}/queue.c
    ${FREERTOS_KERNEL_PATH}/list.c
    ${FREERTOS_KERNEL_PATH}/timers.c
    ${FREERTOS_KERNEL_PATH}/event_groups.c
    ${FREERTOS_KERNEL_PATH}/stream_buffer.c
    ${FREERTOS_KERNEL_PATH}/portable/MemMang/heap_4.c

    ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2350_ARM_NTZ/non_secure/port.c
    ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2350_ARM_NTZ/non_secure/portasm.c

    ## MCP2515 CAN controller files
    CAN/CAN_DEV_Config.c
    CAN/MCP2515.c
)

pico_set_program_name(freeRTOStest "freeRTOStest")
pico_set_program_version(freeRTOStest "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(freeRTOStest 0)
pico_enable_stdio_usb(freeRTOStest 1)

# Add the standard library to the build
target_link_libraries(freeRTOStest
        pico_stdlib
        hardware_spi)

# Add the standard include files to the build
target_include_directories(freeRTOStest PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}

        ${FREERTOS_KERNEL_PATH}/include
        ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2350_ARM_NTZ/non_secure

        ${CMAKE_CURRENT_LIST_DIR}/CAN
)

# Add any user requested libraries
target_link_libraries(freeRTOStest 
        hardware_exception
        hardware_irq
        hardware_sync
        pico_multicore

        pico_cyw43_arch_none
        #pico_cyw43_arch_lwip_threadsafe_background
        )

pico_add_extra_outputs(freeRTOStest)