# Umbreon Pico 2 W firmware: FreeRTOS SMP, core-1 control loop, core-0 console.
cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2_w CACHE STRING "Board type")
include(${CMAKE_CURRENT_LIST_DIR}/pico_sdk_import.cmake)
include($ENV{FREERTOS_KERNEL_PATH}/portable/ThirdParty/Community-Supported-Ports/GCC/RP2350_ARM_NTZ/FreeRTOS_Kernel_import.cmake)

project(umbreon_pico_ros2 C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

set(REPO_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)

# Build-time network defaults from the git-ignored config/secrets.cmake (optional).
set(UMB_SSID "")
set(UMB_PASS "")
set(UMB_AGENT_IP_U32 "0u")
set(UMB_AGENT_PORT "8888u")
if(EXISTS ${REPO_ROOT}/config/secrets.cmake)
    include(${REPO_ROOT}/config/secrets.cmake)
    if(DEFINED WIFI_SSID)
        set(UMB_SSID "${WIFI_SSID}")
    endif()
    if(DEFINED WIFI_PASSWORD)
        set(UMB_PASS "${WIFI_PASSWORD}")
    endif()
    if(DEFINED MICROROS_AGENT_IP AND NOT "${MICROROS_AGENT_IP}" STREQUAL "")
        if(NOT MICROROS_AGENT_IP MATCHES "^([0-9]+)\\.([0-9]+)\\.([0-9]+)\\.([0-9]+)$")
            message(FATAL_ERROR "MICROROS_AGENT_IP must be a dotted IPv4 address")
        endif()
        foreach(oct ${CMAKE_MATCH_1} ${CMAKE_MATCH_2} ${CMAKE_MATCH_3} ${CMAKE_MATCH_4})
            if(oct GREATER 255)
                message(FATAL_ERROR "MICROROS_AGENT_IP octet out of range: ${oct}")
            endif()
        endforeach()
        math(EXPR UMB_AGENT_IP_U32
             "(${CMAKE_MATCH_1} << 24) | (${CMAKE_MATCH_2} << 16) | (${CMAKE_MATCH_3} << 8) | ${CMAKE_MATCH_4}"
             OUTPUT_FORMAT HEXADECIMAL)
        set(UMB_AGENT_IP_U32 "${UMB_AGENT_IP_U32}u")
    endif()
    if(DEFINED MICROROS_AGENT_PORT)
        set(UMB_AGENT_PORT "${MICROROS_AGENT_PORT}u")
    endif()
else()
    message(STATUS "config/secrets.cmake not found: empty WiFi / agent defaults")
endif()
foreach(var UMB_SSID UMB_PASS)
    string(REPLACE "\\" "\\\\" ${var} "${${var}}")
    string(REPLACE "\"" "\\\"" ${var} "${${var}}")
endforeach()

file(GLOB CORE_SRCS CONFIGURE_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/core/*.c)
set(FW_SRCS
    hal/hal_pwm.c
    hal/hal_tach.c
    hal/hal_flash_store.c
    hal/hal_time.c
    hal/hal_i2c.c
    hal/hal_gpio_misc.c
    hal/hal_adc.c
    hal/hal_buzzer.c
    hal/hal_encoder.c
    app/shared_state.c
    app/control_task.c
    app/console_task.c
    app/console_cmds.c
    app/main.c)

add_executable(umbreon_pico_ros2 ${FW_SRCS} ${CORE_SRCS})
set_source_files_properties(${FW_SRCS} ${CORE_SRCS} PROPERTIES
    COMPILE_OPTIONS "-Wall;-Wextra;-Werror")
target_include_directories(umbreon_pico_ros2 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/core
    ${CMAKE_CURRENT_LIST_DIR}/drivers
    ${CMAKE_CURRENT_LIST_DIR}/hal
    ${CMAKE_CURRENT_LIST_DIR}/app)
target_compile_definitions(umbreon_pico_ros2 PRIVATE
    UMB_DEFAULT_WIFI_SSID="${UMB_SSID}"
    UMB_DEFAULT_WIFI_PASS="${UMB_PASS}"
    UMB_DEFAULT_AGENT_IP=${UMB_AGENT_IP_U32}
    UMB_DEFAULT_AGENT_PORT=${UMB_AGENT_PORT}
    PICO_DEFAULT_UART=0
    PICO_DEFAULT_UART_TX_PIN=16
    PICO_DEFAULT_UART_RX_PIN=17
    PICO_DEFAULT_UART_BAUD_RATE=115200
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=30000)  # a stalled terminal must not block core 0 for 500 ms (UMB_STOP_CORE0)
target_link_libraries(umbreon_pico_ros2
    pico_stdlib
    pico_flash
    hardware_pwm
    hardware_i2c
    hardware_adc
    hardware_watchdog
    hardware_flash
    hardware_sync
    FreeRTOS-Kernel-Heap4)
pico_enable_stdio_usb(umbreon_pico_ros2 1)
pico_enable_stdio_uart(umbreon_pico_ros2 1)
pico_add_extra_outputs(umbreon_pico_ros2)
