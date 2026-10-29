# Phase 0 spike (throwaway): micro-ROS Jazzy over WiFi UDP on Pico 2 W, FreeRTOS SMP.
cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2_w CACHE STRING "Board type")
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include($ENV{FREERTOS_KERNEL_PATH}/portable/ThirdParty/Community-Supported-Ports/GCC/RP2350_ARM_NTZ/FreeRTOS_Kernel_import.cmake)

project(umbreon_spike C CXX ASM)
# The build image has no libstdc++ headers for arm-none-eabi (<cstdlib>); skip the SDK's
# C++ new/delete overrides, which are the only C++ source pulled in.
add_compile_definitions(PICO_CXX_DISABLE_ALLOCATION_OVERRIDES=1)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

set(REPO_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)
include(${REPO_ROOT}/config/secrets.cmake)

# -DSPIKE_DISCOVERY=ON builds with an empty AGENT_IP to exercise agent discovery.
option(SPIKE_DISCOVERY "Build with empty AGENT_IP (discovery test)" OFF)
if(SPIKE_DISCOVERY)
    set(SPIKE_AGENT_IP "")
else()
    set(SPIKE_AGENT_IP "${MICROROS_AGENT_IP}")
endif()

set(MICROROS_DIR ${REPO_ROOT}/micro_ros/libmicroros)

add_executable(umbreon_spike main.c udp_transport.c)
target_include_directories(umbreon_spike PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${MICROROS_DIR}/include)
target_compile_definitions(umbreon_spike PRIVATE
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
    AGENT_IP=\"${SPIKE_AGENT_IP}\"
    AGENT_PORT=${MICROROS_AGENT_PORT}
    PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=3000)
target_link_libraries(umbreon_spike
    pico_stdlib
    pico_cyw43_arch_lwip_sys_freertos
    FreeRTOS-Kernel-Heap4
    ${MICROROS_DIR}/libmicroros.a)
pico_enable_stdio_usb(umbreon_spike 1)
pico_enable_stdio_uart(umbreon_spike 0)
set_target_properties(umbreon_spike PROPERTIES LINKER_LANGUAGE C)  # no libstdc++ in image
pico_add_extra_outputs(umbreon_spike)
