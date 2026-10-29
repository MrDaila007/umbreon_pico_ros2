cmake_minimum_required(VERSION 3.16)
project(umbreon_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Werror --coverage)
add_link_options(--coverage)

set(CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/core)
set(DRIVERS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/drivers)
file(GLOB CORE_SRCS CONFIGURE_DEPENDS ${CORE_DIR}/*.c)
file(GLOB DRIVER_SRCS CONFIGURE_DEPENDS ${DRIVERS_DIR}/*.c)
file(GLOB FAKE_SRCS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/fake_*.c)
file(GLOB TEST_SRCS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test_*.c)

if(CORE_SRCS)
  add_library(core STATIC ${CORE_SRCS})
  target_include_directories(core PUBLIC ${CORE_DIR})
  target_link_libraries(core PUBLIC m)
endif()

if(DRIVER_SRCS)
  add_library(drivers STATIC ${DRIVER_SRCS})
  target_include_directories(drivers PUBLIC ${DRIVERS_DIR} ${CORE_DIR})
  target_link_libraries(drivers PUBLIC m)
endif()

enable_testing()
foreach(src ${TEST_SRCS})
  get_filename_component(name ${src} NAME_WE)
  add_executable(${name} ${src} ${FAKE_SRCS})
  target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CORE_DIR} ${DRIVERS_DIR})
  target_link_libraries(${name} PRIVATE m)
  if(CORE_SRCS)
    target_link_libraries(${name} PRIVATE core)
  endif()
  if(DRIVER_SRCS)
    target_link_libraries(${name} PRIVATE drivers)
  endif()
  add_test(NAME ${name} COMMAND ${name})
endforeach()

# Run the real IMU task with a fake clock/bus; no Pico SDK or scheduler required.
target_sources(test_sensors_task PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/app/sensors_task.c)
target_include_directories(test_sensors_task PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/task_stubs
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/app
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/hal)

# Snapshot expiry is checked in the real shared-state reader, including stalled producers.
target_sources(test_shared_state PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/app/shared_state.c)
target_include_directories(test_shared_state PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/task_stubs
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/app
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/hal)

# Run the real control loop/mailboxes against simulated time, PWM and watchdog.
target_sources(test_control_task PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/app/control_task.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/app/shared_state.c)
target_include_directories(test_control_task PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/task_stubs
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/app
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/hal)

# Persistence coordinator with fake flash; never writes hardware in host tests.
target_sources(test_settings_apply PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/app/settings_apply.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/app/shared_state.c)
target_include_directories(test_settings_apply PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/task_stubs
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/app
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/hal)

# Build the production datagram adapter against fake socket functions on the host.
target_sources(test_udp_transport PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/hal/hal_udp_transport.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/hal/hal_udp_probe.c)
target_include_directories(test_udp_transport PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../net/stubs
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/hal)

# Production DHCP RAW wrapper under fake lwIP lock/PCB/pbuf operations.
target_sources(test_dhcp_server PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/hal/hal_dhcp_server.c)
target_include_directories(test_dhcp_server PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../net/stubs
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/hal)

# Production rcl allocator/platform hooks with fake RTOS/time and rcutils registration.
target_sources(test_microros_platform PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/hal/hal_microros_platform.c)
target_include_directories(test_microros_platform PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/task_stubs
  ${CMAKE_CURRENT_SOURCE_DIR}/../net/stubs
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/hal)
target_compile_definitions(test_microros_platform PRIVATE _DEFAULT_SOURCE=1)

# Actual endpoint/radio bridge with fake snapshots/probe, no real network access.
target_sources(test_net_endpoint PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/app/net_endpoint.c)
target_include_directories(test_net_endpoint PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../net/stubs
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/app
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/hal)

# Actual WiFi orchestration with fake async adapters and time.
target_sources(test_net_wifi PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/app/net_wifi.c)
target_include_directories(test_net_wifi PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/app
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/hal)

# Callback mailbox adapter uses real shared state; ROS entities bind it in Task9.
target_sources(test_ros_commands PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/app/ros_commands.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/app/shared_state.c)
target_include_directories(test_ros_commands PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/task_stubs
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/app
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/hal)

# /umbreon/arm handler logic: real shared state, fake clock, simulated control tick.
target_sources(test_ros_services PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/app/ros_services.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/app/shared_state.c)
target_include_directories(test_ros_services PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/task_stubs
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/app
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/hal)
