cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME BNGU_Control_System_FreeRTOS)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add application sources from Self_core. CubeMX-generated sources remain in
# cmake/stm32cubemx/CMakeLists.txt.
set(SELF_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Self_core")
set(SELF_CORE_SOURCES
    "${SELF_CORE_DIR}/App/common/communication/app_chassis_comm.c"
    "${SELF_CORE_DIR}/App/common/communication/app_gimbal_comm.c"
    "${SELF_CORE_DIR}/App/common/system/app_control.c"
    "${SELF_CORE_DIR}/App/common/system/app_init.c"
    "${SELF_CORE_DIR}/App/sentry/chassis/app_sentry_chassis.c"
    "${SELF_CORE_DIR}/App/sentry/gimbal/app_sentry_gimbal.c"
    "${SELF_CORE_DIR}/Bsp/bsp_adc.c"
    "${SELF_CORE_DIR}/Bsp/bsp_can.c"
    "${SELF_CORE_DIR}/Bsp/bsp_flash.c"
    "${SELF_CORE_DIR}/Bsp/bsp_gpio.c"
    "${SELF_CORE_DIR}/Bsp/bsp_i2c.c"
    "${SELF_CORE_DIR}/Bsp/bsp_spi.c"
    "${SELF_CORE_DIR}/Bsp/bsp_tim.c"
    "${SELF_CORE_DIR}/Bsp/bsp_uart.c"
    "${SELF_CORE_DIR}/Drv/Drv_buzzer/drv_buzzer_port.c"
    "${SELF_CORE_DIR}/Drv/Drv_buzzer/drv_buzzer.c"
    "${SELF_CORE_DIR}/Drv/Drv_dbus/drv_dbus_port.c"
    "${SELF_CORE_DIR}/Drv/Drv_dbus/drv_dbus.c"
    "${SELF_CORE_DIR}/Drv/Drv_imu/drv_imu_port.c"
    "${SELF_CORE_DIR}/Drv/Drv_imu/drv_imu.c"
    "${SELF_CORE_DIR}/Drv/Drv_led/drv_led_port.c"
    "${SELF_CORE_DIR}/Drv/Drv_led/drv_led.c"
    "${SELF_CORE_DIR}/Drv/Drv_melody/drv_melody.c"
    "${SELF_CORE_DIR}/Drv/Drv_motor/drv_motor_port.c"
    "${SELF_CORE_DIR}/Drv/Drv_motor/drv_motor.c"
    "${SELF_CORE_DIR}/Drv/Drv_power_measure/drv_power_measure_port.c"
    "${SELF_CORE_DIR}/Drv/Drv_power_measure/drv_power_measure.c"
    "${SELF_CORE_DIR}/Drv/Drv_referee/drv_referee_port.c"
    "${SELF_CORE_DIR}/Drv/Drv_referee/drv_referee.c"
    "${SELF_CORE_DIR}/Drv/Drv_vofa/drv_vofa_port.c"
    "${SELF_CORE_DIR}/Drv/Drv_vofa/drv_vofa.c"
    "${SELF_CORE_DIR}/Lib/Lib_filter/lib_filter.c"
    "${SELF_CORE_DIR}/Lib/Lib_math/lib_math.c"
    "${SELF_CORE_DIR}/Lib/Lib_pid/lib_pid.c"
    "${SELF_CORE_DIR}/Os/FreeRTOS/app_rtos.c"
    "${SELF_CORE_DIR}/Os/FreeRTOS/app_tasks.c"
    "${SELF_CORE_DIR}/Os/FreeRTOS/app_rtos_port.c"
    "${SELF_CORE_DIR}/Os/FreeRTOS/app_rtos_shared.c"
    "${SELF_CORE_DIR}/App/sentry/shoot/app_sentry_shoot.c"
    "${SELF_CORE_DIR}/Lib/Lib_math/lib_periodic.c"
)
# List Self_core headers as target sources so IDEs associate them with this target.
file(GLOB_RECURSE SELF_CORE_HEADERS CONFIGURE_DEPENDS
    "${SELF_CORE_DIR}/*.h"
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${SELF_CORE_SOURCES}
    ${SELF_CORE_HEADERS}
)

# Add include paths for Self_core headers.
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    "${SELF_CORE_DIR}"
    "${SELF_CORE_DIR}/App/common/communication"
    "${SELF_CORE_DIR}/App/common/system"
    "${SELF_CORE_DIR}/App/sentry"
    "${SELF_CORE_DIR}/App/sentry/shoot"
    "${SELF_CORE_DIR}/App/sentry/chassis"
    "${SELF_CORE_DIR}/App/sentry/gimbal"
    "${SELF_CORE_DIR}/Bsp"
    "${SELF_CORE_DIR}/Drv/Drv_buzzer"
    "${SELF_CORE_DIR}/Drv/Drv_dbus"
    "${SELF_CORE_DIR}/Drv/Drv_imu"
    "${SELF_CORE_DIR}/Drv/Drv_led"
    "${SELF_CORE_DIR}/Drv/Drv_melody"
    "${SELF_CORE_DIR}/Drv/Drv_motor"
    "${SELF_CORE_DIR}/Drv/Drv_power_measure"
    "${SELF_CORE_DIR}/Drv/Drv_referee"
    "${SELF_CORE_DIR}/Drv/Drv_vofa"
    "${SELF_CORE_DIR}/Lib/Lib_filter"
    "${SELF_CORE_DIR}/Lib/Lib_math"
    "${SELF_CORE_DIR}/Lib/Lib_pid"
    "${SELF_CORE_DIR}/Lib/Lib_Typedef"
    "${SELF_CORE_DIR}/Os/FreeRTOS"
)


set(APP_BOARD "GIMBAL" CACHE STRING "Board: GIMBAL or CHASSIS")
set_property(CACHE APP_BOARD PROPERTY STRINGS GIMBAL CHASSIS)
set(APP_LOG_MODE "TEXT" CACHE STRING "UART1: TEXT memory reports or VOFA")
set_property(CACHE APP_LOG_MODE PROPERTY STRINGS VOFA TEXT)
set(APP_ROBOT "SENTRY" CACHE STRING "Robot: only SENTRY is implemented")
set_property(CACHE APP_ROBOT PROPERTY STRINGS SENTRY)
if(NOT APP_ROBOT STREQUAL "SENTRY")
    message(FATAL_ERROR "Only APP_ROBOT=SENTRY is implemented")
endif()
if(APP_BOARD STREQUAL "GIMBAL")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE CURRENT_BOARD=BOARD_GIMBAL)
elseif(APP_BOARD STREQUAL "CHASSIS")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE CURRENT_BOARD=BOARD_CHASSIS)
else()
    message(FATAL_ERROR "APP_BOARD must be GIMBAL or CHASSIS")
endif()
if(APP_LOG_MODE STREQUAL "VOFA")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE APP_LOG_VOFA=1)
elseif(APP_LOG_MODE STREQUAL "TEXT")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE APP_LOG_VOFA=0)
else()
    message(FATAL_ERROR "APP_LOG_MODE must be VOFA or TEXT")
endif()
# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
