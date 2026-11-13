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


# 应用选择先确定，再由 Self_core 的各层分别注册源文件与依赖。
set(APP_BOARD "GIMBAL" CACHE STRING "Board: GIMBAL or CHASSIS")
set_property(CACHE APP_BOARD PROPERTY STRINGS GIMBAL CHASSIS)
set(APP_LOG_MODE "TEXT" CACHE STRING "UART1: TEXT memory reports or VOFA")
set_property(CACHE APP_LOG_MODE PROPERTY STRINGS TEXT VOFA)
set(APP_ROBOT "SENTRY" CACHE STRING "Robot: SENTRY or NAVIGATION")
set_property(CACHE APP_ROBOT PROPERTY STRINGS SENTRY NAVIGATION)
set(APP_UI "OFF" CACHE STRING "Referee UI: OFF, REFEREE or DEMO")
set_property(CACHE APP_UI PROPERTY STRINGS OFF REFEREE DEMO)
add_subdirectory(Self_core)
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
target_link_libraries(${CMAKE_PROJECT_NAME} stm32cubemx)
