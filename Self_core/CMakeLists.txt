# 应用配置只定义一次；App 算法与任务分别声明依赖。
add_library(self_config INTERFACE)
target_compile_definitions(self_config INTERFACE
    APP_RTOS_ENABLE_EXAMPLES=$<BOOL:${APP_RTOS_ENABLE_EXAMPLES}>)
target_include_directories(self_config INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/App/Config")
if(APP_BOARD STREQUAL "GIMBAL")
    target_compile_definitions(self_config INTERFACE CURRENT_BOARD=BOARD_GIMBAL)
elseif(APP_BOARD STREQUAL "CHASSIS")
    target_compile_definitions(self_config INTERFACE CURRENT_BOARD=BOARD_CHASSIS)
else()
    message(FATAL_ERROR "APP_BOARD must be GIMBAL or CHASSIS")
endif()
if(APP_LOG_MODE STREQUAL "VOFA")
    target_compile_definitions(self_config INTERFACE APP_LOG_VOFA=1)
elseif(APP_LOG_MODE STREQUAL "TEXT")
    target_compile_definitions(self_config INTERFACE APP_LOG_VOFA=0)
else()
    message(FATAL_ERROR "APP_LOG_MODE must be VOFA or TEXT")
endif()
if(APP_ROBOT STREQUAL "SENTRY")
    target_compile_definitions(self_config INTERFACE CURRENT_ROBOT=ROBOT_SENTRY)
elseif(APP_ROBOT STREQUAL "NAVIGATION")
    target_compile_definitions(self_config INTERFACE CURRENT_ROBOT=ROBOT_NAVIGATION)
elseif(APP_ROBOT STREQUAL "ARMORED")
    message(STATUS "ARMORED: independent algorithm integration; full vehicle control is not wired")
    target_compile_definitions(self_config INTERFACE CURRENT_ROBOT=ROBOT_ARMORED)
else()
    message(FATAL_ERROR "APP_ROBOT must be SENTRY, NAVIGATION or ARMORED")
endif()

if(APP_UI STREQUAL "OFF")
    target_compile_definitions(self_config INTERFACE APP_UI_MODE=0)
elseif(APP_UI STREQUAL "REFEREE" OR APP_UI STREQUAL "DEMO")
    if(NOT APP_BOARD STREQUAL "CHASSIS")
        message(FATAL_ERROR "UI uses the CHASSIS referee task and USART6")
    endif()
    if(APP_UI STREQUAL "DEMO")
        target_compile_definitions(self_config INTERFACE APP_UI_MODE=2)
    else()
        target_compile_definitions(self_config INTERFACE APP_UI_MODE=1)
    endif()
else()
    message(FATAL_ERROR "APP_UI must be OFF, REFEREE or DEMO")
endif()

# 纯类型依赖不把驱动端口的 HAL/Bsp 路径传给 App。
add_library(self_drv_types INTERFACE)
file(GLOB driver_header_dirs LIST_DIRECTORIES true "${CMAKE_CURRENT_SOURCE_DIR}/Drv/Drv_*")
list(FILTER driver_header_dirs EXCLUDE REGEX "\\.h$")
target_include_directories(self_drv_types INTERFACE ${driver_header_dirs} "${CMAKE_CURRENT_SOURCE_DIR}/Drv" "${CMAKE_CURRENT_SOURCE_DIR}/Lib/Lib_Typedef")
add_library(self_app_contracts INTERFACE)
target_include_directories(self_app_contracts INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/App/Contracts"
    "${CMAKE_CURRENT_SOURCE_DIR}/Lib/Lib_math")
target_link_libraries(self_app_contracts INTERFACE self_config)

# 仅传递头文件/API，不把任务 OBJECT 库反向链接进 Os，避免目标循环依赖。
add_library(self_os_api INTERFACE)
target_include_directories(self_os_api INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/Os/FreeRTOS"
    "${CMAKE_CURRENT_SOURCE_DIR}/Os/Port"
    "${PROJECT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2")
# CMSIS 声明本身不需要 HAL 或项目板型宏，平台依赖只在实现目标内。
add_library(self_app_tasks_api INTERFACE)
# 各模块接口目录由 App/CMakeLists.txt 的 app_modules 清单统一填充。
target_link_libraries(self_app_tasks_api INTERFACE self_app_contracts self_os_api)

add_subdirectory(Lib)
add_subdirectory(Bsp)
add_subdirectory(Drv)
add_subdirectory(App)
add_subdirectory(Os)

# 所有 OBJECT 目标都直接加入最终链接，避免仅传播 include 而漏掉目标文件。
# CubeMX 的根目标使用 plain 签名，本调用保持一致。
get_property(app_objects GLOBAL PROPERTY SELF_OBJECT_TARGETS)
target_link_libraries(${CMAKE_PROJECT_NAME} self_os ${app_objects} self_drv self_bsp self_lib self_config self_app_tasks_api)
include("${PROJECT_SOURCE_DIR}/cmake/export_self_manifest.cmake")

# Generated Core code calls the composition entry; functional modules never see Assembly.
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/App/Assembly")
