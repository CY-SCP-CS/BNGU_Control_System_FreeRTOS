# 应用配置只定义一次；App 算法与任务分别声明依赖。
add_library(self_config INTERFACE)
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
if(NOT APP_ROBOT STREQUAL "SENTRY")
    message(FATAL_ERROR "Only APP_ROBOT=SENTRY is implemented")
endif()

# 纯类型依赖不把驱动端口的 HAL/Bsp 路径传给 App。
add_library(self_drv_types INTERFACE)
file(GLOB driver_header_dirs LIST_DIRECTORIES true "${CMAKE_CURRENT_SOURCE_DIR}/Drv/Drv_*")
target_include_directories(self_drv_types INTERFACE ${driver_header_dirs} "${CMAKE_CURRENT_SOURCE_DIR}/Lib/Lib_Typedef")
add_library(self_app_contracts INTERFACE)
target_include_directories(self_app_contracts INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/App/common/data"
    "${CMAKE_CURRENT_SOURCE_DIR}/App/common/communication"
    "${CMAKE_CURRENT_SOURCE_DIR}/App/sentry"
    "${CMAKE_CURRENT_SOURCE_DIR}/Lib/Lib_math")
target_link_libraries(self_app_contracts INTERFACE self_drv_types self_config)

# 仅传递头文件/API，不把任务 OBJECT 库反向链接进 Os，避免目标循环依赖。
add_library(self_os_api INTERFACE)
target_include_directories(self_os_api INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/Os/FreeRTOS"
    "${CMAKE_CURRENT_SOURCE_DIR}/Os/Port"
    "${PROJECT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2")
# CMSIS 声明本身不需要 HAL 或项目板型宏，平台依赖只在实现目标内。
add_library(self_app_tasks_api INTERFACE)
# 各任务的接口目录由 App/CMakeLists.txt 的 module_includes 统一填充。
target_link_libraries(self_app_tasks_api INTERFACE self_app_contracts self_os_api)

add_subdirectory(Lib)
add_subdirectory(Bsp)
add_subdirectory(Drv)
add_subdirectory(App)
add_subdirectory(Os)

# 所有 OBJECT 目标都直接加入最终链接，避免仅传播 include 而漏掉目标文件。
# CubeMX 的根目标使用 plain 签名，本调用保持一致。
target_link_libraries(${CMAKE_PROJECT_NAME} self_os self_app_tasks self_app_task_ports self_app self_drv self_bsp self_lib self_config)
