# 本层维护所有 App 源文件；tools/sync_keil_project.py 从同一清单同步 MDK。
set(module_sources
    sentry/app_sentry_can.c
    common/communication/app_chassis_comm.c
    common/communication/app_gimbal_comm.c
    common/system/app_control.c
    common/system/app_init.c
    sentry/chassis/app_sentry_chassis.c
    sentry/gimbal/app_sentry_gimbal.c
    sentry/shoot/app_sentry_shoot.c
    Tasks/gimbal/app_task_gimbal.c
    Tasks/chassis/app_task_chassis.c
    Tasks/shoot/app_task_shoot.c
    Tasks/can/app_task_can.c
    Tasks/dbus/app_task_dbus.c
    Tasks/referee/app_task_referee.c
    Tasks/imu/app_task_imu.c
    Tasks/init/app_task_init.c
    Tasks/health/app_task_health.c
    Tasks/log/app_task_log.c
    Tasks/common/app_task_telemetry.c
    Tasks/common/app_task_monitor.c
    Tasks/common/app_rtos.c
    Tasks/common/app_tasks.c
    Tasks/init/app_rtos_board.c
    Tasks/can/app_task_can_port.c
    Tasks/log/app_task_log_port.c
)
set(module_includes
    Config
    common/data
    common/communication
    common/system
    sentry
    sentry/chassis
    sentry/gimbal
    sentry/shoot
    Tasks/can
    Tasks/chassis
    Tasks/dbus
    Tasks/gimbal
    Tasks/health
    Tasks/imu
    Tasks/init
    Tasks/log
    Tasks/referee
    Tasks/shoot
    Tasks/common
)

# 同属 App 的任务与算法分别编译；算法目标不依赖 Os/HAL。
set(algorithm_sources ${module_sources})
list(FILTER algorithm_sources EXCLUDE REGEX "^Tasks/")
set(task_sources ${module_sources})
list(FILTER task_sources INCLUDE REGEX "^Tasks/")
set(task_port_sources ${task_sources})
list(FILTER task_port_sources INCLUDE REGEX "(_port|app_rtos_board)\\.c$")
list(REMOVE_ITEM task_sources ${task_port_sources})
add_library(self_app OBJECT ${algorithm_sources})
foreach(include_dir IN LISTS module_includes)
    if(include_dir MATCHES "^Tasks/")
        # 任务 API 路径与 Keil 共用 module_includes，不再另维护一份。
        target_include_directories(self_app_tasks_api INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/${include_dir}")
    else()
        target_include_directories(self_app PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/${include_dir}")
    endif()
endforeach()
target_link_libraries(self_app PUBLIC self_app_contracts self_lib PRIVATE self_config)

add_library(self_app_tasks OBJECT ${task_sources})
target_link_libraries(self_app_tasks
    PUBLIC self_app_tasks_api
    PRIVATE self_os self_app self_lib)

# HAL/Bsp 只供硬件适配与板级装配编译；不传给任务循环和纯算法。
add_library(self_app_task_ports OBJECT ${task_port_sources})
target_link_libraries(self_app_task_ports
    PRIVATE self_app_tasks_api self_app self_os self_drv self_bsp self_lib)

file(GLOB_RECURSE module_headers CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.def")
# 头文件归入对应目标，.def 只供包含，不单独参与编译或链接。
set_source_files_properties(${module_headers} PROPERTIES HEADER_FILE_ONLY TRUE)
set(task_headers ${module_headers})
list(FILTER task_headers INCLUDE REGEX "/Tasks/")
list(FILTER module_headers EXCLUDE REGEX "/Tasks/")
target_sources(self_app PRIVATE ${module_headers})
target_sources(self_app_tasks PRIVATE ${task_headers})
