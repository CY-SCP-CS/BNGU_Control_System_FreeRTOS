add_subdirectory(Shoot)
if(NOT (APP_ROBOT STREQUAL "SENTRY" AND APP_BOARD STREQUAL "GIMBAL"))
    return()
endif()
# Explicit inventory owned by this functional module.
app_module(gimbal algorithm
    app_sentry_gimbal.c
)
app_module(gimbal runtime
    app_task_gimbal.c
)

target_link_libraries(app_gimbal_algorithm PRIVATE app_control_support_api)
target_include_directories(app_gimbal_algorithm PRIVATE "${PROJECT_SOURCE_DIR}/Self_core/App/Modules/control_support/internal")

target_link_libraries(app_gimbal_runtime PRIVATE app_can_api app_dbus_api app_imu_api self_app_tasks_api app_shoot_api app_telemetry_api)

app_module(gimbal port app_task_gimbal_port.c)
