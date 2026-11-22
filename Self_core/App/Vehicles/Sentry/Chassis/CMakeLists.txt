if(NOT (APP_ROBOT STREQUAL "SENTRY" AND APP_BOARD STREQUAL "CHASSIS"))
    return()
endif()
# Explicit inventory owned by this functional module.
app_module(chassis algorithm
    app_sentry_chassis.c
)
app_module(chassis runtime
    app_task_chassis.c
)

target_link_libraries(app_chassis_algorithm PRIVATE app_control_support_api)
target_include_directories(app_chassis_algorithm PRIVATE "${PROJECT_SOURCE_DIR}/Self_core/App/Modules/control_support/internal")

target_link_libraries(app_chassis_runtime PRIVATE app_can_api app_dbus_api app_imu_api app_navigation_api self_app_tasks_api app_telemetry_api)

app_module(chassis port app_task_chassis_port.c)
