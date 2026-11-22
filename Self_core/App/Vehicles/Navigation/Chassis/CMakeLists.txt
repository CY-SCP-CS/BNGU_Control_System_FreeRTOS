if(NOT (APP_ROBOT STREQUAL "NAVIGATION" AND APP_BOARD STREQUAL "CHASSIS"))
    return()
endif()
app_module(navigation algorithm
    chassis.c
    navigation_ctrl.c
)

target_link_libraries(app_navigation_algorithm PRIVATE app_control_support_api)

target_include_directories(app_navigation_algorithm PRIVATE "${PROJECT_SOURCE_DIR}/Self_core/App/Modules/control_support/internal")

app_module(navigation runtime app_task_navigation.c)
app_module(navigation port app_task_navigation_port.c)
target_link_libraries(app_navigation_runtime PRIVATE app_can_api app_dbus_api app_imu_api app_telemetry_api self_app_tasks_api)
