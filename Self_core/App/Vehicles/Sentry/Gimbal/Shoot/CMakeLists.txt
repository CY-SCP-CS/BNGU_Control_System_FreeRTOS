# Sentry/GIMBAL owns launcher control; the command snapshot remains shared.
if(APP_ROBOT STREQUAL "SENTRY" AND APP_BOARD STREQUAL "GIMBAL")
    app_module(shoot_control runtime app_task_shoot_control.c)
    target_link_libraries(app_shoot_control_runtime PRIVATE app_shoot_api app_can_api app_telemetry_api self_app_tasks_api)
    app_module(shoot algorithm app_sentry_shoot.c)
    target_link_libraries(app_shoot_algorithm PRIVATE app_control_support_api)
    target_include_directories(app_shoot_algorithm PRIVATE "${PROJECT_SOURCE_DIR}/Self_core/App/Modules/control_support/internal")
endif()


if(TARGET app_shoot_control_runtime)
 target_include_directories(app_shoot_control_runtime PRIVATE "${PROJECT_SOURCE_DIR}/Self_core/App/Modules/shoot/internal")
endif()
