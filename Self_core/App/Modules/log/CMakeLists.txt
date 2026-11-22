# Explicit inventory owned by this functional module.
app_module(log runtime
    app_task_log.c
)
app_module(log port
    app_task_log_port.c
)

target_link_libraries(app_log_runtime PRIVATE app_health_api self_app_tasks_api app_telemetry_api)

target_link_libraries(app_log_port PRIVATE self_app_tasks_api)
