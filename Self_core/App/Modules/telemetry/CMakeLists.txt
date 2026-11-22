# Explicit inventory owned by this functional module.
app_module(telemetry runtime
    app_task_telemetry.c
)

target_link_libraries(app_telemetry_runtime PRIVATE self_app_tasks_api)
