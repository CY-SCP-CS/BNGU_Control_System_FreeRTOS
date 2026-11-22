# Explicit inventory owned by this functional module.
app_module(health runtime
    app_task_health.c
    app_task_monitor.c
)

target_link_libraries(app_health_runtime PRIVATE self_app_tasks_api)
