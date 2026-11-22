# Explicit inventory owned by this functional module.
app_module(referee runtime
    app_task_referee.c
)

app_module(referee port app_task_referee_port.c)

target_link_libraries(app_referee_runtime PRIVATE app_health_api self_app_tasks_api app_ui_api)

target_link_libraries(app_referee_port PRIVATE self_app_tasks_api)
