# Explicit inventory owned by this functional module.
app_module(imu runtime
    app_task_imu.c
)

app_module(imu port app_task_imu_port.c)

target_link_libraries(app_imu_runtime PRIVATE app_health_api self_app_tasks_api)

target_link_libraries(app_imu_port PRIVATE app_health_api self_app_tasks_api)
