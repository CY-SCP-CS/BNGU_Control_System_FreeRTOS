# Explicit inventory owned by this functional module.
app_module(can algorithm
    app_chassis_comm.c
    app_gimbal_comm.c
    app_vehicle_can.c
)
app_module(can runtime
    app_task_can.c
)
app_module(can port
    app_task_can_port.c
)

target_link_libraries(app_can_runtime PRIVATE app_health_api self_app_tasks_api)
