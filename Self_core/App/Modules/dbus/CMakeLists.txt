# Explicit inventory owned by this functional module.
app_module(dbus algorithm
    app_control.c
)
app_module(dbus runtime
    app_task_dbus.c
)

app_module(dbus port app_task_dbus_port.c)

target_link_libraries(app_dbus_runtime PRIVATE app_health_api self_app_tasks_api)

target_link_libraries(app_dbus_port PRIVATE self_app_tasks_api)

target_link_libraries(app_dbus_algorithm PRIVATE self_drv_types)
