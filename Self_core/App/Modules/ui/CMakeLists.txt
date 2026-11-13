app_module(ui algorithm app_Armored_ui.c app_ui_io.c
    ui_generated/ui_g.c ui_generated/ui_interface.c)
target_include_directories(app_ui_algorithm PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/ui_generated")
target_link_libraries(app_ui_algorithm PRIVATE self_drv_types)
app_module(ui runtime app_ui_service.c)
target_include_directories(app_ui_runtime PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/ui_generated")
app_module(ui port app_ui_port.c)
