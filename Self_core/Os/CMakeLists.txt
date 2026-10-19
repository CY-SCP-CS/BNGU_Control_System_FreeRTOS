# Os 只编译通用对象机制、FreeRTOS 专用适配和 CPU 适配，不依赖 App/Bsp/Drv。
set(module_sources
    FreeRTOS/app_rtos_objects.c
    FreeRTOS/app_rtos_kernel.c
    Port/app_rtos_port.c
)
set(module_includes
    FreeRTOS
    Port
)
add_library(self_os OBJECT ${module_sources})
target_link_libraries(self_os PUBLIC self_os_api PRIVATE stm32cubemx)
file(GLOB_RECURSE module_headers CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
target_sources(self_os PRIVATE ${module_headers})
