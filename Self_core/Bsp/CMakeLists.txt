# 本层维护自己的源文件；tools/sync_keil_project.py 从同一清单同步 MDK。
set(module_sources
    bsp_adc.c
    bsp_can.c
    bsp_flash.c
    bsp_gpio.c
    bsp_i2c.c
    bsp_spi.c
    bsp_tim.c
    bsp_uart.c
)
set(module_includes
    .
)
add_library(self_bsp OBJECT ${module_sources})
foreach(include_dir IN LISTS module_includes)
    target_include_directories(self_bsp PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/${include_dir}")
endforeach()
file(GLOB_RECURSE module_headers CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.def")
# .h/.def 仅供包含或 IDE 展示；避免 .def 被识别为链接定义文件。
set_source_files_properties(${module_headers} PROPERTIES HEADER_FILE_ONLY TRUE)
target_sources(self_bsp PRIVATE ${module_headers})
target_link_libraries(self_bsp PUBLIC stm32cubemx self_config)
