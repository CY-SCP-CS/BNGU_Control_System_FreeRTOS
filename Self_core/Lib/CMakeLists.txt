# 本层维护自己的源文件；tools/sync_keil_project.py 从同一清单同步 MDK。
set(module_sources
    Lib_filter/lib_filter.c
    Lib_math/lib_math.c
    Lib_math/lib_periodic.c
    Lib_pid/lib_pid.c
)
set(module_includes
    Lib_Typedef
    Lib_filter
    Lib_math
    Lib_pid
)
add_library(self_lib OBJECT ${module_sources})
foreach(include_dir IN LISTS module_includes)
    target_include_directories(self_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/${include_dir}")
endforeach()
file(GLOB_RECURSE module_headers CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.def")
# .h/.def 仅供包含或 IDE 展示；避免 .def 被识别为链接定义文件。
set_source_files_properties(${module_headers} PROPERTIES HEADER_FILE_ONLY TRUE)
target_sources(self_lib PRIVATE ${module_headers})
