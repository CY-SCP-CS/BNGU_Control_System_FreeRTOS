# 本层维护自己的源文件；tools/sync_keil_project.py 从同一清单同步 MDK。
set(module_sources
    Drv_buzzer/drv_buzzer.c
    Drv_buzzer/drv_buzzer_port.c
    Drv_dbus/drv_dbus.c
    Drv_dbus/drv_dbus_port.c
    Drv_imu/drv_imu_ekf.c
    Drv_imu/drv_imu_vqf.c
    Drv_imu/drv_imu.c
    Drv_imu/drv_imu_port.c
    Drv_mag/drv_mag.c
    Drv_mag/drv_mag_port.c
    Drv_led/drv_led.c
    Drv_led/drv_led_port.c
    Drv_melody/drv_melody.c
    Drv_motor/drv_motor.c
    Drv_motor/drv_motor_port.c
    Drv_power_measure/drv_power_measure.c
    Drv_power_measure/drv_power_measure_port.c
    Drv_referee/drv_referee.c
    Drv_referee/drv_referee_port.c
    Drv_vofa/drv_vofa.c
    Drv_vofa/drv_vofa_port.c
)
set(module_includes
    Drv_buzzer
    Drv_dbus
    Drv_imu
    Drv_mag
    Drv_led
    Drv_melody
    Drv_motor
    Drv_power_measure
    Drv_referee
    Drv_vofa
)
add_library(self_drv OBJECT ${module_sources})
foreach(include_dir IN LISTS module_includes)
    target_include_directories(self_drv PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/${include_dir}")
endforeach()
file(GLOB_RECURSE module_headers CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.def")
# .h/.def 仅供包含或 IDE 展示；避免 .def 被识别为链接定义文件。
set_source_files_properties(${module_headers} PROPERTIES HEADER_FILE_ONLY TRUE)
target_sources(self_drv PRIVATE ${module_headers})
target_link_libraries(self_drv PUBLIC self_drv_types PRIVATE self_bsp self_lib)
