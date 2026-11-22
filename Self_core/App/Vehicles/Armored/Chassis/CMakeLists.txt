if(NOT (APP_ROBOT STREQUAL "ARMORED" AND APP_BOARD STREQUAL "CHASSIS"))
    return()
endif()
# Algorithms compile for regression and future integration; no automatic actuator startup.
app_module(armored algorithm app_Armored_mecanum.c)
app_module(armored port app_Armored_joint.c)

target_link_libraries(app_armored_algorithm PRIVATE app_control_support_api)

target_include_directories(app_armored_algorithm PRIVATE "${PROJECT_SOURCE_DIR}/Self_core/App/Modules/control_support/internal")
