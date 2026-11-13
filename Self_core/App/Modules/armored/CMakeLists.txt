# Algorithms compile for regression and future integration; no automatic actuator startup.
app_module(armored algorithm app_Armored_mecanum.c)
app_module(armored port app_Armored_joint.c app_armored_io.c)
