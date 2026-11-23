"""Compile pure migrated modules with host GCC. No hardware or RTOS mocks."""
from pathlib import Path
import argparse, subprocess, tempfile
p=argparse.ArgumentParser();p.add_argument('--cc',default='gcc');args=p.parse_args()
root=Path(__file__).resolve().parents[2]
paths = ['App/Config','App/Contracts','App/Modules/dbus','App/Vehicles/Sentry/Gimbal',
         'App/Vehicles/Sentry/Gimbal/internal','App/Vehicles/Sentry/Chassis','App/Vehicles/Sentry/Chassis/internal',
         'App/Vehicles/Sentry/Gimbal/Shoot','App/Vehicles/Sentry/Gimbal/Shoot/internal','App/Modules/shoot','App/Modules/shoot/internal','App/Vehicles/Navigation/Chassis',
         'App/Vehicles/Navigation/Chassis/internal','App/Modules/can','App/Modules/can/internal',
         'App/Modules/control_support/internal','Drv/Drv_motor','Drv/Drv_referee','Drv/Drv_vofa',
         'Drv/Drv_dbus','Lib/Lib_math','Lib/Lib_pid','Lib/Lib_filter','Lib/Lib_Typedef']
includes=[str(root/'Self_core'/p) for p in paths]
includes.append(str(root/'Self_core'))
sources=['tests/port/test_port.c','Self_core/Drv/Drv_game_controller/drv_game_controller.c','Self_core/App/Modules/dbus/app_control.c','Self_core/App/Vehicles/Sentry/Gimbal/app_sentry_gimbal.c','Self_core/App/Vehicles/Sentry/Chassis/app_sentry_chassis.c','Self_core/App/Vehicles/Sentry/Gimbal/Shoot/app_sentry_shoot.c','Self_core/App/Vehicles/Navigation/Chassis/chassis.c','Self_core/App/Vehicles/Navigation/Chassis/navigation_ctrl.c','Self_core/App/Modules/can/app_vehicle_can.c','Self_core/App/Modules/can/app_core_protocol.c','Self_core/App/Modules/can/app_chassis_comm.c','Self_core/App/Modules/can/app_gimbal_comm.c','Self_core/Drv/Drv_motor/drv_motor.c','Self_core/Drv/Drv_referee/drv_referee.c','Self_core/Drv/Drv_vofa/drv_vofa.c','Self_core/Lib/Lib_math/lib_math.c','Self_core/Lib/Lib_pid/lib_pid.c','Self_core/Lib/Lib_filter/lib_filter.c']
with tempfile.TemporaryDirectory() as tmp:
    for robot,board in [('ROBOT_SENTRY','BOARD_GIMBAL'),('ROBOT_SENTRY','BOARD_CHASSIS'),('ROBOT_NAVIGATION','BOARD_CHASSIS')]:
        binary=str(Path(tmp)/('port_'+robot+'_'+board+'.exe'))
        subprocess.run([args.cc,'-std=c11','-mno-ms-bitfields','-Wall','-Wextra','-DCURRENT_ROBOT='+robot,'-DCURRENT_BOARD='+board,*['-I'+p for p in includes],*[str(root/p) for p in sources],'-lm','-o',binary],check=True)
        subprocess.run([binary],check=True)
