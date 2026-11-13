"""Compile pure migrated modules with host GCC. No hardware or RTOS mocks."""
from pathlib import Path
import argparse, subprocess, tempfile
p=argparse.ArgumentParser();p.add_argument('--cc',default='gcc');args=p.parse_args()
root=Path(__file__).resolve().parents[2]
includes=[str(p) for p in (root/'Self_core').rglob('*') if p.is_dir()]
includes.append(str(root/'Self_core'))
sources=['tests/port/test_port.c','Self_core/App/Modules/dbus/app_control.c','Self_core/App/Modules/gimbal/app_sentry_gimbal.c','Self_core/App/Modules/shoot/app_sentry_shoot.c','Self_core/App/Modules/navigation/chassis.c','Self_core/App/Modules/navigation/navigation_ctrl.c','Self_core/App/Modules/can/app_sentry_can.c','Self_core/App/Modules/can/app_chassis_comm.c','Self_core/App/Modules/can/app_gimbal_comm.c','Self_core/Drv/Drv_motor/drv_motor.c','Self_core/Drv/Drv_referee/drv_referee.c','Self_core/Drv/Drv_vofa/drv_vofa.c','Self_core/Lib/Lib_math/lib_math.c','Self_core/Lib/Lib_pid/lib_pid.c','Self_core/Lib/Lib_filter/lib_filter.c']
with tempfile.TemporaryDirectory() as tmp:
    for robot,board in [('ROBOT_SENTRY','BOARD_GIMBAL'),('ROBOT_NAVIGATION','BOARD_CHASSIS')]:
        binary=str(Path(tmp)/('port_'+robot+'.exe'))
        subprocess.run([args.cc,'-std=c11','-mno-ms-bitfields','-Wall','-Wextra','-DCURRENT_ROBOT='+robot,'-DCURRENT_BOARD='+board,*['-I'+p for p in includes],*[str(root/p) for p in sources],'-lm','-o',binary],check=True)
        subprocess.run([binary],check=True)
