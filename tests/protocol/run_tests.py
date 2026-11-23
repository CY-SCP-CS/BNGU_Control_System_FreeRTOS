from pathlib import Path
import os,subprocess,tempfile
root=Path(__file__).resolve().parents[2]
cc=os.environ.get('HOST_CC','gcc')
paths=['Self_core','Self_core/App/Config','Self_core/App/Contracts','Self_core/App/Modules/can','Self_core/App/Modules/can/internal','Self_core/Lib/Lib_math','Self_core/Lib/Lib_Typedef']
files=['tests/protocol/test_protocol.c','Self_core/App/Modules/can/app_chassis_comm.c','Self_core/App/Modules/can/app_gimbal_comm.c','Self_core/App/Modules/can/app_vehicle_can.c','Self_core/App/Modules/can/app_core_protocol.c','Self_core/Lib/Lib_math/lib_math.c']
with tempfile.TemporaryDirectory() as temp:
 for board in ('BOARD_GIMBAL','BOARD_CHASSIS'):
  exe=str(Path(temp)/(board+'.exe'))
  subprocess.run([cc,'-std=c11','-Wall','-Wextra','-Werror','-DAPP_CAN_DM_IMU_BUS=2','-DCURRENT_BOARD='+board,*['-I'+str(root/p) for p in paths],*[str(root/p) for p in files],'-lm','-o',exe],check=True)
  subprocess.run([exe],check=True)
