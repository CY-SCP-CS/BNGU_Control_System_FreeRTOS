"""Verify per-vehicle CAN mapping and reject removed/unsupported configurations."""
from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
cc = os.environ.get('HOST_CC', 'gcc')
includes = ['Self_core', 'Self_core/App/Config', 'Self_core/App/Contracts',
            'Self_core/App/Modules/can', 'Self_core/App/Modules/can/internal',
            'Self_core/Lib/Lib_math', 'Self_core/Lib/Lib_Typedef']
sources = ['tests/vehicles/test_can_vehicle.c', 'Self_core/App/Modules/can/app_vehicle_can.c',
           'Self_core/App/Modules/can/app_chassis_comm.c',
           'Self_core/App/Modules/can/app_gimbal_comm.c', 'Self_core/Lib/Lib_math/lib_math.c']
with tempfile.TemporaryDirectory() as tmp:
    for robot in ('SENTRY', 'NAVIGATION', 'ARMORED'):
        exe = str(Path(tmp) / (robot + '.exe'))
        subprocess.run([cc, '-std=c11', '-Wall', '-Wextra', '-Werror', '-mno-ms-bitfields',
                        '-DCURRENT_ROBOT=ROBOT_' + robot, '-DCURRENT_BOARD=BOARD_CHASSIS',
                        *['-I' + str(root / p) for p in includes],
                        *[str(root / p) for p in sources], '-lm', '-o', exe], check=True)
        subprocess.run([exe], check=True)
        print('PASS vehicle CAN isolation:', robot)
    for robot, board in [('HERO', 'CHASSIS')]:
        result = subprocess.run([cc, '-x', 'c', '-fsyntax-only', '-DCURRENT_ROBOT=ROBOT_' + robot,
                                 '-DCURRENT_BOARD=BOARD_' + board, '-I' + str(root / 'Self_core'), '-'],
                                input='#include "project_cfg.h"\n', text=True, capture_output=True)
        assert result.returncode != 0, (robot, board)
        print('PASS rejected configuration:', robot, board)
