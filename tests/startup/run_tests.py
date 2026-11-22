"""Compile real task composition and inject failure at every creation step."""
from pathlib import Path
import os
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
app = root / 'Self_core/App'
includes = [root / 'Self_core', app / 'Config', app / 'Contracts', app / 'Runtime',
            app / 'Runtime/internal', app / 'Assembly', root / 'Self_core/Os/FreeRTOS',
            root / 'Self_core/Os/Port', root / 'Self_core/Lib/Lib_math',
            root / 'Self_core/Lib/Lib_Typedef',
            root / 'Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2']
for name in ('can', 'chassis', 'dbus', 'gimbal', 'health', 'imu', 'log', 'navigation', 'referee', 'shoot', 'telemetry'):
    directory = app / 'Modules' / name
    vehicle_paths = {'gimbal': 'Sentry/Gimbal', 'chassis': 'Sentry/Chassis', 'navigation': 'Navigation/Chassis', 'armored': 'Armored/Chassis'}
    if name in vehicle_paths: directory = app / 'Vehicles' / vehicle_paths[name]
    includes.extend([directory, directory / 'internal'])
includes.extend(root / 'Self_core/Drv' / name for name in ('Drv_imu', 'Drv_referee'))
cc = os.environ.get('HOST_CC', 'gcc')
with tempfile.TemporaryDirectory() as temporary:
    for robot, board in (('ROBOT_SENTRY', 'BOARD_GIMBAL'), ('ROBOT_SENTRY', 'BOARD_CHASSIS'),
                         ('ROBOT_NAVIGATION', 'BOARD_CHASSIS'), ('ROBOT_ARMORED', 'BOARD_CHASSIS'), ('ROBOT_NAVIGATION', 'BOARD_GIMBAL'), ('ROBOT_ARMORED', 'BOARD_GIMBAL')):
        exe = str(Path(temporary) / (robot + '_' + board + '.exe'))
        subprocess.run([cc, '-std=c11', '-Wall', '-Wextra', '-Werror', '-mno-ms-bitfields',
                        '-DCURRENT_BOARD=' + board, '-DCURRENT_ROBOT=' + robot, *['-I' + str(p) for p in includes],
                        str(app / 'Assembly/app_rtos_setup.c'),
                        str(root / 'tests/startup/test_startup.c'), '-o', exe], check=True)
        result = subprocess.check_output([exe, '0'], text=True)
        count = int(re.search(r'STARTUP_STEPS=(\d+)', result).group(1))
        for failure in range(1, count + 1):
            subprocess.run([exe, str(failure)], check=True)
        print(f'PASS {robot}/{board}: {count} startup failure points, rollback, retry and idempotency')
