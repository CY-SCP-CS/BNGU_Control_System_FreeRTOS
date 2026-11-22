"""Check real telemetry snapshots and VOFA frames for both board configurations."""
from pathlib import Path
import argparse
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default='gcc')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    includes = [
        'Self_core', 'Self_core/App/Config', 'Self_core/App/Contracts',
        'Self_core/App/Runtime', 'Self_core/App/Modules/telemetry',
        'Self_core/App/Modules/telemetry/internal',
        'Self_core/App/Modules/can', 'Self_core/App/Modules/control_support',
        'Self_core/Os/FreeRTOS', 'Self_core/Lib/Lib_math', 'Self_core/Lib/Lib_Typedef',
        'Self_core/Drv/Drv_vofa',
        'Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2',
    ]
    sources = [
        'tests/telemetry/test_telemetry.c',
        'Self_core/App/Modules/telemetry/app_task_telemetry.c',
        'Self_core/Os/FreeRTOS/app_rtos_objects.c',
        'Self_core/Drv/Drv_vofa/drv_vofa.c',
    ]
    with tempfile.TemporaryDirectory() as temp:
        for board in ['BOARD_GIMBAL', 'BOARD_CHASSIS']:
            binary = str(Path(temp) / (board + '.exe'))
            subprocess.run([
                args.cc, '-std=c11', '-Wall', '-Wextra', '-Werror', '-mno-ms-bitfields',
                '-DCURRENT_BOARD=' + board,
                *['-I' + str(root / path) for path in includes],
                *[str(root / path) for path in sources], '-lm', '-o', binary,
            ], check=True)
            subprocess.run([binary], check=True)


if __name__ == '__main__':
    main()
