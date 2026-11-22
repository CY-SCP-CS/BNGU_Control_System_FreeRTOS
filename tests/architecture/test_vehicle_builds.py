"""Check actual compiler commands for cross-vehicle source leakage after firmware builds."""
from pathlib import Path
import json
import re
import sys

root = Path(__file__).resolve().parents[2]
for preset in sys.argv[1:]:
    entries = json.loads((root / 'build' / preset / 'compile_commands.json').read_text())
    selected = set()
    for entry in entries:
        match = re.search(r'app_(gimbal|chassis|shoot|navigation|armored)_algorithm', entry['command'])
        if match:
            selected.add(match.group(1))
    expected = (set() if '_GIMBAL_' in preset and preset.startswith(('ARMORED', 'NAVIGATION')) else {'navigation'} if preset.startswith('NAVIGATION') else
                {'armored'} if preset.startswith('ARMORED') else
                {'chassis'} if preset.startswith('CHASSIS') else {'gimbal', 'shoot'})
    assert selected == expected, (preset, selected, expected)
    navigation_task = any(Path(entry['file']).name == 'app_task_navigation.c' for entry in entries)
    assert navigation_task == (preset.startswith('NAVIGATION') and '_GIMBAL_' not in preset), preset
    print('PASS compiled vehicle sources:', preset, ', '.join(sorted(selected)))
