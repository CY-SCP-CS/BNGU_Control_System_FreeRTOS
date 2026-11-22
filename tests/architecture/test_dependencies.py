"""Negative tests: prove the architecture gate rejects forbidden dependencies."""
from pathlib import Path
import importlib.util
import tempfile
import unittest
import json

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location('dependencies', ROOT / 'tools/check_dependencies.py')
CHECK = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECK)


class DependencyTests(unittest.TestCase):
    def setUp(self):
        work = ROOT / "build" / "architecture-tests"
        work.mkdir(parents=True, exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(dir=work)
        self.root = Path(self.temp.name)
        self.addCleanup(self.temp.cleanup)

    def put(self, name, body=''):
        path = self.root / 'Self_core' / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(body, encoding='utf-8')
        return path

    def test_valid_contract(self):
        self.put('App/Contracts/value.h', '#include <stdint.h>')
        self.put('App/Modules/a/a.c', '#include "value.h"')
        self.assertEqual(CHECK.audit(self.root)['errors'], [])

    def test_relative_private_escape(self):
        self.put('App/Modules/b/internal/private.h')
        self.put('App/Modules/a/a.c', '#include "../b/internal/private.h"')
        self.assertTrue(any('private header' in x for x in CHECK.audit(self.root)['errors']))

    def test_reverse_layer(self):
        self.put('App/Modules/a/a.h')
        self.put('Lib/lib.c', '#include "a.h"')
        self.assertTrue(any('forbidden layer' in x for x in CHECK.audit(self.root)['errors']))

    def test_module_cycle(self):
        self.put('App/Modules/a/a.h', '#include "b.h"')
        self.put('App/Modules/b/b.h', '#include "a.h"')
        result = CHECK.audit(self.root)
        self.assertTrue(result['module_cycles'])
        self.assertTrue(result['include_cycles'])

    def test_transitive_algorithm_platform(self):
        self.put('Bsp/bsp.h', '#include "stm32f4xx_hal.h"')
        self.put('Drv/driver.h', '#include "bsp.h"')
        source = self.put('App/Modules/a/a.c', '#include "driver.h"')
        database = self.root / 'commands.json'
        database.write_text(json.dumps([{'file': str(source),
                                        'command': 'cc -o app_a_algorithm.dir/a.o'}]), encoding='utf-8')
        result = CHECK.audit(self.root, database)
        self.assertTrue(any('algorithm transitively' in x for x in result['errors']))


if __name__ == '__main__':
    unittest.main()
