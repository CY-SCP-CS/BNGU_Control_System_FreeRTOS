"""Audit resolved Self_core includes, private boundaries, cycles and algorithm purity.

The graph is conservative: all textual conditional branches are included. External
HAL/CMSIS implementation graphs and runtime callback execution are outside its scope.
Use --compile-commands to also check compiler include visibility per object target.
"""
from pathlib import Path
from collections import defaultdict
import argparse
import json
import re

INCLUDE = re.compile(r'^\s*#\s*include\s*["<]([^">]+)[">]', re.M)


def owner(path):
    parts = path.parts
    if parts[:2] == ('App', 'Vehicles') and len(parts) > 3 and parts[3] == 'Contracts':
        return 'Contracts'
    if parts[:2] == ('App', 'Vehicles') and len(parts) > 3 and parts[3] == 'Config':
        return 'Config'
    if parts[:2] == ('App', 'Vehicles'):
        paths = {'Sentry/Gimbal': 'gimbal', 'Sentry/Gimbal/Shoot': 'shoot', 'Sentry/Chassis': 'chassis', 'Navigation/Chassis': 'navigation', 'Armored/Chassis': 'armored'}
        for prefix in sorted(paths, key=len, reverse=True):
            if '/'.join(parts[2:]).startswith(prefix + '/'):
                return 'module:' + paths[prefix]
    if path.name in ('app_can_data.h', 'app_ctrl_data.h', 'app_can_commands.h', 'app_core_protocol.h'):
        return 'Contracts'
    if parts[:2] == ('App', 'Modules'):
        return 'module:' + parts[2]
    if parts[:2] in [('App', 'Runtime'), ('App', 'Assembly'), ('App', 'Contracts'), ('App', 'Config')]:
        return parts[1]
    return 'Config' if path.name == 'project_cfg.h' else parts[0]


def cycles(graph):
    """Return strongly connected components, including explicit self-includes."""
    index, low, stack, active, result = {}, {}, [], set(), []
    def visit(node):
        index[node] = low[node] = len(index)
        stack.append(node)
        active.add(node)
        for target in sorted(graph.get(node, ())):
            if target not in index:
                visit(target)
                low[node] = min(low[node], low[target])
            elif target in active:
                low[node] = min(low[node], index[target])
        if low[node] == index[node]:
            component = []
            while True:
                target = stack.pop()
                active.remove(target)
                component.append(target)
                if target == node:
                    break
            if len(component) > 1 or node in graph.get(node, ()):
                result.append(sorted(component))
    nodes = set(graph) | {x for values in graph.values() for x in values}
    for node in sorted(nodes):
        if node not in index:
            visit(node)
    return result


def audit(root, compile_commands=None):
    root = Path(root).resolve()
    core = root / 'Self_core'
    files = sorted(p for p in core.rglob('*') if p.suffix in ('.c', '.h', '.inc', '.def'))
    by_name = defaultdict(list)
    for path in files:
        by_name[path.name].append(path)
    graph, modules, layers = defaultdict(set), defaultdict(set), defaultdict(set)
    errors = []
    allowed = {
        'Lib': {'Lib'}, 'Bsp': {'Bsp', 'Config', 'Lib'},
        'Drv': {'Drv', 'Bsp', 'Config', 'Lib'}, 'Os': {'Os'},
        'Contracts': {'Contracts', 'Config', 'Lib'},
        'Config': {'Config', 'Lib'},
        'Runtime': {'Runtime', 'Contracts', 'Config', 'Os', 'Lib'},
    }
    for path in files:
        relative = path.relative_to(core)
        source = relative.as_posix()
        origin = owner(relative)
        for include in INCLUDE.findall(path.read_text(encoding='utf-8-sig')):
            local = (path.parent / include).resolve()
            if local in files:
                target = local
            else:
                matches = by_name.get(Path(include).name, [])
                if len(matches) != 1:
                    if len(matches) > 1:
                        errors.append(f'{source}: ambiguous include {include}')
                    continue
                target = matches[0]
            rel_target = target.relative_to(core)
            destination = owner(rel_target)
            graph[source].add(rel_target.as_posix())
            if origin != destination:
                layers[origin].add(destination)
                if origin.startswith('module:') and destination.startswith('module:'):
                    modules[origin].add(destination)
            if origin in allowed and destination not in allowed[origin]:
                errors.append(f'{source}: forbidden layer dependency {origin} -> {destination}: {include}')
            if origin.startswith('module:') and destination == 'Assembly':
                errors.append(f'{source}: module depends on Assembly: {include}')
            private = 'internal' in rel_target.parts or 'ui_generated' in rel_target.parts
            legacy_consumer = (destination == 'module:control_support' and
                               origin in {'module:gimbal', 'module:chassis', 'module:shoot',
                                          'module:navigation', 'module:armored'})
            if private and origin != destination and origin != 'Assembly' and not legacy_consumer:
                errors.append(f'{source}: private header escape: {include}')
    file_cycles, module_cycles, layer_cycles = cycles(graph), cycles(modules), cycles(layers)
    for name, findings in [('include', file_cycles), ('module', module_cycles), ('layer', layer_cycles)]:
        for component in findings:
            errors.append(f'{name} cycle: ' + ' -> '.join(component))
    # Use the actual compiler command to reject accidentally widened include visibility.
    if compile_commands:
        for entry in json.loads(Path(compile_commands).read_text(encoding='utf-8')):
            command = entry.get('command', ' '.join(entry.get('arguments', [])))
            match = re.search(r'app_(\w+)_algorithm\.dir', command)
            if not match:
                continue
            module = match.group(1)
            source = Path(entry['file']).resolve().relative_to(core).as_posix()
            pending, seen = [source], set()
            while pending:
                included = pending.pop()
                if included in seen:
                    continue
                seen.add(included)
                kind = owner(Path(included))
                if kind in {'Runtime', 'Assembly', 'Bsp', 'Os'}:
                    errors.append(f'{source}: algorithm transitively includes platform {included}')
                body = (core / included).read_text(encoding='utf-8-sig')
                for header in INCLUDE.findall(body):
                    if header.startswith(('stm32', 'cmsis_os', 'FreeRTOS')):
                        errors.append(f'{source}: algorithm transitively includes {header}')
                pending.extend(graph.get(included, ()))
            for directory in re.findall(r'-I(?:"([^"]+)"|([^\s]+))', command):
                value = next(x for x in directory if x).replace('\\', '/')
                if any(marker in value for marker in ('/Assembly', '/Runtime', '/Bsp', '/Os/', '/Drivers/', '/Middlewares/')):
                    errors.append(f'{entry["file"]}: algorithm sees platform directory {value}')
                if '/Vehicles/' in value and value.endswith('/internal'):
                    vehicle_owner = owner(Path('App/Vehicles/' + value.split('/Vehicles/', 1)[1]))
                    if vehicle_owner != 'module:' + module:
                        errors.append(f'{entry["file"]}: algorithm sees another vehicle private directory {value}')
                if '/Modules/' in value:
                    suffix = value.split('/Modules/', 1)[1].split('/')
                    if suffix[0] != module and suffix[-1] == 'internal' and suffix[0] != 'control_support':
                        errors.append(f'{entry["file"]}: algorithm sees another module private directory {value}')
    return {'errors': sorted(set(errors)), 'files_checked': len(files),
            'include_cycles': file_cycles, 'module_cycles': module_cycles,
            'layer_cycles': layer_cycles,
            'dependencies': {key: sorted(value) for key, value in sorted(layers.items())}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compile-commands', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = audit(Path(__file__).resolve().parents[1], args.compile_commands)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    raise SystemExit(bool(report['errors']))


if __name__ == '__main__':
    main()
