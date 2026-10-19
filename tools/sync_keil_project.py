"""从各层 CMakeLists.txt 的显式清单同步 Self_core 源文件和包含路径。

用法（工程根目录）：
    python tools/sync_keil_project.py
    python tools/sync_keil_project.py --board CHASSIS --log-mode TEXT

按实际层级分组（含 App/Tasks/<任务>、Os/FreeRTOS、Os/Port），更新源文件/包含路径及显式指定的应用宏。
不替换 FreeRTOS 的 RVDS port、
CubeMX 文件、调试器、优化等级或链接设置。同步不代表已构建或已上板验证。
"""
from pathlib import Path
import argparse
import os
import re
import xml.etree.ElementTree as ET


def cmake_list(text, name):
    match = re.search(r"set\(" + re.escape(name) + r"\s+(.*?)\)", text, re.S)
    if not match:
        raise ValueError(f"缺少 {name} 清单")
    body = re.sub(r"#[^\n]*", "", match.group(1))
    return [s.strip('"') for s in body.split()]


def sync(root, project, board=None, log_mode=None):
    inventory = {}
    include_dirs = [root / "Self_core"]
    for layer in ("Lib", "Bsp", "Drv", "App", "Os"):
        folder = root / "Self_core" / layer
        text = (folder / "CMakeLists.txt").read_text(encoding="utf-8-sig")
        sources = [folder / name for name in cmake_list(text, "module_sources")]
        includes = [folder / name for name in cmake_list(text, "module_includes")]
        for path in sources + includes:
            if not path.exists():
                raise FileNotFoundError(path)
        for source in sources:
            relative = source.relative_to(folder)
            group = layer
            if layer == "App" and relative.parts[0] == "Tasks":
                group = "App/" + "/".join(relative.parts[:-1])
            elif layer == "Os":
                group = "Os/" + relative.parts[0]
            inventory.setdefault(group, []).append(source)
        include_dirs += includes

    original = project.read_bytes()
    document = ET.fromstring(original)
    targets = document.findall("./Targets/Target")
    if not targets:
        raise ValueError("工程没有 Target，未写入")
    for target in targets:
        groups = target.find("Groups")
        if groups is None:
            groups = ET.SubElement(target, "Groups")
        for group in list(groups):
            name = group.findtext("GroupName", "")
            if name == "Self_core" or name.startswith("Self_core/"):
                groups.remove(group)
        for layer, sources in inventory.items():
            group = ET.SubElement(groups, "Group")
            ET.SubElement(group, "GroupName").text = "Self_core/" + layer
            files = ET.SubElement(group, "Files")
            for path in sources:
                item = ET.SubElement(files, "File")
                ET.SubElement(item, "FileName").text = path.name
                ET.SubElement(item, "FileType").text = "1"
                ET.SubElement(item, "FilePath").text = os.path.relpath(path, project.parent).replace("\\", "/")

        controls = target.find("./TargetOption/TargetArmAds/Cads/VariousControls")
        if controls is None:
            raise ValueError("没有 C 编译器配置，未写入")
        include = controls.find("IncludePath")
        if include is None:
            include = ET.SubElement(controls, "IncludePath")
        # Preserve all non-Self_core include paths, including the toolchain-specific RTOS port.
        paths = [p for p in (include.text or "").split(";") if p and "self_core" not in p.lower()]
        paths += [os.path.relpath(p, project.parent).replace("\\", "/") for p in include_dirs]
        include.text = ";".join(dict.fromkeys(paths))

        if board or log_mode:
            defines = controls.find("Define")
            if defines is None:
                defines = ET.SubElement(controls, "Define")
            values = [v.strip() for v in (defines.text or "").split(",") if v.strip()]
            overrides = {}
            if board:
                overrides["CURRENT_BOARD"] = "BOARD_" + board
            if log_mode:
                overrides["APP_LOG_VOFA"] = "1" if log_mode == "VOFA" else "0"
            values = [v for v in values if v.split("=", 1)[0].strip() not in overrides]
            values += [key + "=" + value for key, value in overrides.items()]
            defines.text = ",".join(values)

    ET.indent(document, space="  ")
    body = ET.tostring(document, encoding="utf-8", short_empty_elements=True)
    # Retain the original project as a sibling backup before the first real change.
    result = b'<?xml version="1.0" encoding="UTF-8"?>\n' + body + b"\n"
    result = result.replace(b"\r\n", b"\n").replace(b"\n", b"\r\n")
    if result != original:
        backup = project.with_suffix(project.suffix + ".before_self_core_sync.bak")
        if not backup.exists():
            backup.write_bytes(original)
        project.write_bytes(result)
    return sum(map(len, inventory.values()))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--board", choices=("GIMBAL", "CHASSIS"))
    parser.add_argument("--log-mode", choices=("TEXT", "VOFA"))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    project = root / "MDK-ARM/BNGU_Control_System_FreeRTOS.uvprojx"
    count = sync(root, project, args.board, args.log_mode)
    print(f"已同步 {count} 个 Self_core 源文件：{project}")
