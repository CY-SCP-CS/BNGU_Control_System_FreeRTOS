"""检查层间依赖和函数文档；--strict-docs 将文档缺项视为错误。"""
from pathlib import Path
import argparse
import re
import json
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--strict-docs", action="store_true")
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
errors, docs = [], []
for path in (root / "Self_core").rglob("*"):
    if path.suffix not in (".c", ".h"):
        continue
    text = path.read_text(encoding="utf-8-sig")
    relative = path.relative_to(root).as_posix()
    layer = path.relative_to(root / "Self_core").parts[0]
    for include in re.findall(r'#include "([^"]+)"', text):
        if layer == "Lib" and include.startswith(("app_", "drv_", "bsp_", "cmsis_os")):
            errors.append(f"{relative}: Lib depends on {include}")
        if layer == "Os" and include.startswith(("app_task", "drv_", "bsp_", "app_board", "app_can", "app_imu")):
            errors.append(f"{relative}: Os depends on business {include}")
        if "/Contracts/" in relative and include.startswith(("drv_", "bsp_", "cmsis_os", "FreeRTOS")):
            errors.append(f"{relative}: business contract depends on {include}")
    pattern = r'^(?:static\s+)?(?:inline\s+)?(?:const\s+)?[\w* ]+?\s+(\w+)\s*\(([^;{}]*)\)\s*(?:;|\{)'
    for m in re.finditer(pattern, text, re.M):
        name, parameters = m.groups()
        if name in ("if", "for", "while", "switch", "return", "sizeof"):
            continue
        preceding = text[:m.start()].rstrip()
        block = re.search(r"/\*\*([^*]*(?:\*(?!/)[^*]*)*)\*/$", preceding, re.S)
        comment = block.group(1) if block else ""
        missing = [tag for tag in ("@brief", "@return", "@note") if tag not in comment]
        for parameter in parameters.split(","):
            if parameter.strip() in ("", "void"):
                continue
            match = re.search(r"(\w+)\s*(?:\[[^]]*\])?\s*$", parameter)
            if match and not re.search(r"@param(?:\[[^]]+\])?\s+" + match.group(1) + r"\b", comment):
                missing.append("@param " + match.group(1))
        if missing:
            line = text[:m.start()].count("\n") + 1
            docs.append(f"{relative}:{line}: {name}: {', '.join(missing)}")
report = {"dependency_errors": errors, "documentation_findings": docs}
output = root / "build" / "architecture_audit.json"
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
print(f"dependency errors={len(errors)}; documentation findings={len(docs)}; {output}")
raise SystemExit(bool(errors or (args.strict_docs and docs)))
