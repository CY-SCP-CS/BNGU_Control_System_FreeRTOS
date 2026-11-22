"""Exercise the real IST8310 core and board acquisition with injected failures."""
from pathlib import Path
import os
import shutil
import subprocess

root = Path(__file__).resolve().parents[2]
out = root / "build/host-tests"
out.mkdir(parents=True, exist_ok=True)
cc = os.environ.get("HOST_CC") or shutil.which("gcc")
if cc is None:
    raise SystemExit("gcc is required for host regression tests")
mag = "Self_core/Drv/Drv_mag"
base = [cc, "-std=c11", "-O1", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function"]
for name, sources, includes in [
    ("core", [f"{mag}/drv_mag.c"], [mag, "tests/mag"]),
    ("port", [f"{mag}/drv_mag.c", f"{mag}/drv_mag_port.c", "Self_core/Bsp/bsp_gpio.c", "Self_core/Bsp/bsp_i2c.c"],
     [mag, "tests/mag", "tests/mag/stubs", "Self_core/Bsp"]),
]:
    exe = out / f"mag_{name}.exe"
    flags = [arg for path in includes for arg in ["-I", str(root / path)]]
    if name != "core":
        flags.append("-DTEST_IRQ_STATE")
    subprocess.run([*base, *flags, str(root / f"tests/mag/test_{name}.c"),
                    *[str(root / src) for src in sources], "-lm", "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
