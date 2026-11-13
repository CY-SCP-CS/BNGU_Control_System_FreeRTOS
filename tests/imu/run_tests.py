"""Run IMU host regressions; optionally compare to the pinned official VQF C++ source."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference-dir", type=Path,
                        help="directory containing official vqf.cpp and vqf.hpp (see VQF_LICENSE.txt)")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    out = root / "build" / "imu-tests"
    out.mkdir(parents=True, exist_ok=True)
    compiler = os.environ.get("HOST_CC") or shutil.which("gcc")
    if not compiler:
        raise SystemExit("Set HOST_CC to a native GCC compiler.")
    imu = root / "Self_core" / "Drv" / "Drv_imu"
    flags = ["-std=c11", "-O2", "-g", "-Wall", "-Wextra", "-Werror", "-Wpedantic",
             "-I", str(root / "Self_core/Lib/Lib_math"), "-I", str(imu), "-I", str(root / "Self_core" / "Lib" / "Lib_Typedef")]
    objects = []
    for name in ("drv_imu", "drv_imu_ekf", "drv_imu_vqf"):
        obj = out / (name + ".o")
        subprocess.run([compiler, *flags, "-c", str(imu / (name + ".c")), "-o", str(obj)], check=True)
        objects.append(str(obj))
    executable = out / ("test_imu.exe" if os.name == "nt" else "test_imu")
    subprocess.run([compiler, *flags, str(root / "tests/imu/test_imu.c"), *objects,
                    "-lm", "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
    if args.reference_dir:
        cxx = os.environ.get("HOST_CXX") or shutil.which("g++")
        if not cxx:
            raise SystemExit("Set HOST_CXX to a native C++ compiler for the reference comparison.")
        reference = args.reference_dir.resolve()
        executable = out / ("compare_vqf.exe" if os.name == "nt" else "compare_vqf")
        subprocess.run([cxx, "-std=c++11", "-O2", "-DVQF_SINGLE_PRECISION", "-I", str(reference),
                        "-I", str(root / "Self_core/Lib/Lib_math"), "-I", str(imu), "-I", str(root / "Self_core/Lib/Lib_Typedef"),
                        str(root / "tests/imu/compare_vqf.cpp"), str(reference / "vqf.cpp"),
                        *objects, "-lm", "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
