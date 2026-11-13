# IMU host verification

Run from the repository root with a native GCC compiler and Python:

```powershell
python tests/imu/run_tests.py
```

`HOST_CC` can select a native compiler. Production modules compile as ISO C11
with `-Wall -Wextra -Werror -Wpedantic`. The tests require no HAL or hardware.
They cover three-axis rotations, noisy analytic motion, gyro bias convergence,
positive-definite EKF covariance, acceleration rejection, invalid inputs,
gyro-only updates, switching/reset/alignment, VQF rest detection, an inverted
pose, and sample-period changes. Generated files are under `build/imu-tests`.

Mounting tests cover identity/default behavior, the +90-degree vertical-board
mount, both sides of the raw Euler singularity, combined tilt/heading, reference
gyro axes, reset/switch persistence, and all three algorithms with upright-board
alignment and physical yaw motion. The filter quaternion stays in sensor axes;
only output quaternion/Euler angles and the explicit body gyro accessor change
coordinates. `python tests/run_gimbal_attitude_tests.py` also checks the vertical
mount against shared PID, CAN angle/quaternion, and VOFA outputs.

The VQF low-pass warmup uses explicit ready flags rather than NaN sentinels.
Regressions check finite histories across reset and variable-dt warmup, and
preserve the differential comparison to the official implementation. Body gyro
output tests cover static offsets and online bias compensation in all three
algorithms. The gimbal regression also simulates a stationary vertical sensor
with residual yaw-axis bias for 180 seconds and checks CAN yaw after convergence
and the compensated yaw-rate output. These are synthetic-data checks, not a
claim of hardware drift accuracy.

For a differential comparison, place the author's unmodified `vqf.cpp` and
`vqf.hpp` from `vqf/cpp` in a local reference directory, then run:

```powershell
python tests/imu/run_tests.py --reference-dir build/imu-reference
```

The reviewed upstream revision is
`86ba56bdd3158b9b05f9f9fe5596866ba326438c` from
[dlaidig/vqf](https://github.com/dlaidig/vqf/tree/86ba56bdd3158b9b05f9f9fe5596866ba326438c).
Keep its MIT license with downloaded copies. `HOST_CXX` can select a native C++
compiler. The comparison builds the upstream single-precision implementation
and checks identical input streams at 1 kHz and 200 Hz for five configurations:
full six-axis, basic (no bias estimation), rest only, motion only, and changed
time constants/noise. Quaternion distance is sign invariant; bias estimates and
rest flags are also compared. Each stream contains 30 seconds of rest, motion,
sensor noise, and residual bias.

These tests establish behavior for synthetic data. Hardware timing, mounting,
sensor synchronization, vibration, and real sensor noise need board validation.
