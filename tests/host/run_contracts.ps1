param([string]$Compiler = "gcc")
$ErrorActionPreference = "Stop"
$root = (Resolve-Path "$PSScriptRoot/../..").Path
$binary = Join-Path $env:TEMP "bngu_contract_tests.exe"
$paths = @("Self_core/App/Contracts", "Self_core/App/Config", "Self_core/App/Modules/can", "Self_core/App/Modules/can/internal", "Self_core/App/Modules/control_support/internal", "Self_core/Drv/Drv_referee", "Self_core/Lib/Lib_Typedef", "Self_core/Lib/Lib_math", "Self_core")
$includes = $paths | ForEach-Object { "-I" + (Join-Path $root $_) }
$sources = @("tests/host/test_contracts.c", "Self_core/App/Modules/can/app_vehicle_can.c", "Self_core/App/Modules/can/app_core_protocol.c", "Self_core/App/Modules/can/app_chassis_comm.c", "Self_core/App/Modules/can/app_gimbal_comm.c", "Self_core/Drv/Drv_referee/drv_referee.c", "Self_core/Lib/Lib_math/lib_math.c") | ForEach-Object { Join-Path $root $_ }
& $Compiler -std=c11 -mno-ms-bitfields -Wall -DCURRENT_BOARD=BOARD_GIMBAL @includes @sources -lm -o $binary
if ($LASTEXITCODE -ne 0) { throw "Host compile failed" }
& $binary
if ($LASTEXITCODE -ne 0) { throw "Host regression failed" }
