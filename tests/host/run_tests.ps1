param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$compilerPath = (Get-Command $Compiler -ErrorAction Stop).Source

# The IMU runner reads HOST_CC; restore the caller's environment after the suite.
$previousCompiler = $env:HOST_CC
try {
    $env:HOST_CC = $compilerPath
    & python (Join-Path $projectRoot 'tests/architecture/test_dependencies.py')
    if ($LASTEXITCODE -ne 0) { throw 'Architecture regression failed' }
    & (Join-Path $PSScriptRoot 'run_contracts.ps1') -Compiler $compilerPath
    foreach ($suite in @('startup', 'vehicles', 'port', 'mag', 'imu', 'telemetry')) {
        $runner = Join-Path $projectRoot "tests/$suite/run_tests.py"
        if ($suite -in @('port', 'telemetry')) {
            & python $runner --cc $compilerPath
        } else {
            & python $runner
        }
        if ($LASTEXITCODE -ne 0) {
            throw "Host regression failed: $suite"
        }
    }
} finally {
    $env:HOST_CC = $previousCompiler
}
