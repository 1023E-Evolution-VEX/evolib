param([string]$Compiler = 'C:/Program Files/LLVM/bin/clang++.exe')
$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    New-Item -ItemType Directory -Force bin | Out-Null
    & $Compiler -std=c++20 -Wall -Wextra -Werror -D_CRT_SECURE_NO_WARNINGS -Itelemetry/tests/mock -Iinclude telemetry/tests/check.cpp src/evolib/telemetry.cpp -o bin/telemetry-check.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host compilation failed' }
    & ./bin/telemetry-check.exe
    if ($LASTEXITCODE -ne 0) { throw 'Telemetry checks failed' }
} finally { Pop-Location }
