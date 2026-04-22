# run.ps1 — activates the venv and delegates to run.py
# Usage: .\run.ps1 <command> [args]
# Examples:
#   .\run.ps1 ar   .\session_1234\session_1234  .\scenes\bottle
#   .\run.ps1 view .\scenes\bottle
#   .\run.ps1 sparse .\scenes\panda --sequential

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$activate  = Join-Path $scriptDir "venv\Scripts\Activate.ps1"

if (Test-Path $activate) {
    & $activate
} else {
    Write-Warning "venv not found at $activate - using system Python"
}

python (Join-Path $scriptDir "run.py") @args
