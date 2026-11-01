@echo off
setlocal
cd /d "%~dp0"

set "BASE=https://raw.githubusercontent.com/mvl-boston/opendbc/cursor/windows-cpu-health-check-65b2/scripts/windows"

echo Downloading CPU health check scripts from GitHub RAW (not the HTML web page)...
echo Target folder: %CD%
echo.

powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$ErrorActionPreference='Stop';" ^
  "$base='%BASE%';" ^
  "Invoke-WebRequest -Uri ($base + '/cpu-health-check.ps1') -OutFile 'cpu-health-check.ps1' -UseBasicParsing;" ^
  "Invoke-WebRequest -Uri ($base + '/cpu-health-check.bat') -OutFile 'cpu-health-check.bat' -UseBasicParsing;" ^
  "$head = Get-Content -LiteralPath 'cpu-health-check.ps1' -TotalCount 5 -Raw;" ^
  "if ($head -match '<!DOCTYPE|<html|githubassets') { throw 'Download still looks like HTML. Check proxy or paste the one-liner from the README.' };" ^
  "Write-Host 'OK: cpu-health-check.ps1 starts with:';" ^
  "Get-Content -LiteralPath 'cpu-health-check.ps1' -TotalCount 2"

if errorlevel 1 (
  echo.
  echo DOWNLOAD FAILED.
  exit /b 1
)

echo.
echo Done. Run cpu-health-check.bat as Administrator.
pause
endlocal
