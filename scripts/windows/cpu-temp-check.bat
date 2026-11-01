@echo off
setlocal
cd /d "%~dp0"

echo CPU temperature check
echo   Default: safe mode (no WinRing0) - cpu-temp-check-safe.ps1
echo   Optional: cpu-temp-check.bat lhm 60 stress  ^(LibreHardwareMonitor - may be blocked by Defender^)
echo.

if /I "%~1"=="lhm" (
  shift
  set "SEC=%~1"
  if "%SEC%"=="" set "SEC=30"
  set "STRESS=%~2"
  if /I "%STRESS%"=="stress" (
    powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0cpu-temp-check.ps1" -MonitorSeconds %SEC% -StressCpu
  ) else (
    powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0cpu-temp-check.ps1" -MonitorSeconds %SEC%
  )
  goto :done
)

call "%~dp0cpu-temp-check-safe.bat" %*

:done
set "ERR=%ERRORLEVEL%"
exit /b %ERR%
