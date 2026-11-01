@echo off
setlocal
cd /d "%~dp0"

echo CPU temperature check (LibreHardwareMonitor) - Win10
echo Run this window as Administrator for best results.
echo.

findstr /I /C:"#Requires" "%~dp0cpu-temp-check.ps1" >nul 2>&1
if errorlevel 1 (
  echo ERROR: cpu-temp-check.ps1 missing or invalid. Run download-cpu-temp-check.bat first.
  pause
  exit /b 1
)

set "SEC=%~1"
if "%SEC%"=="" set "SEC=30"
set "STRESS=%~2"

if /I "%STRESS%"=="stress" (
  powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0cpu-temp-check.ps1" -MonitorSeconds %SEC% -StressCpu
) else (
  powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0cpu-temp-check.ps1" -MonitorSeconds %SEC%
)

set "ERR=%ERRORLEVEL%"
echo.
if not "%ERR%"=="0" echo Exit code %ERR%.
pause
endlocal
exit /b %ERR%
