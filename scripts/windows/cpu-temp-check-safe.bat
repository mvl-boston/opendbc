@echo off
setlocal
cd /d "%~dp0"

echo CPU temperature (Windows built-in sensors - no WinRing0 driver)
echo Defender-safe. For die temps use HWiNFO64 - see DEFENDER-WINRING0.txt
echo.

set "SEC=%~1"
if "%SEC%"=="" set "SEC=30"
set "STRESS=%~2"

if /I "%STRESS%"=="stress" (
  powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0cpu-temp-check-safe.ps1" -MonitorSeconds %SEC% -StressCpu
) else (
  powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0cpu-temp-check-safe.ps1" -MonitorSeconds %SEC%
)

set "ERR=%ERRORLEVEL%"
echo.
pause
exit /b %ERR%
