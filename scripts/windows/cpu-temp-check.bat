@echo off
setlocal
cd /d "%~dp0"

echo CPU temperature (HWiNFO64 - no WinRing0 / Defender block)
echo Run as Administrator. First HWiNFO run may need one manual accept of the license.
echo.

set "SEC=%~1"
if "%SEC%"=="" set "SEC=60"
set "STRESS=%~2"
if /I not "%STRESS%"=="stress" set "STRESS=stress"

powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0cpu-temp-check.ps1" -MonitorSeconds %SEC% -StressCpu
set "ERR=%ERRORLEVEL%"
echo.
pause
exit /b %ERR%
