@echo off
setlocal
cd /d "%~dp0"

echo CPU health check - launching Windows PowerShell...
echo.

where powershell >nul 2>&1
if errorlevel 1 (
  echo ERROR: powershell.exe not found.
  exit /b 1
)

REM Optional: pass duration as first argument, e.g. cpu-health-check.bat 90
set "DUR=%~1"
if "%DUR%"=="" set "DUR=45"

if not exist "%~dp0cpu-health-check.ps1" (
  echo ERROR: cpu-health-check.ps1 not found in %~dp0
  echo Run download-cpu-health-check.bat first.
  pause
  exit /b 1
)

findstr /I /C:"#Requires" "%~dp0cpu-health-check.ps1" >nul 2>&1
if errorlevel 1 (
  echo.
  echo ERROR: cpu-health-check.ps1 is not a valid PowerShell script.
  echo You likely saved the GitHub WEB PAGE ^(HTML^) instead of the raw script.
  echo.
  echo Fix: run download-cpu-health-check.bat in this folder
  echo   OR paste this in PowerShell:
  echo   iwr https://raw.githubusercontent.com/mvl-boston/opendbc/cursor/windows-cpu-health-check-65b2/scripts/windows/cpu-health-check.ps1 -OutFile cpu-health-check.ps1
  echo.
  pause
  exit /b 1
)

powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0cpu-health-check.ps1" -DurationSeconds %DUR%
set "ERR=%ERRORLEVEL%"

echo.
if not "%ERR%"=="0" (
  echo Script exited with code %ERR%.
) else (
  echo Finished. See output above or redirect to a file:
  echo   cpu-health-check.bat 60 ^> cpu-report.txt
)

pause
endlocal
exit /b %ERR%
