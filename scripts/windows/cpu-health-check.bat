@echo off
setlocal
cd /d "%~dp0"

echo CPU health check — launching PowerShell...
echo.

where powershell >nul 2>&1
if errorlevel 1 (
  echo ERROR: powershell.exe not found.
  exit /b 1
)

REM Optional: pass duration as first argument, e.g. cpu-health-check.bat 90
set "DUR=%~1"
if "%DUR%"=="" set "DUR=45"

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
