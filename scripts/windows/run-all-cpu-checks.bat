@echo off
setlocal EnableDelayedExpansion
cd /d "%~dp0"

set "RAW=https://raw.githubusercontent.com/mvl-boston/opendbc/cursor/windows-cpu-health-check-65b2/scripts/windows"

REM --- Request Administrator (needed for clocks + CPU temps) ---
net session >nul 2>&1
if errorlevel 1 (
  if /I not "%~1"=="elevated" (
    echo Requesting Administrator approval...
    powershell -NoProfile -Command "Start-Process -FilePath '%~f0' -Verb RunAs -ArgumentList 'elevated'"
    exit /b 0
  )
)

echo ================================================================
echo  One-click CPU check: download tools, speed test, temperature
echo  Folder: %CD%
echo  Expect about 2-3 minutes total.
echo ================================================================
echo.

where powershell >nul 2>&1
if errorlevel 1 (
  echo ERROR: powershell.exe not found.
  pause
  exit /b 1
)

echo [1/4] Updating downloader from GitHub...
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$ErrorActionPreference='Stop';" ^
  "Invoke-WebRequest -Uri '%RAW%/download-cpu-tools.ps1' -OutFile '%~dp0download-cpu-tools.ps1' -UseBasicParsing;" ^
  "$l=Get-Content -LiteralPath '%~dp0download-cpu-tools.ps1' -TotalCount 1;" ^
  "if ($l -notmatch '#Requires') { throw 'Bad download (HTML?). Open the raw GitHub link only.' }"
if errorlevel 1 goto :fail

echo [2/4] Downloading all scripts + LibreHardwareMonitor...
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0download-cpu-tools.ps1" -TargetDir "%CD%" -IncludeLibreHardwareMonitor
if errorlevel 1 goto :fail

echo.
echo [3/4] CPU speed / stress test (~45 seconds)...
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0cpu-health-check.ps1" -DurationSeconds 45
set "HERR=%ERRORLEVEL%"

echo.
echo ================================================================
echo [4/4] CPU temperature under load (~60 seconds)...
echo ================================================================
echo.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0cpu-temp-check.ps1" -MonitorSeconds 60 -StressCpu
set "TERR=%ERRORLEVEL%"

echo.
echo ================================================================
echo  ALL DONE
echo  Health check exit code: %HERR%   Temp check exit code: %TERR%
echo  To save a log next time:
echo    run-all-cpu-checks.bat ^> cpu-full-report.txt
echo ================================================================
pause
exit /b 0

:fail
echo.
echo FAILED. Check internet, then re-run this file.
pause
exit /b 1
