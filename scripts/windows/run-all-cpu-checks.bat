@echo off
setlocal EnableDelayedExpansion
cd /d "%~dp0"

set "LOG=%~dp0cpu-full-report.txt"

REM Hidden mode: all output goes to stdout; caller redirects to LOG
if /I "%~1"=="_run" goto :main

echo Report file:
echo   %LOG%
echo.

net session >nul 2>&1
if errorlevel 1 (
  echo Requesting Administrator ^(required for CPU counters^)...
  powershell -NoProfile -ExecutionPolicy Bypass -Command ^
    "Start-Process -FilePath '%ComSpec%' -Verb RunAs -Wait -ArgumentList @('/c', 'cd /d \"%~dp0.\" && \"%~f0\" _run > \"%~dp0cpu-full-report.txt\" 2>&1');"
  if exist "%LOG%" (
    echo.
    echo ========== %LOG% ==========
    type "%LOG%"
  ) else (
    echo WARN: Log file was not created. Did you approve Administrator?
  )
  echo.
  pause
  exit /b 0
)

call "%~f0" _run > "%LOG%" 2>&1
set "ERR=!ERRORLEVEL!"
echo.
echo ========== %LOG% ==========
type "%LOG%"
echo.
echo Saved: %LOG%
pause
exit /b %ERR%

:main
set "RAW=https://raw.githubusercontent.com/mvl-boston/opendbc/cursor/windows-cpu-health-check-65b2/scripts/windows"

echo ================================================================
echo  One-click CPU check: download tools, speed test, temperature
echo  Folder: %CD%
echo  Log file: %LOG%
echo  Started: %date% %time%
echo  Expect about 2-3 minutes total.
echo ================================================================
echo.

where powershell >nul 2>&1
if errorlevel 1 (
  echo ERROR: powershell.exe not found.
  exit /b 1
)

echo [1/4] Updating downloader from GitHub...
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$ErrorActionPreference='Stop';" ^
  "Invoke-WebRequest -Uri '%RAW%/download-cpu-tools.ps1' -OutFile '%~dp0download-cpu-tools.ps1' -UseBasicParsing;" ^
  "$l=Get-Content -LiteralPath '%~dp0download-cpu-tools.ps1' -TotalCount 1;" ^
  "if ($l -notmatch '#Requires') { throw 'Bad download (HTML?). Open the raw GitHub link only.' }"
if errorlevel 1 goto :fail

echo [2/4] Downloading scripts (no LibreHardwareMonitor - avoids Defender WinRing0 block)...
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0download-cpu-tools.ps1" -TargetDir "%CD%"
if errorlevel 1 goto :fail

echo.
echo Note: Step 4 uses HWiNFO64 (Defender-safe). See HARDWARE-MONITORING.txt
echo.
echo [3/4] CPU speed / stress test (~45 seconds)...
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0cpu-health-check.ps1" -DurationSeconds 45
set "HERR=%ERRORLEVEL%"

echo.
echo ================================================================
echo [4/4] CPU temperature under load (~60 seconds, HWiNFO64)...
echo       If first run: may need install-hwinfo.bat or auto-download.
echo ================================================================
echo.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0cpu-temp-check.ps1" -MonitorSeconds 60 -StressCpu
set "TERR=%ERRORLEVEL%"

echo.
echo ================================================================
echo  ALL DONE
echo  Finished: %date% %time%
echo  Health check exit code: %HERR%   Temp check exit code: %TERR%
echo.
echo  SPEED: i7-4790 should show MHz min/max/avg near ~3600 under stress.
echo         Green OK on stability = good. WARN on MHz drop = check cooling.
echo  TEMP:  HWiNFO64 CPU Package/Core; peak under ~100 C is OK on i7-4790.
echo         If HWiNFO missing, run install-hwinfo.bat once.
echo ================================================================
exit /b 0

:fail
echo.
echo FAILED. Check internet, then re-run this file.
exit /b 1
