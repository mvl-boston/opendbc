@echo off
setlocal EnableDelayedExpansion
cd /d "%~dp0"

set "RAW=https://raw.githubusercontent.com/mvl-boston/opendbc/cursor/windows-cpu-health-check-65b2/scripts/windows"
set "LOG=%~dp0cpu-full-report.txt"

REM --- Elevation: child runs with "inner" so output can be logged ---
if /I not "%~1"=="inner" (
  net session >nul 2>&1
  if errorlevel 1 (
    echo Requesting Administrator approval...
    echo Output will be saved to: %LOG%
    powershell -NoProfile -Command "Start-Process -FilePath '%~f0' -Verb RunAs -ArgumentList 'inner'"
    exit /b 0
  )
  call "%~f0" inner >> "%LOG%" 2>&1
  set "ERR=%ERRORLEVEL%"
  echo.
  echo ========== Full log saved to: %LOG% ==========
  type "%LOG%"
  echo.
  pause
  exit /b %ERR%
)

REM ========== inner (Administrator) ==========
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
echo Note: For accurate CPU die temps, run HWiNFO64 Sensors during step 4.
echo       See DEFENDER-WINRING0.txt in this folder.
echo.
echo [3/4] CPU speed / stress test (~45 seconds)...
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0cpu-health-check.ps1" -DurationSeconds 45
set "HERR=%ERRORLEVEL%"

echo.
echo ================================================================
echo [4/4] CPU temperature under load (~60 seconds, Windows zones)...
echo ================================================================
echo.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0cpu-temp-check-safe.ps1" -MonitorSeconds 60 -StressCpu
set "TERR=%ERRORLEVEL%"

echo.
echo ================================================================
echo  ALL DONE
echo  Health check exit code: %HERR%   Temp check exit code: %TERR%
echo.
echo  SPEED: Look for "MHz min/max/avg" near 3600 on i7-4790 and green OK on stability.
echo  TEMP:  "no readings" is common here; use HWiNFO64 for real CPU die temps.
echo  Log:   %LOG%
echo ================================================================
exit /b 0

:fail
echo.
echo FAILED. Check internet, then re-run this file.
exit /b 1
