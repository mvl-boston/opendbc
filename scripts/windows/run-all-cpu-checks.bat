@echo off
setlocal EnableDelayedExpansion
cd /d "%~dp0"

set "LOG=%~dp0cpu-full-report.txt"
set "RAW=https://raw.githubusercontent.com/mvl-boston/opendbc/cursor/windows-cpu-health-check-65b2/scripts/windows"

REM Optional first argument: throttle test length in seconds (default 180)
set "THROTTLE_SEC=%~1"
if /I "%THROTTLE_SEC%"=="_run" set "THROTTLE_SEC="
if "%THROTTLE_SEC%"=="" set "THROTTLE_SEC=180"

REM ========== Double-click entry (requests Admin, writes log automatically) ==========
if /I not "%~1"=="_run" (
  echo ================================================================
  echo  CPU check - driver-free, no third-party tools
  echo  Log: %LOG%
  echo  Runtime: about %THROTTLE_SEC%s stress + 1 minute
  echo ================================================================
  echo.
  net session >nul 2>&1
  if errorlevel 1 (
    echo Requesting Administrator ^(needed for CPU frequency counters^)...
    powershell -NoProfile -ExecutionPolicy Bypass -Command ^
      "Start-Process -FilePath '%ComSpec%' -Verb RunAs -Wait -ArgumentList @('/c', 'cd /d \"%~dp0.\" && \"%~f0\" _run %THROTTLE_SEC% > \"%~dp0cpu-full-report.txt\" 2^>^&1');"
    if exist "%LOG%" type "%LOG%"
    echo.
    echo Saved: %LOG%
    pause
    exit /b 0
  )
  call "%~f0" _run %THROTTLE_SEC% > "%LOG%" 2>&1
  set "ERR=!ERRORLEVEL!"
  type "%LOG%"
  echo.
  echo Saved: %LOG%
  pause
  exit /b %ERR%
)

REM ========== _run (Administrator) ==========
set "THROTTLE_SEC=%~2"
if "%THROTTLE_SEC%"=="" set "THROTTLE_SEC=180"

echo Started: %date% %time%
echo Mode: native Windows only (no HWiNFO, no LibreHardwareMonitor, no drivers)
echo.

where powershell >nul 2>&1
if errorlevel 1 (
  echo FATAL: powershell.exe not found.
  exit /b 1
)

echo [1/4] Update scripts from GitHub...
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$ErrorActionPreference='Stop';" ^
  "Invoke-WebRequest -Uri '%RAW%/download-cpu-tools.ps1' -OutFile '%~dp0download-cpu-tools.ps1' -UseBasicParsing;" ^
  "$l=Get-Content -LiteralPath '%~dp0download-cpu-tools.ps1' -TotalCount 1;" ^
  "if ($l -notmatch '#Requires') { throw 'Bad download - use Raw GitHub link only.' }"
if errorlevel 1 goto :fail

echo [2/4] Download all tools...
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0download-cpu-tools.ps1" -TargetDir "%CD%"
if errorlevel 1 goto :fail

echo.
echo [3/4] CPU identity + 45s speed test...
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0cpu-health-check.ps1" -DurationSeconds 45
set "HERR=%ERRORLEVEL%"

echo.
echo [4/4] Driver-free thermal throttle test (%THROTTLE_SEC%s sustained load)...
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0cpu-throttle-test.ps1" -DurationSeconds %THROTTLE_SEC%
set "TERR=%ERRORLEVEL%"

echo.
echo ================================================================
echo  FINISHED: %date% %time%
echo  Key lines in this log:
echo    SPEED_TEST: PASS ...
echo    THROTTLE_EVENTS: NONE
echo    THROTTLE_TEST: PASS ...
echo  All three = CPU is running at full speed and not thermally throttling.
echo  Longer test: run-all-cpu-checks.bat 300
echo ================================================================
if %TERR% NEQ 0 exit /b %TERR%
if %HERR% NEQ 0 exit /b %HERR%
exit /b 0

:fail
echo RUN FAILED - fix issue above and double-click this bat again.
exit /b 1
