@echo off
setlocal EnableDelayedExpansion
cd /d "%~dp0"

set "LOG=%~dp0disk-full-report.txt"
set "RAW=https://raw.githubusercontent.com/mvl-boston/opendbc/cursor/windows-cpu-health-check-65b2/scripts/windows"

REM Optional first argument: drive letter (default = Windows drive, usually C)
set "DRV=%~1"
if /I "%DRV%"=="_run" set "DRV="
if "%DRV%"=="" set "DRV=%SystemDrive:~0,1%"

if /I not "%~1"=="_run" (
  echo ================================================================
  echo  Disk / SSD health + speed check - built-in Windows tools only
  echo  Drive: %DRV%:    Log: %LOG%
  echo  Runtime: about 3-6 minutes
  echo ================================================================
  echo.
  net session >nul 2>&1
  if errorlevel 1 (
    echo Requesting Administrator ^(needed for SMART data and winsat^)...
    powershell -NoProfile -ExecutionPolicy Bypass -Command ^
      "Start-Process -FilePath '%ComSpec%' -Verb RunAs -Wait -ArgumentList @('/c', 'cd /d \"%~dp0.\" && \"%~f0\" _run %DRV% > \"%~dp0disk-full-report.txt\" 2^>^&1');"
    if exist "%LOG%" type "%LOG%"
    echo.
    echo Saved: %LOG%
    pause
    exit /b 0
  )
  call "%~f0" _run %DRV% > "%LOG%" 2>&1
  set "ERR=!ERRORLEVEL!"
  type "%LOG%"
  echo.
  echo Saved: %LOG%
  pause
  exit /b %ERR%
)

set "DRV=%~2"
if "%DRV%"=="" set "DRV=%SystemDrive:~0,1%"

echo Started: %date% %time%
echo Mode: native Windows only (Get-PhysicalDisk, reliability counters, Event Log, winsat)
echo.

echo [1/2] Update scripts from GitHub...
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$ErrorActionPreference='Stop';" ^
  "Invoke-WebRequest -Uri '%RAW%/download-cpu-tools.ps1' -OutFile '%~dp0download-cpu-tools.ps1' -UseBasicParsing;" ^
  "$l=Get-Content -LiteralPath '%~dp0download-cpu-tools.ps1' -TotalCount 1;" ^
  "if ($l -notmatch '#Requires') { throw 'Bad download - use Raw GitHub link only.' }"
if errorlevel 1 goto :fail
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0download-cpu-tools.ps1" -TargetDir "%CD%" >nul
if errorlevel 1 goto :fail
echo     done

echo [2/2] Disk health + speed on %DRV%: ...
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0disk-health-check.ps1" -DriveLetter %DRV%
set "DERR=%ERRORLEVEL%"

echo.
echo ================================================================
echo  FINISHED: %date% %time%
echo  Key lines in this log:
echo    DISK_HEALTH: PASS ...   (SMART, errors, events, file system)
echo    DISK_SPEED:  PASS ...   (measured vs expected for drive type)
echo  Other drive:  run-disk-check.bat D
echo ================================================================
exit /b %DERR%

:fail
echo RUN FAILED - fix issue above and double-click this bat again.
exit /b 1
