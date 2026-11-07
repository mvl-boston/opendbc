@echo off
setlocal EnableDelayedExpansion
cd /d "%~dp0"

set "LOG=%~dp0rma-evidence.txt"
set "RAW=https://raw.githubusercontent.com/mvl-boston/opendbc/cursor/windows-cpu-health-check-65b2/scripts/windows"

REM Optional first argument: drive letter (default = Windows drive, usually C)
set "DRV=%~1"
if /I "%DRV%"=="_run" set "DRV="
if "%DRV%"=="" set "DRV=%SystemDrive:~0,1%"

if /I not "%~1"=="_run" (
  echo ================================================================
  echo  SSD warranty / return evidence file - built-in Windows tools only
  echo  Drive: %DRV%:    Output: %LOG%
  echo  Runtime: about 5-8 minutes. Leave the PC alone while it runs.
  echo ================================================================
  echo.
  net session >nul 2>&1
  if errorlevel 1 (
    echo Requesting Administrator ^(needed for SMART data and winsat^)...
    powershell -NoProfile -ExecutionPolicy Bypass -Command ^
      "Start-Process -FilePath '%ComSpec%' -Verb RunAs -Wait -ArgumentList @('/c', 'cd /d \"%~dp0.\" && \"%~f0\" _run %DRV% > \"%~dp0rma-evidence.txt\" 2^>^&1');"
    if exist "%LOG%" type "%LOG%"
    echo.
    echo Saved: %LOG%
    echo Submit this file plus your Dashboard SMART screenshots with the warranty claim.
    pause
    exit /b 0
  )
  call "%~f0" _run %DRV% > "%LOG%" 2>&1
  set "ERR=!ERRORLEVEL!"
  type "%LOG%"
  echo.
  echo Saved: %LOG%
  echo Submit this file plus your Dashboard SMART screenshots with the warranty claim.
  pause
  exit /b %ERR%
)

set "DRV=%~2"
if "%DRV%"=="" set "DRV=%SystemDrive:~0,1%"

REM Refresh scripts first (quietly) so the evidence always comes from the current version.
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$ErrorActionPreference='Stop';" ^
  "Invoke-WebRequest -Uri '%RAW%/download-cpu-tools.ps1' -OutFile '%~dp0download-cpu-tools.ps1' -UseBasicParsing;" ^
  "$l=Get-Content -LiteralPath '%~dp0download-cpu-tools.ps1' -TotalCount 1;" ^
  "if ($l -notmatch '#Requires') { throw 'Bad download - use Raw GitHub link only.' }" >nul 2>&1
if errorlevel 1 goto :fail
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0download-cpu-tools.ps1" -TargetDir "%CD%" >nul 2>&1
if errorlevel 1 goto :fail

powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0rma-evidence.ps1" -DriveLetter %DRV% -ScriptDir "%~dp0."
exit /b %ERRORLEVEL%

:fail
echo RUN FAILED - could not download the scripts from GitHub. Check the internet connection and run again.
exit /b 1
