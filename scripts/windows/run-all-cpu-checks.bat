@echo off
setlocal EnableDelayedExpansion
cd /d "%~dp0"

set "LOG=%~dp0cpu-full-report.txt"
set "RAW=https://raw.githubusercontent.com/mvl-boston/opendbc/cursor/windows-cpu-health-check-65b2/scripts/windows"

REM ========== Double-click entry (requests Admin, writes log automatically) ==========
if /I not "%~1"=="_run" (
  echo ================================================================
  echo  CPU check - ONE file does everything
  echo  Log: %LOG%
  echo  Paste cpu-full-report.txt back until you see CPU_TEMP_TEST: PASS
  echo ================================================================
  echo.
  net session >nul 2>&1
  if errorlevel 1 (
    echo Requesting Administrator...
    powershell -NoProfile -ExecutionPolicy Bypass -Command ^
      "Start-Process -FilePath '%ComSpec%' -Verb RunAs -Wait -ArgumentList @('/c', 'cd /d \"%~dp0.\" && \"%~f0\" _run > \"%~dp0cpu-full-report.txt\" 2^>^&1');"
    if exist "%LOG%" type "%LOG%"
    echo.
    echo Saved: %LOG%
    pause
    exit /b 0
  )
  call "%~f0" _run > "%LOG%" 2>&1
  set "ERR=!ERRORLEVEL!"
  type "%LOG%"
  echo.
  echo Saved: %LOG%
  pause
  exit /b %ERR%
)

REM ========== _run (Administrator) ==========
echo Started: %date% %time%
echo.

where powershell >nul 2>&1
if errorlevel 1 (
  echo FATAL: powershell.exe not found.
  exit /b 1
)

echo [1/5] Update scripts from GitHub...
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$ErrorActionPreference='Stop';" ^
  "Invoke-WebRequest -Uri '%RAW%/download-cpu-tools.ps1' -OutFile '%~dp0download-cpu-tools.ps1' -UseBasicParsing;" ^
  "$l=Get-Content -LiteralPath '%~dp0download-cpu-tools.ps1' -TotalCount 1;" ^
  "if ($l -notmatch '#Requires') { throw 'Bad download - use Raw GitHub link only.' }"
if errorlevel 1 goto :fail

echo [2/5] Download all tools...
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0download-cpu-tools.ps1" -TargetDir "%CD%"
if errorlevel 1 goto :fail

echo [3/5] CPU speed / stress (~45s)...
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0cpu-health-check.ps1" -DurationSeconds 45
set "HERR=%ERRORLEVEL%"

echo.
echo [4/5] Ensure HWiNFO64 local copy ^(avoids referral error in Program Files^)...
call :ensure_hwinfo
if errorlevel 1 goto :fail

echo.
echo [5/5] CPU temperature under load (~60s)...
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0cpu-temp-check.ps1" -MonitorSeconds 60 -StressCpu -RequireHWiNFO
set "TERR=%ERRORLEVEL%"

echo.
echo ================================================================
echo  FINISHED: %date% %time%
echo  Look for these lines in this log:
echo    SPEED_TEST: PASS ...
echo    CPU_TEMP_TEST: PASS peak=...C
echo  If CPU_TEMP_TEST: FAIL - follow NEXT_STEP in the log, then run this bat again.
echo ================================================================
if %TERR% NEQ 0 exit /b %TERR%
if %HERR% NEQ 0 exit /b %HERR%
exit /b 0

:ensure_hwinfo
set "LOCAL=%LOCALAPPDATA%\CpuHealthCheck\hwinfo"
if not exist "%LOCAL%" mkdir "%LOCAL%"

if exist "%ProgramFiles%\HWiNFO64\HWiNFO64.exe" (
  echo Copying HWiNFO64 out of Program Files ^(fixes referral error^)...
  xcopy "%ProgramFiles%\HWiNFO64\*" "%LOCAL%\" /E /I /Y /Q >nul
)

if exist "%~dp0hwinfo\HWiNFO64.exe" (
  echo Copying portable hwinfo from script folder to local AppData...
  xcopy "%~dp0hwinfo\*" "%LOCAL%\" /E /I /Y /Q >nul
)

if exist "%LOCAL%\HWiNFO64.exe" goto :hwinfo_license

echo Trying winget install REALiX.HWiNFO ...
winget install -e --id REALiX.HWiNFO --accept-package-agreements --accept-source-agreements -h 2>nul
if exist "%ProgramFiles%\HWiNFO64\HWiNFO64.exe" (
  xcopy "%ProgramFiles%\HWiNFO64\*" "%LOCAL%\" /E /I /Y /Q >nul
)
if exist "%LOCAL%\HWiNFO64.exe" goto :hwinfo_license

echo.
echo *** ONE-TIME SETUP ***
echo If HWiNFO shows "A referral was returned from the server" - use PORTABLE zip here:
echo   %LOCAL%
echo Do NOT run HWiNFO from Program Files on this PC.
echo.
start https://www.hwinfo.com/download/
echo Press any key after HWiNFO64.exe is in that folder...
pause >nul
if exist "%LOCAL%\HWiNFO64.exe" goto :hwinfo_license
echo HWiNFO64 still not found in %LOCAL%
exit /b 1

:hwinfo_license
set "HWI=%LOCAL%\HWiNFO64.exe"
if not exist "%TEMP%\cpu-check-hwinfo-license.ok" (
  echo First run: accept HWiNFO license if prompted ^(local copy^)...
  start "" /wait "%HWI%" /SKIPSPLASH /SENSORSONLY
  powershell -NoProfile -Command "Set-Content -LiteralPath $env:TEMP\cpu-check-hwinfo-license.ok -Value ok -Force" 2>nul
)
echo HWiNFO64 ready: %HWI%
exit /b 0

:fail
echo RUN FAILED - fix issue above and double-click this bat again.
exit /b 1
