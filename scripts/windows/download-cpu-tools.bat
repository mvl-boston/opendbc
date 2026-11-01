@echo off
setlocal
cd /d "%~dp0"

echo ============================================================
echo  CPU tools downloader (health check + reliable temp check)
echo  Uses GitHub RAW files - not the HTML web page.
echo ============================================================
echo.
echo Target folder:
echo   %CD%
echo.

where powershell >nul 2>&1
if errorlevel 1 (
  echo ERROR: powershell.exe not found.
  pause
  exit /b 1
)

REM Optional: download-cpu-tools.bat all   ^(includes LibreHardwareMonitor ~2MB^)
set "MODE=%~1"
set "LHM="
if /I "%MODE%"=="all" set "LHM=-IncludeLibreHardwareMonitor"

REM If .ps1 helper missing, bootstrap it from GitHub first
if not exist "%~dp0download-cpu-tools.ps1" (
  echo Fetching downloader helper...
  powershell -NoProfile -ExecutionPolicy Bypass -Command ^
    "$u='https://raw.githubusercontent.com/mvl-boston/opendbc/cursor/windows-cpu-health-check-65b2/scripts/windows/download-cpu-tools.ps1';" ^
    "Invoke-WebRequest -Uri $u -OutFile '%~dp0download-cpu-tools.ps1' -UseBasicParsing"
  if errorlevel 1 (
    echo Bootstrap download failed. Check internet or firewall.
    pause
    exit /b 1
  )
)

powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0download-cpu-tools.ps1" -TargetDir "%CD%" %LHM%
set "ERR=%ERRORLEVEL%"

echo.
if not "%ERR%"=="0" (
  echo Download failed with code %ERR%.
) else (
  echo Tip: save THIS file somewhere permanent, e.g. Desktop\cpu-test\
  echo      then run it again anytime to refresh scripts.
)
pause
endlocal
exit /b %ERR%
