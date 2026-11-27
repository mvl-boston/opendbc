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

set "RAW=https://raw.githubusercontent.com/mvl-boston/opendbc/cursor/windows-cpu-health-check-65b2/scripts/windows"

REM Always refresh the helper .ps1 first (fixes stale local copies)
echo Updating download-cpu-tools.ps1 from GitHub...
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$ErrorActionPreference='Stop';" ^
  "$u='%RAW%/download-cpu-tools.ps1';" ^
  "$o='%~dp0download-cpu-tools.ps1';" ^
  "Invoke-WebRequest -Uri $u -OutFile $o -UseBasicParsing;" ^
  "$l=Get-Content -LiteralPath $o -TotalCount 1;" ^
  "if ($l -notmatch '#Requires') { throw 'Downloaded ps1 is not valid (HTML or corrupt). Use the raw GitHub link.' }"
if errorlevel 1 (
  echo Failed to download download-cpu-tools.ps1. Check internet or firewall.
  pause
  exit /b 1
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
