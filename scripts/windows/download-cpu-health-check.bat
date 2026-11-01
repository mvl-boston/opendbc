@echo off
setlocal
cd /d "%~dp0"
set "RAW=https://raw.githubusercontent.com/mvl-boston/opendbc/cursor/windows-cpu-health-check-65b2/scripts/windows"
if not exist "%~dp0download-cpu-tools.bat" (
  echo Fetching download-cpu-tools.bat...
  powershell -NoProfile -ExecutionPolicy Bypass -Command "Invoke-WebRequest -Uri '%RAW%/download-cpu-tools.bat' -OutFile '%~dp0download-cpu-tools.bat' -UseBasicParsing"
)
call "%~dp0download-cpu-tools.bat" %*
endlocal
