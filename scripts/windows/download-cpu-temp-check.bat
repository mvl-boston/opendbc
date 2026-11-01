@echo off
setlocal
cd /d "%~dp0"

set "BASE=https://raw.githubusercontent.com/mvl-boston/opendbc/cursor/windows-cpu-health-check-65b2/scripts/windows"

echo Downloading CPU temp check scripts (raw GitHub)...
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$ErrorActionPreference='Stop'; $b='%BASE%';" ^
  "Invoke-WebRequest ($b + '/cpu-temp-check.ps1') -OutFile 'cpu-temp-check.ps1' -UseBasicParsing;" ^
  "Invoke-WebRequest ($b + '/cpu-temp-check.bat') -OutFile 'cpu-temp-check.bat' -UseBasicParsing;" ^
  "Write-Host 'OK. First run will download LibreHardwareMonitor (~2 MB) automatically.'"

if errorlevel 1 (
  echo Download failed.
  pause
  exit /b 1
)

echo.
echo Next: Right-click cpu-temp-check.bat -^> Run as administrator
echo   Normal:  cpu-temp-check.bat
echo   60s load: cpu-temp-check.bat 60 stress
pause
endlocal
