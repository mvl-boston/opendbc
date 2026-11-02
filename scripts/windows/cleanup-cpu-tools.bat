@echo off
setlocal EnableDelayedExpansion
cd /d "%~dp0"

echo ================================================================
echo  Remove everything the CPU check scripts installed or downloaded
echo ================================================================
echo.
echo This will:
echo   - Uninstall HWiNFO64 (installed by winget)
echo   - Uninstall Intel Power Gadget if present
echo   - Delete LibreHardwareMonitor folder (lhm) and any WinRing0 driver service
echo   - Delete hwinfo\, hwinfo-logs\, %%LOCALAPPDATA%%\CpuHealthCheck
echo   - Delete the downloaded cpu-*.ps1 / *.bat helper files in this folder
echo.
echo It will NOT delete this cleanup file or cpu-full-report.txt.
echo.
set /p CONFIRM=Type YES to continue: 
if /I not "%CONFIRM%"=="YES" (
  echo Cancelled.
  pause
  exit /b 0
)

net session >nul 2>&1
if errorlevel 1 (
  echo.
  echo Administrator is required for uninstall. Requesting...
  powershell -NoProfile -ExecutionPolicy Bypass -Command "Start-Process -FilePath '%ComSpec%' -Verb RunAs -Wait -ArgumentList @('/c', 'cd /d \"%~dp0.\" && \"%~f0\" _admin')"
  echo.
  echo Done. Review output in the Administrator window.
  pause
  exit /b 0
)
if /I "%~1"=="" goto :work
if /I "%~1"=="_admin" goto :work
goto :work

:work
echo.
echo [1] HWiNFO64 ...
winget uninstall -e --id REALiX.HWiNFO --silent 2>nul
if exist "%ProgramFiles%\HWiNFO64\unins000.exe" "%ProgramFiles%\HWiNFO64\unins000.exe" /SILENT
taskkill /IM HWiNFO64.exe /F >nul 2>&1
if exist "%ProgramFiles%\HWiNFO64" rmdir /S /Q "%ProgramFiles%\HWiNFO64" 2>nul
reg delete "HKCU\Software\HWiNFO64" /f >nul 2>&1
sc stop HWiNFO_xxx >nul 2>&1
echo     done

echo [2] Intel Power Gadget ...
winget uninstall -e --id Intel.PowerGadget --silent 2>nul
powershell -NoProfile -Command "Get-CimInstance Win32_Product -Filter \"Name like 'Intel%%Power Gadget%%'\" -ErrorAction SilentlyContinue | ForEach-Object { $_ | Invoke-CimMethod -MethodName Uninstall | Out-Null }" 2>nul
echo     done

echo [3] LibreHardwareMonitor / WinRing0 driver service ...
taskkill /IM LibreHardwareMonitor.exe /F >nul 2>&1
for /f "tokens=2 delims=: " %%S in ('sc query type^= driver state^= all ^| findstr /I /R "SERVICE_NAME:.*R0.*" ') do (
  echo     removing driver service %%S
  sc stop %%S >nul 2>&1
  sc delete %%S >nul 2>&1
)
for /f "tokens=2 delims=: " %%S in ('sc query type^= driver state^= all ^| findstr /I /R "SERVICE_NAME:.*WinRing0.*" ') do (
  echo     removing driver service %%S
  sc stop %%S >nul 2>&1
  sc delete %%S >nul 2>&1
)
if exist "%~dp0lhm" rmdir /S /Q "%~dp0lhm"
if exist "%windir%\System32\drivers\WinRing0x64.sys" del /F /Q "%windir%\System32\drivers\WinRing0x64.sys" 2>nul
echo     done

echo [4] Local tool folders ...
if exist "%LOCALAPPDATA%\CpuHealthCheck" rmdir /S /Q "%LOCALAPPDATA%\CpuHealthCheck"
if exist "%~dp0hwinfo" rmdir /S /Q "%~dp0hwinfo"
if exist "%~dp0hwinfo-logs" rmdir /S /Q "%~dp0hwinfo-logs"
del /F /Q "%TEMP%\cpu-check-hwinfo-license.ok" 2>nul
del /F /Q "%TEMP%\hw64.zip" 2>nul
del /F /Q "%TEMP%\LibreHardwareMonitor-net472.zip" 2>nul
for /d %%D in ("%TEMP%\cpu-throttle-*") do rmdir /S /Q "%%D" 2>nul
echo     done

echo [5] Downloaded helper scripts in this folder ...
for %%F in (
  cpu-health-check.ps1 cpu-health-check.bat
  cpu-temp-check.ps1 cpu-temp-check.bat
  cpu-temp-check-safe.ps1 cpu-temp-check-safe.bat
  cpu-throttle-test.ps1
  install-hwinfo.bat
  download-cpu-tools.ps1 download-cpu-tools.bat download-cpu-tools-all.bat
  download-cpu-health-check.bat download-cpu-temp-check.bat
  run-all-cpu-checks.bat
  HARDWARE-MONITORING.txt DEFENDER-WINRING0.txt README-cpu-health-check.txt
  .hwinfo-license-ok
) do (
  if exist "%~dp0%%F" del /F /Q "%~dp0%%F"
)
echo     done

echo.
echo Remaining in this folder:
dir /B "%~dp0"
echo.
echo Cleanup complete. You may delete cleanup-cpu-tools.bat and cpu-full-report.txt manually.
echo Windows Security ^> Protection history still lists the earlier WinRing0 block; that is just history.
pause
exit /b 0
