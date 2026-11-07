@echo off
setlocal EnableDelayedExpansion
cd /d "%~dp0"

set "LOG=%~dp0cleanup-report.txt"

if /I "%~1"=="_admin" goto :work

echo ================================================================
echo  Remove everything the CPU check scripts installed or downloaded
echo ================================================================
echo.
echo This will:
echo   - Force-remove HWiNFO64 (winget / Inno uninstaller / manual) and VERIFY
echo   - Uninstall Intel Power Gadget if present
echo   - Remove LibreHardwareMonitor folder and any WinRing0 driver service
echo   - Delete hwinfo\, hwinfo-logs\, lhm\, %%LOCALAPPDATA%%\CpuHealthCheck
echo   - Delete downloaded cpu-*/disk-* helper scripts in this folder
echo.
echo Kept: this file, cpu-full-report.txt, disk-full-report.txt, cleanup-report.txt
echo.
set /p CONFIRM=Type YES to continue: 
if /I not "%CONFIRM%"=="YES" (
  echo Cancelled.
  pause
  exit /b 0
)

net session >nul 2>&1
if errorlevel 1 (
  echo Requesting Administrator...
  powershell -NoProfile -ExecutionPolicy Bypass -Command ^
    "Start-Process -FilePath '%ComSpec%' -Verb RunAs -Wait -ArgumentList @('/c', 'cd /d \"%~dp0.\" && \"%~f0\" _admin > \"%LOG%\" 2^>^&1');"
) else (
  call "%~f0" _admin > "%LOG%" 2>&1
)
if exist "%LOG%" type "%LOG%"
echo.
echo Saved: %LOG%
pause
exit /b 0

:work
echo Started: %date% %time%
echo.

echo ===== [1] HWiNFO64 =====
taskkill /IM HWiNFO64.exe /F >nul 2>&1
taskkill /IM HWiNFO32.exe /F >nul 2>&1

echo  1a. winget uninstall...
winget uninstall -e --id REALiX.HWiNFO --silent --disable-interactivity >nul 2>&1
if not exist "%ProgramFiles%\HWiNFO64\HWiNFO64.exe" echo      winget: removed

echo  1b. Inno uninstaller (copied out of Program Files to dodge the "referral" error)...
set "UNINS="
if exist "%ProgramFiles%\HWiNFO64\unins000.exe" set "UNINS=%ProgramFiles%\HWiNFO64"
if exist "%ProgramFiles(x86)%\HWiNFO64\unins000.exe" set "UNINS=%ProgramFiles(x86)%\HWiNFO64"
if defined UNINS (
  if exist "%TEMP%\hwinfo-unins" rmdir /S /Q "%TEMP%\hwinfo-unins"
  mkdir "%TEMP%\hwinfo-unins" >nul 2>&1
  copy /Y "!UNINS!\unins000.*" "%TEMP%\hwinfo-unins\" >nul 2>&1
  "%TEMP%\hwinfo-unins\unins000.exe" /VERYSILENT /SUPPRESSMSGBOXES /NORESTART >nul 2>&1
  if errorlevel 1 (
    echo      copied uninstaller failed ^(code !ERRORLEVEL!^), trying in place...
    "!UNINS!\unins000.exe" /VERYSILENT /SUPPRESSMSGBOXES /NORESTART >nul 2>&1
  )
  rmdir /S /Q "%TEMP%\hwinfo-unins" >nul 2>&1
) else (
  echo      no unins000.exe found ^(already gone or never installed^)
)

echo  1c. Manual removal of leftovers...
for %%D in ("%ProgramFiles%\HWiNFO64" "%ProgramFiles(x86)%\HWiNFO64" "%ProgramFiles%\HWiNFO32" "%LOCALAPPDATA%\CpuHealthCheck") do (
  if exist %%D (
    takeown /F %%D /R /D Y >nul 2>&1
    icacls %%D /grant Administrators:F /T >nul 2>&1
    rmdir /S /Q %%D >nul 2>&1
  )
)
for %%K in (
  "HKLM\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\HWiNFO64_is1"
  "HKLM\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\HWiNFO64_is1"
  "HKCU\Software\HWiNFO64"
  "HKLM\SOFTWARE\HWiNFO64"
) do reg delete %%K /f >nul 2>&1
reg delete "HKCU\Software\Microsoft\Windows\CurrentVersion\Run" /v HWiNFO64 /f >nul 2>&1
reg delete "HKLM\SOFTWARE\Microsoft\Windows\CurrentVersion\Run" /v HWiNFO64 /f >nul 2>&1
schtasks /Delete /TN "HWiNFO64" /F >nul 2>&1
for /f "tokens=2 delims=: " %%S in ('sc query type^= driver state^= all ^| findstr /I /R "SERVICE_NAME:.*HWiNFO" ') do (
  echo      removing driver service %%S
  sc stop %%S >nul 2>&1
  sc delete %%S >nul 2>&1
)
del /F /Q "%windir%\System32\drivers\HWiNFO*.sys" >nul 2>&1
del /F /Q "%windir%\HWiNFO*.sys" >nul 2>&1
del /F /Q "%TEMP%\HWiNFO*.sys" >nul 2>&1
rmdir /S /Q "%ProgramData%\Microsoft\Windows\Start Menu\Programs\HWiNFO64" >nul 2>&1
rmdir /S /Q "%APPDATA%\Microsoft\Windows\Start Menu\Programs\HWiNFO64" >nul 2>&1
del /F /Q "%PUBLIC%\Desktop\HWiNFO64.lnk" "%USERPROFILE%\Desktop\HWiNFO64.lnk" "%USERPROFILE%\OneDrive\Desktop\HWiNFO64.lnk" >nul 2>&1
del /F /Q "%TEMP%\cpu-check-hwinfo-license.ok" "%TEMP%\hw64.zip" >nul 2>&1
if exist "%~dp0hwinfo" rmdir /S /Q "%~dp0hwinfo"
if exist "%~dp0hwinfo-logs" rmdir /S /Q "%~dp0hwinfo-logs"
echo      done

echo ===== [2] Intel Power Gadget =====
winget uninstall -e --id Intel.PowerGadget --silent --disable-interactivity >nul 2>&1
powershell -NoProfile -Command "Get-CimInstance Win32_Product -Filter \"Name like 'Intel%%Power Gadget%%'\" -ErrorAction SilentlyContinue | ForEach-Object { $_ | Invoke-CimMethod -MethodName Uninstall | Out-Null }" >nul 2>&1
echo      done

echo ===== [3] LibreHardwareMonitor / WinRing0 =====
taskkill /IM LibreHardwareMonitor.exe /F >nul 2>&1
for /f "tokens=2 delims=: " %%S in ('sc query type^= driver state^= all ^| findstr /I /R "SERVICE_NAME:.*R0.*" ') do (
  echo      removing driver service %%S
  sc stop %%S >nul 2>&1
  sc delete %%S >nul 2>&1
)
for /f "tokens=2 delims=: " %%S in ('sc query type^= driver state^= all ^| findstr /I /R "SERVICE_NAME:.*WinRing0.*" ') do (
  echo      removing driver service %%S
  sc stop %%S >nul 2>&1
  sc delete %%S >nul 2>&1
)
if exist "%~dp0lhm" rmdir /S /Q "%~dp0lhm"
del /F /Q "%windir%\System32\drivers\WinRing0*.sys" "%TEMP%\LibreHardwareMonitor-net472.zip" >nul 2>&1
for /d %%D in ("%TEMP%\cpu-throttle-*") do rmdir /S /Q "%%D" >nul 2>&1
echo      done

echo ===== [4] Downloaded helper scripts =====
for %%F in (
  cpu-health-check.ps1 cpu-health-check.bat cpu-throttle-test.ps1
  cpu-temp-check.ps1 cpu-temp-check.bat cpu-temp-check-safe.ps1 cpu-temp-check-safe.bat
  disk-health-check.ps1 run-disk-check.bat rma-evidence.ps1 make-rma-evidence.bat
  install-hwinfo.bat download-cpu-tools.ps1 download-cpu-tools.bat download-cpu-tools-all.bat
  download-cpu-health-check.bat download-cpu-temp-check.bat run-all-cpu-checks.bat
  HARDWARE-MONITORING.txt DEFENDER-WINRING0.txt README-cpu-health-check.txt .hwinfo-license-ok
) do if exist "%~dp0%%F" del /F /Q "%~dp0%%F"
echo      done

echo.
echo ===== [5] VERIFY HWiNFO fully removed =====
set "LEFT=0"
for %%D in (
  "%ProgramFiles%\HWiNFO64"
  "%ProgramFiles(x86)%\HWiNFO64"
  "%LOCALAPPDATA%\CpuHealthCheck"
  "%ProgramData%\Microsoft\Windows\Start Menu\Programs\HWiNFO64"
  "%APPDATA%\Microsoft\Windows\Start Menu\Programs\HWiNFO64"
) do (
  if exist "%%~D\" (echo  FOUND   folder %%~D & set "LEFT=1") else (echo  clean   %%~D)
)
for %%F in (
  "%windir%\System32\drivers\HWiNFO64A.SYS"
  "%windir%\HWiNFO64A.SYS"
  "%ProgramFiles%\HWiNFO64\HWiNFO64.exe"
) do (
  if exist "%%~F" (echo  FOUND   file %%~F & set "LEFT=1") else (echo  clean   %%~F)
)
for %%K in (
  "HKLM\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\HWiNFO64_is1"
  "HKLM\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\HWiNFO64_is1"
  "HKCU\Software\HWiNFO64"
  "HKLM\SOFTWARE\HWiNFO64"
) do (
  reg query "%%~K" >nul 2>&1 && (echo  FOUND   registry %%~K & set "LEFT=1") || echo  clean   %%~K
)
sc query type= driver state= all 2>nul | findstr /I "HWiNFO" >nul && (echo  FOUND   driver service containing HWiNFO & set "LEFT=1") || echo  clean   no HWiNFO driver service
tasklist /FI "IMAGENAME eq HWiNFO64.exe" 2>nul | findstr /I "HWiNFO64.exe" >nul && (echo  FOUND   HWiNFO64.exe still running & set "LEFT=1") || echo  clean   no HWiNFO process
winget list -e --id REALiX.HWiNFO --disable-interactivity 2>nul | findstr /I "HWiNFO" >nul && (echo  FOUND   winget still lists REALiX.HWiNFO & set "LEFT=1") || echo  clean   winget does not list HWiNFO
powershell -NoProfile -Command "$k=Get-ChildItem 'HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall','HKLM:\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall','HKCU:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall' -ErrorAction SilentlyContinue | Get-ItemProperty -ErrorAction SilentlyContinue | Where-Object { $_.DisplayName -like '*HWiNFO*' }; if ($k) { 'FOUND' } else { 'clean' }" 2>nul | findstr /I "FOUND" >nul && (echo  FOUND   HWiNFO still in Apps ^& Features list & set "LEFT=1") || echo  clean   not in Apps ^& Features

echo.
if "%LEFT%"=="0" (
  echo HWINFO_REMOVAL: COMPLETE
) else (
  echo HWINFO_REMOVAL: INCOMPLETE - items marked FOUND above remain. Reboot and run this bat again;
  echo   a locked driver file or in-use folder usually clears after reboot.
)

echo.
echo Remaining in this folder:
dir /B "%~dp0"
echo.
echo Finished: %date% %time%
echo Windows Security ^> Protection history still shows the earlier WinRing0 block; that is history only.
exit /b 0
