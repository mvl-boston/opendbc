@echo off
setlocal
cd /d "%~dp0"
echo Opens HWiNFO download page. Extract portable ZIP to:
echo   %~dp0hwinfo\
echo so this folder contains hwinfo\HWiNFO64.exe
echo.
start https://www.hwinfo.com/download/
pause
