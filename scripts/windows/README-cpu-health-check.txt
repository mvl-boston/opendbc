Windows CPU health check - download help
======================================

If PowerShell shows errors about "<", "html", or "githubassets", your
cpu-health-check.ps1 file is an HTML web page, not the script.

DO NOT use "Save As" on the normal GitHub file view.

Option A - one PowerShell command (recommended)
-----------------------------------------------
Open PowerShell, cd to your folder, then paste:

  cd $env:USERPROFILE\OneDrive\Desktop\TempMDX
  Invoke-WebRequest -Uri "https://raw.githubusercontent.com/mvl-boston/opendbc/cursor/windows-cpu-health-check-65b2/scripts/windows/cpu-health-check.ps1" -OutFile cpu-health-check.ps1 -UseBasicParsing
  Invoke-WebRequest -Uri "https://raw.githubusercontent.com/mvl-boston/opendbc/cursor/windows-cpu-health-check-65b2/scripts/windows/cpu-health-check.bat" -OutFile cpu-health-check.bat -UseBasicParsing
  Get-Content .\cpu-health-check.ps1 -TotalCount 2

The first lines must look like:
  #Requires -Version 5.1

Option B - download helper batch file
-------------------------------------
Save this ONE file, double-click it (downloads everything):

  https://raw.githubusercontent.com/mvl-boston/opendbc/cursor/windows-cpu-health-check-65b2/scripts/windows/download-cpu-tools.bat

Optional (includes LibreHardwareMonitor ~2 MB up front):

  download-cpu-tools.bat all

Legacy names download-cpu-health-check.bat and download-cpu-temp-check.bat
call the same downloader.

Option C - GitHub Raw button
----------------------------
On the file page on GitHub, click the "Raw" button, then Save As.

Run as Administrator: right-click cpu-health-check.bat -> Run as administrator

Reliable CPU temperatures (recommended)
---------------------------------------
Use cpu-temp-check.ps1 (LibreHardwareMonitor, auto-download on first run):

  download-cpu-temp-check.bat
  Right-click cpu-temp-check.bat -> Run as administrator

  60 seconds under full CPU load:
  cpu-temp-check.bat 60 stress

Raw files:
  https://raw.githubusercontent.com/mvl-boston/opendbc/cursor/windows-cpu-health-check-65b2/scripts/windows/cpu-temp-check.ps1
