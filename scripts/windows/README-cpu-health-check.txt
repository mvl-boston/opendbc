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
Open this RAW link in the browser, Save As download-cpu-health-check.bat:

  https://raw.githubusercontent.com/mvl-boston/opendbc/cursor/windows-cpu-health-check-65b2/scripts/windows/download-cpu-health-check.bat

Double-click it. It fetches the .ps1 and .bat for you.

Option C - GitHub Raw button
----------------------------
On the file page on GitHub, click the "Raw" button, then Save As.

Run as Administrator: right-click cpu-health-check.bat -> Run as administrator
