﻿#Requires -Version 5.1
<#
.SYNOPSIS
  Download CPU health + temperature scripts from GitHub raw (not HTML pages).

.PARAMETER TargetDir
  Folder to write files (default: directory containing this script).

.PARAMETER IncludeLibreHardwareMonitor
  Also download and extract LibreHardwareMonitor (~2 MB) for cpu-temp-check.
#>
[CmdletBinding()]
param(
    [string] $TargetDir = $PSScriptRoot,
    [switch] $IncludeLibreHardwareMonitor
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$BaseUrl = 'https://raw.githubusercontent.com/mvl-boston/opendbc/cursor/windows-cpu-health-check-65b2/scripts/windows'

$Files = @(
    'cpu-health-check.ps1',
    'cpu-health-check.bat',
    'cpu-temp-check.ps1',
    'cpu-temp-check.bat',
    'download-cpu-tools.bat',
    'download-cpu-tools.ps1',
    'download-cpu-health-check.bat',
    'download-cpu-temp-check.bat',
    'README-cpu-health-check.txt'
)

function Test-ScriptNotHtml {
    param([string] $Path)
    $head = (Get-Content -LiteralPath $Path -TotalCount 8) -join [Environment]::NewLine
    if ($head -match '<!DOCTYPE|<html|githubassets|/assets/') {
        throw ('File looks like a web page, not a script: {0}' -f $Path)
    }
    if ($Path -like '*.ps1' -and $head -notmatch '#Requires') {
        throw ('PowerShell file missing #Requires header: {0}' -f $Path)
    }
}

New-Item -ItemType Directory -Path $TargetDir -Force | Out-Null
Write-Host ('Downloading to: {0}' -f (Resolve-Path -LiteralPath $TargetDir))
Write-Host ('Source: {0}' -f $BaseUrl)
Write-Host ''

foreach ($name in $Files) {
    $uri = '{0}/{1}' -f $BaseUrl, $name
    $out = Join-Path $TargetDir $name
    Write-Host ('  {0}' -f $name)
    Invoke-WebRequest -Uri $uri -OutFile $out -UseBasicParsing
    if ($name -like '*.ps1') {
        Test-ScriptNotHtml -Path $out
    }
}

if ($IncludeLibreHardwareMonitor) {
    $lhmRoot = Join-Path $TargetDir 'lhm'
    $lhmExe = Join-Path $lhmRoot 'LibreHardwareMonitor.exe'
    if (-not (Test-Path -LiteralPath $lhmExe)) {
        Write-Host ''
        Write-Host 'Downloading LibreHardwareMonitor 0.9.4...'
        $zipUrl = 'https://github.com/LibreHardwareMonitor/LibreHardwareMonitor/releases/download/v0.9.4/LibreHardwareMonitor-net472.zip'
        $zipPath = Join-Path $env:TEMP 'LibreHardwareMonitor-net472.zip'
        New-Item -ItemType Directory -Path $lhmRoot -Force | Out-Null
        Invoke-WebRequest -Uri $zipUrl -OutFile $zipPath -UseBasicParsing
        Expand-Archive -LiteralPath $zipPath -DestinationPath $lhmRoot -Force
        Remove-Item -LiteralPath $zipPath -Force -ErrorAction SilentlyContinue
        if (-not (Test-Path -LiteralPath $lhmExe)) {
            throw 'LibreHardwareMonitor zip layout unexpected.'
        }
        Write-Host ('  OK: {0}' -f $lhmExe)
    }
    else {
        Write-Host ''
        Write-Host 'LibreHardwareMonitor already present in lhm\'
    }
}

Write-Host ''
Write-Host 'OK: All files downloaded and verified.'
Write-Host ''
Write-Host 'Next steps (run as Administrator):'
Write-Host '  cpu-health-check.bat          - clocks / stress'
Write-Host '  cpu-temp-check.bat            - reliable temps (~30s)'
Write-Host '  cpu-temp-check.bat 60 stress  - temps under full load'
Write-Host ''
