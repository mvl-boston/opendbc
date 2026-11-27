#Requires -Version 5.1
param(
    [string] $TargetDir = $PSScriptRoot
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$BaseUrl = 'https://raw.githubusercontent.com/mvl-boston/opendbc/cursor/windows-cpu-health-check-65b2/scripts/windows'

$Files = @(
    'cpu-health-check.ps1',
    'cpu-health-check.bat',
    'cpu-throttle-test.ps1',
    'disk-health-check.ps1',
    'run-disk-check.bat',
    'rma-evidence.ps1',
    'make-rma-evidence.bat',
    'cleanup-cpu-tools.bat',
    'download-cpu-tools.bat',
    'download-cpu-tools.ps1',
    'run-all-cpu-checks.bat',
    'README-cpu-health-check.txt'
)

function Test-ScriptNotHtml {
    param([string] $Path)
    $lines = Get-Content -LiteralPath $Path -TotalCount 8
    $head = $lines -join [Environment]::NewLine
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

Write-Host ''
Write-Host 'OK: All files downloaded and verified.'
Write-Host ''
Write-Host 'Next steps (run as Administrator):'
Write-Host '  run-all-cpu-checks.bat       - one-click: speed test + throttle test + log'
Write-Host '  run-disk-check.bat           - one-click: SSD/HDD health + speed + log'
Write-Host '  make-rma-evidence.bat        - one-click: warranty-claim evidence file for a slow SSD'
Write-Host '  cpu-health-check.bat         - 45s clocks / stress only'
Write-Host '  cleanup-cpu-tools.bat        - uninstall HWiNFO/LHM and remove downloaded files'
Write-Host ''
