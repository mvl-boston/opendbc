#Requires -Version 5.1
<#
.SYNOPSIS
  Reliable CPU temperature readout on Win10 using LibreHardwareMonitor (direct sensor access).

.PARAMETER MonitorSeconds
  How long to sample (default 30). Use 0 for one-shot.

.PARAMETER SampleIntervalSeconds
  Seconds between samples (default 2).

.PARAMETER StressCpu
  Run all-core load while monitoring (helps expose cooling limits).

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File .\cpu-temp-check.ps1

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File .\cpu-temp-check.ps1 -MonitorSeconds 60 -StressCpu
#>
[CmdletBinding()]
param(
    [int] $MonitorSeconds = 30,
    [double] $SampleIntervalSeconds = 2,
    [switch] $StressCpu,
    [switch] $InstallOnly
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function Write-Section {
    param([string] $Title)
    Write-Host ''
    Write-Host ('========== {0} ==========' -f $Title) -ForegroundColor Cyan
}

function Write-Warn {
    param([string] $Message)
    Write-Host ('WARN: {0}' -f $Message) -ForegroundColor Yellow
}

function Write-Ok {
    param([string] $Message)
    Write-Host ('OK: {0}' -f $Message) -ForegroundColor Green
}

function Test-IsAdmin {
    $id = [Security.Principal.WindowsIdentity]::GetCurrent()
    $p = New-Object Security.Principal.WindowsPrincipal($id)
    return $p.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

function Get-LhmRoot {
    return (Join-Path $PSScriptRoot 'lhm')
}

function Install-LibreHardwareMonitor {
    $root = Get-LhmRoot
    $exe = Join-Path $root 'LibreHardwareMonitor.exe'
    if (Test-Path -LiteralPath $exe) { return $exe }

    $zipUrl = 'https://github.com/LibreHardwareMonitor/LibreHardwareMonitor/releases/download/v0.9.4/LibreHardwareMonitor-net472.zip'
    $zipPath = Join-Path $env:TEMP 'LibreHardwareMonitor-net472.zip'

    Write-Host ('Downloading LibreHardwareMonitor 0.9.4 to {0} ...' -f $root)
    New-Item -ItemType Directory -Path $root -Force | Out-Null
    Invoke-WebRequest -Uri $zipUrl -OutFile $zipPath -UseBasicParsing
    Expand-Archive -LiteralPath $zipPath -DestinationPath $root -Force
    Remove-Item -LiteralPath $zipPath -Force -ErrorAction SilentlyContinue

    if (-not (Test-Path -LiteralPath $exe)) {
        throw 'LibreHardwareMonitor download failed or unexpected zip layout.'
    }
    return $exe
}

function Initialize-LhmComputer {
    $root = Get-LhmRoot
    $null = Install-LibreHardwareMonitor

    $hid = Join-Path $root 'HidSharp.dll'
    $lib = Join-Path $root 'LibreHardwareMonitorLib.dll'
    if (-not (Test-Path -LiteralPath $lib)) { throw 'LibreHardwareMonitorLib.dll missing.' }

    foreach ($path in @($hid, $lib)) {
        if (Test-Path -LiteralPath $path) {
            try {
                Add-Type -Path $path -ErrorAction Stop
            }
            catch {
                if ($_.Exception.Message -notmatch 'already (exists|added)') { throw }
            }
        }
    }

    $computer = New-Object LibreHardwareMonitor.Hardware.Computer
    $computer.IsCpuEnabled = $true
    $computer.Open()
    return $computer
}

function Get-CpuTemperatureSamples {
    param(
        [Parameter(Mandatory = $true)]
        $Computer
    )

    $cpuType = [LibreHardwareMonitor.Hardware.HardwareType]::Cpu
    $tempType = [LibreHardwareMonitor.Hardware.SensorType]::Temperature
    $rows = New-Object System.Collections.Generic.List[object]

    function Add-HwSensors {
        param($Hardware)
        $Hardware.Update()
        foreach ($sensor in $Hardware.Sensors) {
            if ($sensor.SensorType -ne $tempType) { continue }
            if (-not $sensor.Value.HasValue) { continue }
            $v = [double]$sensor.Value.Value
            if ($v -lt 15 -or $v -gt 115) { continue }
            [void]$rows.Add([pscustomobject]@{
                    Hardware = $Hardware.Name
                    Sensor   = $sensor.Name
                    Celsius  = [math]::Round($v, 1)
                })
        }
        foreach ($sub in $Hardware.SubHardware) {
            Add-HwSensors -Hardware $sub
        }
    }

    foreach ($hw in $Computer.Hardware) {
        if ($hw.HardwareType -ne $cpuType) { continue }
        Add-HwSensors -Hardware $hw
    }

    return $rows
}

function Format-TempSummary {
    param($Rows)
    if (-not $Rows -or $Rows.Count -eq 0) { return $null }

    $package = $Rows | Where-Object { $_.Sensor -match 'Package|Tctl|CPU' } | Select-Object -First 1
    $cores = $Rows | Where-Object { $_.Sensor -match 'Core|CCD|Die' }
    $all = $Rows | ForEach-Object { $_.Celsius }

    return [pscustomobject]@{
        PackageC    = if ($package) { $package.Celsius } else { $null }
        CoreAvgC    = if ($cores) { [math]::Round(($cores.Celsius | Measure-Object -Average).Average, 1) } else { $null }
        CoreMaxC    = if ($cores) { ($cores.Celsius | Measure-Object -Maximum).Maximum } else { ($all | Measure-Object -Maximum).Maximum }
        CoreMinC    = if ($cores) { ($cores.Celsius | Measure-Object -Minimum).Minimum } else { ($all | Measure-Object -Minimum).Minimum }
        SensorCount = $Rows.Count
        Rows        = $Rows
    }
}

function Start-StressJobs {
    param([int] $Seconds)
    $endUtc = [datetime]::UtcNow.AddSeconds($Seconds)
    $n = [Environment]::ProcessorCount
    $jobs = @()
    for ($t = 0; $t -lt $n; $t++) {
        $jobs += Start-Job -ScriptBlock {
            param($EndUtc, $ThreadIndex)
            $x = 0.0
            while ([datetime]::UtcNow -lt $EndUtc) {
                for ($i = 1; $i -le 80000; $i++) { $x += [math]::Sqrt($i + $ThreadIndex) }
            }
        } -ArgumentList $endUtc, $t
    }
    return $jobs
}

Write-Section 'CPU temperature check (LibreHardwareMonitor)'
Write-Host ('Admin:       {0}' -f (Test-IsAdmin))
Write-Host ('PowerShell:  {0}' -f $PSVersionTable.PSVersion)
if (-not (Test-IsAdmin)) {
    Write-Warn 'Run as Administrator for best access to CPU sensors (right-click cpu-temp-check.bat).'
}

try {
    $null = Install-LibreHardwareMonitor
}
catch {
    Write-Warn $_.Exception.Message
    exit 1
}

if ($InstallOnly) {
    Write-Ok ('LibreHardwareMonitor ready in {0}' -f (Get-LhmRoot))
    exit 0
}

$computer = $null
$stressJobs = $null
try {
    $computer = Initialize-LhmComputer

    Write-Section 'One-shot CPU sensors'
    $rows = Get-CpuTemperatureSamples -Computer $computer
    $summary = Format-TempSummary -Rows $rows
    if (-not $summary) {
        Write-Warn 'No CPU temperature sensors found. Try Administrator, or reboot after first LHM download.'
        exit 2
    }

    foreach ($r in ($rows | Sort-Object Sensor)) {
        Write-Host ('  {0} / {1}: {2} C' -f $r.Hardware, $r.Sensor, $r.Celsius)
    }
    Write-Host ''
    if ($null -ne $summary.PackageC) {
        Write-Host ('Package (preferred): {0} C' -f $summary.PackageC)
    }
    Write-Host ('Core max: {0} C   Core avg: {1} C' -f $summary.CoreMaxC, $summary.CoreAvgC)

    if ($MonitorSeconds -le 0) {
        Write-Section 'Done (one-shot)'
        exit 0
    }

    Write-Section ('Monitoring {0}s (interval {1}s){2}' -f $MonitorSeconds, $SampleIntervalSeconds, $(if ($StressCpu) { ' WITH all-core stress' } else { '' }))
    if ($StressCpu) {
        $stressJobs = Start-StressJobs -Seconds ($MonitorSeconds + 5)
    }

    $maxSeen = 0.0
    $end = [datetime]::UtcNow.AddSeconds($MonitorSeconds)
    while ([datetime]::UtcNow -lt $end) {
        $rows = Get-CpuTemperatureSamples -Computer $computer
        $summary = Format-TempSummary -Rows $rows
        $ts = Get-Date -Format 'HH:mm:ss'
        if ($summary) {
            $line = '{0}  Package={1}  CoreMax={2}  CoreAvg={3}' -f $ts,
            $(if ($null -ne $summary.PackageC) { ('{0} C' -f $summary.PackageC) } else { 'n/a' }),
            ('{0} C' -f $summary.CoreMaxC),
            $(if ($null -ne $summary.CoreAvgC) { ('{0} C' -f $summary.CoreAvgC) } else { 'n/a' })
            Write-Host $line
            if ($summary.CoreMaxC -gt $maxSeen) { $maxSeen = $summary.CoreMaxC }
        }
        else {
            Write-Host ('{0}  (no readings)' -f $ts)
        }
        Start-Sleep -Seconds $SampleIntervalSeconds
    }

    Write-Section 'Assessment'
    Write-Host ('Peak core temperature this run: {0} C' -f [math]::Round($maxSeen, 1))
    if ($maxSeen -ge 100) {
        Write-Warn 'At or above ~100 C (typical Intel throttle zone for Haswell). Check paste, mount pressure, dust, and fans.'
    }
    elseif ($maxSeen -ge 85) {
        Write-Warn 'Warm under load. Cooling may be marginal; watch for rising temps over longer runs.'
    }
    else {
        Write-Ok 'Temperatures look reasonable for a short load test.'
    }
    if (-not $StressCpu) {
        Write-Host 'Tip: re-run with -StressCpu to see temps under full load (paste/cooler stress test).'
    }
}
finally {
    if ($stressJobs) {
        $stressJobs | Wait-Job | Out-Null
        $stressJobs | Remove-Job -Force
    }
    if ($computer) {
        $computer.Close()
    }
}

Write-Section 'Done'
