#Requires -Version 5.1
param(
    [int] $MonitorSeconds = 30,
    [double] $SampleIntervalSeconds = 2,
    [switch] $StressCpu
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Continue'

function Write-Section {
    param([string] $Title)
    Write-Host ''
    Write-Host ('========== {0} ==========' -f $Title) -ForegroundColor Cyan
}

function Write-Warn {
    param([string] $Message)
    Write-Host ('WARN: {0}' -f $Message) -ForegroundColor Yellow
}

function Get-SaneThermalSamples {
    $out = New-Object System.Collections.Generic.List[object]

    try {
        $samples = (Get-Counter -Counter '\Thermal Zone Information(*)\Temperature' -ErrorAction Stop).CounterSamples
        foreach ($s in $samples) {
            $c = [double]$s.CookedValue
            if ($c -ge 25 -and $c -le 105) {
                [void]$out.Add([pscustomobject]@{ Source = 'ThermalZone'; Name = $s.Path; Celsius = [math]::Round($c, 1) })
            }
        }
    }
    catch { }

    try {
        Get-CimInstance -Namespace root/wmi -ClassName MSAcpi_ThermalZoneTemperature -ErrorAction Stop | ForEach-Object {
            $c = ($_.CurrentTemperature / 10.0) - 273.15
            if ($c -ge 25 -and $c -le 105) {
                [void]$out.Add([pscustomobject]@{ Source = 'ACPI'; Name = $_.InstanceName; Celsius = [math]::Round($c, 1) })
            }
        }
    }
    catch { }

    return $out
}

function Get-TempSummary {
    param($Rows)
    if (-not $Rows -or $Rows.Count -eq 0) { return $null }
    $max = ($Rows.Celsius | Measure-Object -Maximum).Maximum
    $avg = [math]::Round(($Rows.Celsius | Measure-Object -Average).Average, 1)
    return [pscustomobject]@{ MaxC = $max; AvgC = $avg; Count = $Rows.Count }
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

Write-Section 'CPU temperature (Windows built-in sensors only)'
Write-Host 'No kernel drivers (LibreHardwareMonitor / WinRing0) - avoids Defender blocks.'
Write-Host 'Tradeoff: many PCs do not expose CPU die temp here; values may be missing or approximate.'
Write-Host ''
Write-Host 'For die-accurate temps: run HWiNFO64 Sensors-only (official site) while this stress runs,'
Write-Host 'or see DEFENDER-WINRING0.txt if you choose LibreHardwareMonitor + a Defender exclusion.'
Write-Host ''

$rows = Get-SaneThermalSamples
if ($rows.Count -gt 0) {
    foreach ($r in $rows) {
        Write-Host ('  [{0}] {1}: {2} C' -f $r.Source, $r.Name, $r.Celsius)
    }
}
else {
    Write-Warn 'No sane temperature readings at idle (25-105 C). Common on desktops.'
}

$stressJobs = $null
if ($MonitorSeconds -gt 0) {
    Write-Section ('Monitoring {0}s{1}' -f $MonitorSeconds, $(if ($StressCpu) { ' with CPU stress' } else { '' }))
    if ($StressCpu) { $stressJobs = Start-StressJobs -Seconds ($MonitorSeconds + 5) }

    $peak = 0.0
    $end = [datetime]::UtcNow.AddSeconds($MonitorSeconds)
    while ([datetime]::UtcNow -lt $end) {
        $rows = Get-SaneThermalSamples
        $sum = Get-TempSummary -Rows $rows
        $ts = Get-Date -Format 'HH:mm:ss'
        if ($sum) {
            Write-Host ('{0}  zones={1}  max={2} C  avg={3} C' -f $ts, $sum.Count, $sum.MaxC, $sum.AvgC)
            if ($sum.MaxC -gt $peak) { $peak = $sum.MaxC }
        }
        else {
            Write-Host ('{0}  (no readings in 25-105 C range)' -f $ts)
        }
        Start-Sleep -Seconds $SampleIntervalSeconds
    }

    Write-Section 'Summary'
    if ($peak -gt 0) {
        Write-Host ('Peak zone temperature seen: {0} C' -f [math]::Round($peak, 1))
        if ($peak -ge 95) {
            Write-Warn 'High reading - confirm with HWiNFO if this is really CPU die temp.'
        }
    }
    else {
        Write-Warn 'Windows did not report usable CPU/zone temps during this run.'
        Write-Host 'Use HWiNFO64 (Sensors) under load, or check BIOS hardware monitor.'
    }
}

if ($stressJobs) {
    $stressJobs | Wait-Job | Out-Null
    $stressJobs | Remove-Job -Force
}

Write-Section 'Done'
