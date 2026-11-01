#Requires -Version 5.1
<#
.SYNOPSIS
  Identify CPU, run quick benchmarks, stress all cores, and sample clocks (and temps if exposed).

.DESCRIPTION
  Helps spot thermal throttling / cooling issues (e.g. dried thermal paste):
  clocks or "Processor Performance" % dropping under sustained load while CPU stays busy.

.PARAMETER DurationSeconds
  Length of the all-core stress phase (default 45).

.PARAMETER SampleIntervalSeconds
  How often to log samples during stress (default 1).

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File .\cpu-health-check.ps1

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File .\cpu-health-check.ps1 -DurationSeconds 90
#>
[CmdletBinding()]
param(
    [int] $DurationSeconds = 45,
    [double] $SampleIntervalSeconds = 1
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Continue'

function Write-Section([string] $Title) {
    Write-Host ""
    Write-Host ("========== {0} ==========" -f $Title) -ForegroundColor Cyan
}

function Write-Warn([string] $Message) {
    Write-Host ("WARN: {0}" -f $Message) -ForegroundColor Yellow
}

function Write-Ok([string] $Message) {
    Write-Host ("OK: {0}" -f $Message) -ForegroundColor Green
}

function Get-ProcessorCim {
    Get-CimInstance -ClassName Win32_Processor
}

function Get-CpuFrequencyMhz {
    try {
        $c = Get-Counter -Counter '\Processor Information(_Total)\Processor Frequency' -ErrorAction Stop
        return [math]::Round($c.CounterSamples[0].CookedValue, 0)
    } catch {
        return $null
    }
}

function Get-CpuPerformancePercent {
    try {
        $c = Get-Counter -Counter '\Processor Information(_Total)\% Processor Performance' -ErrorAction Stop
        return [math]::Round($c.CounterSamples[0].CookedValue, 1)
    } catch {
        return $null
    }
}

function Get-ThermalCelsius {
    $temps = @()
    try {
        $samples = (Get-Counter -Counter '\Thermal Zone Information(*)\Temperature' -ErrorAction Stop).CounterSamples
        foreach ($s in $samples) {
            if ($s.CookedValue -gt 0) { $temps += $s.CookedValue }
        }
    } catch { }

    try {
        Get-CimInstance -Namespace root/wmi -ClassName MSAcpi_ThermalZoneTemperature -ErrorAction Stop | ForEach-Object {
            $k = $_.CurrentTemperature / 10.0
            $c = $k - 273.15
            if ($c -gt -40 -and $c -lt 150) { $temps += $c }
        }
    } catch { }

    if ($temps.Count -eq 0) { return $null }
    return [math]::Round(($temps | Measure-Object -Average).Average, 1)
}

function Measure-SingleThreadScore {
    param([int] $Seconds = 3)
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $x = 0.0
    $iterations = 0
    while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
        for ($i = 1; $i -le 50000; $i++) {
            $x += [math]::Sqrt($i)
        }
        $iterations++
    }
    $sw.Stop()
    [pscustomobject]@{
        Seconds    = [math]::Round($sw.Elapsed.TotalSeconds, 2)
        LoopBlocks = $iterations
        Score      = [math]::Round($iterations / $sw.Elapsed.TotalSeconds, 2)
    }
}

function Start-CpuStressJob {
    param(
        [datetime] $EndUtc,
        [int] $ThreadIndex
    )
    $sb = {
        param($EndUtc, $ThreadIndex)
        $x = 0.0
        while ([datetime]::UtcNow -lt $EndUtc) {
            for ($i = 1; $i -le 80000; $i++) {
                $x += [math]::Sqrt($i + $ThreadIndex)
            }
        }
        return $null
    }
    return Start-Job -ScriptBlock $sb -ArgumentList $EndUtc, $ThreadIndex
}

Write-Section "1. CPU identity"
$cpus = @(Get-ProcessorCim)
if ($cpus.Count -eq 0) {
    Write-Warn "Could not read Win32_Processor."
    exit 1
}

foreach ($cpu in $cpus) {
    Write-Host ("Name:              {0}" -f $cpu.Name)
    Write-Host ("Manufacturer:      {0}" -f $cpu.Manufacturer)
    Write-Host ("Cores / threads:   {0} / {1}" -f $cpu.NumberOfCores, $cpu.NumberOfLogicalProcessors)
    Write-Host ("Socket:            {0}" -f $cpu.SocketDesignation)
    Write-Host ("MaxClockSpeed:     {0} MHz (WMI nominal max)" -f $cpu.MaxClockSpeed)
    Write-Host ("CurrentClockSpeed: {0} MHz (WMI; often stale until load)" -f $cpu.CurrentClockSpeed)
    if ($cpu.ExtClock) { Write-Host ("External clock:    {0} MHz" -f $cpu.ExtClock) }
}

$logical = ($cpus | Measure-Object -Property NumberOfLogicalProcessors -Sum).Sum
Write-Host ("Logical processors for stress: {0}" -f $logical)
Write-Host ("PowerShell:        {0}" -f $PSVersionTable.PSVersion)

Write-Section "2. Idle baseline (frequency counters)"
$idleMhz = Get-CpuFrequencyMhz
$idlePerf = Get-CpuPerformancePercent
$idleTemp = Get-ThermalCelsius
if ($null -ne $idleMhz) { Write-Host ("Processor Frequency (_Total): {0} MHz" -f $idleMhz) }
else { Write-Warn "Counter '\Processor Information(_Total)\Processor Frequency' unavailable. Run as Administrator or use HWiNFO alongside this script." }
if ($null -ne $idlePerf) { Write-Host ("% Processor Performance:      {0}%" -f $idlePerf) }
if ($null -ne $idleTemp) { Write-Host ("Thermal (approx):             {0} C" -f $idleTemp) }
else { Write-Warn "No thermal zones exposed to Windows counters/WMI. Use HWiNFO64 sensors during the stress phase." }

Write-Section "3. Single-thread benchmark (~3s)"
$single = Measure-SingleThreadScore -Seconds 3
Write-Host ("Completed {0} loop blocks in {1}s -> score {2} blocks/s (higher is faster; compare before/after repaste on same PC)" -f $single.LoopBlocks, $single.Seconds, $single.Score)

Write-Section ("4. All-core stress ({0}s) + sampling" -f $DurationSeconds)
Write-Host "Starting CPU stress on $logical threads. Watch for frequency or Performance % falling while load stays high."
Write-Host ""

$endUtc = [datetime]::UtcNow.AddSeconds($DurationSeconds)
$jobs = @()
for ($t = 0; $t -lt $logical; $t++) {
    $jobs += Start-CpuStressJob -EndUtc $endUtc -ThreadIndex $t
}

$log = New-Object System.Collections.Generic.List[string]
$sampleEnd = [datetime]::UtcNow.AddSeconds($DurationSeconds)
while ([datetime]::UtcNow -lt $sampleEnd) {
    $ts = Get-Date -Format 'HH:mm:ss'
    $mhz = Get-CpuFrequencyMhz
    $perf = Get-CpuPerformancePercent
    $temp = Get-ThermalCelsius
    $mhzTxt = if ($null -ne $mhz) { "$mhz MHz" } else { "n/a" }
    $perfTxt = if ($null -ne $perf) { "$perf%" } else { "n/a" }
    $tempTxt = if ($null -ne $temp) { "$temp C" } else { "n/a" }
    $line = "{0}  Freq={1}  Perf={2}  Temp={3}" -f $ts, $mhzTxt, $perfTxt, $tempTxt
    Write-Host $line
    $log.Add($line) | Out-Null
    Start-Sleep -Seconds $SampleIntervalSeconds
}

$jobs | Wait-Job | Out-Null
$jobs | Remove-Job -Force

$mhzValues = @()
foreach ($line in $log) {
    if ($line -match 'Freq=(\d+) MHz') { $mhzValues += [int]$Matches[1] }
}
if ($mhzValues.Count -gt 0) {
    $minMhz = ($mhzValues | Measure-Object -Minimum).Minimum
    $maxMhz = ($mhzValues | Measure-Object -Maximum).Maximum
    $avgMhz = [math]::Round(($mhzValues | Measure-Object -Average).Average, 0)
    Write-Host ""
    Write-Host ("During stress: frequency min={0} max={1} avg={2} MHz (from performance counter)" -f $minMhz, $maxMhz, $avgMhz)
    $nominal = ($cpus | Measure-Object -Property MaxClockSpeed -Maximum).Maximum
    if ($nominal -gt 0 -and $avgMhz -lt ($nominal * 0.75)) {
        Write-Warn "Average frequency under load is well below WMI MaxClockSpeed ($nominal MHz). Sustained thermal or power limit throttling is possible — check temps and cooling."
    } elseif ($minMhz -lt ($maxMhz * 0.85)) {
        Write-Warn "Frequency dropped more than ~15% during the run. If temps are high, suspect cooler contact, paste, dust, or fan curve."
    } else {
        Write-Ok "Frequency stayed relatively stable during this run (still verify temps with HWiNFO if counter data alone looks fine)."
    }
} else {
    Write-Warn "No frequency samples captured — re-run PowerShell as Administrator."
}

Write-Section "5. How to interpret (thermal paste / cooling)"
@"

Signs of poor paste or cooler contact under a 45–90s all-core load:
  • Temperature at or above your CPU's throttle point (often ~100 C on Intel desktop).
  • Processor Frequency or % Processor Performance drifts down while Task Manager still shows ~100% CPU.
  • Single-thread score (section 3) much lower than on the same PC after repasting.

Healthy cooling (typical guidelines, varies by chip and cooler):
  • Temps plateau below throttle with moderate fans.
  • Frequency and Performance % stay fairly flat through the stress window.

Tips:
  • Re-run this script before and after fixing cooling; compare blocks/s and avg MHz.
  • For logging, redirect output:  powershell -ExecutionPolicy Bypass -File cpu-health-check.ps1 *> report.txt
  • HWiNFO64 (Sensors only) complements this script when Windows hides temperatures.

"@ | Write-Host

Write-Section "Done"
