#Requires -Version 5.1
<#
.SYNOPSIS
  CPU identity, quick benchmark, all-core stress, clock sampling (Win10 / PowerShell 5.1).

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File .\cpu-health-check.ps1
#>
[CmdletBinding()]
param(
    [int] $DurationSeconds = 45,
    [double] $SampleIntervalSeconds = 1
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

function Write-Ok {
    param([string] $Message)
    Write-Host ('OK: {0}' -f $Message) -ForegroundColor Green
}

function Get-ProcessorCim {
    Get-CimInstance -ClassName Win32_Processor
}

function Get-CpuFrequencyMhz {
    try {
        $path = '\Processor Information(_Total)\Processor Frequency'
        $c = Get-Counter -Counter $path -ErrorAction Stop
        return [math]::Round($c.CounterSamples[0].CookedValue, 0)
    }
    catch {
        return $null
    }
}

function Get-CpuPerformancePercent {
    try {
        $path = '\Processor Information(_Total)\% Processor Performance'
        $c = Get-Counter -Counter $path -ErrorAction Stop
        return [math]::Round($c.CounterSamples[0].CookedValue, 1)
    }
    catch {
        return $null
    }
}

function Get-WmiClockMhz {
    $sum = 0
    $n = 0
    Get-CimInstance -ClassName Win32_Processor | ForEach-Object {
        if ($_.CurrentClockSpeed -gt 0) {
            $sum += $_.CurrentClockSpeed
            $n++
        }
    }
    if ($n -eq 0) { return $null }
    return [math]::Round($sum / $n, 0)
}

function Get-ThermalCelsius {
    $temps = @()
    try {
        $samples = (Get-Counter -Counter '\Thermal Zone Information(*)\Temperature' -ErrorAction Stop).CounterSamples
        foreach ($s in $samples) {
            if ($s.CookedValue -ge 15 -and $s.CookedValue -le 105) { $temps += $s.CookedValue }
        }
    }
    catch {
        # ignore
    }

    try {
        Get-CimInstance -Namespace root/wmi -ClassName MSAcpi_ThermalZoneTemperature -ErrorAction Stop | ForEach-Object {
            $k = $_.CurrentTemperature / 10.0
            $c = $k - 273.15
            if ($c -ge 15 -and $c -le 105) { $temps += $c }
        }
    }
    catch {
        # ignore
    }

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

Write-Section 'Environment'
Write-Host ('OS:          {0}' -f [System.Environment]::OSVersion.VersionString)
Write-Host ('PowerShell:  {0}' -f $PSVersionTable.PSVersion)

Write-Section '1. CPU identity'
$cpus = @(Get-ProcessorCim)
if ($cpus.Count -eq 0) {
    Write-Warn 'Could not read Win32_Processor.'
    exit 1
}

foreach ($cpu in $cpus) {
    Write-Host ('Name:              {0}' -f $cpu.Name)
    Write-Host ('Manufacturer:      {0}' -f $cpu.Manufacturer)
    Write-Host ('Cores / threads:   {0} / {1}' -f $cpu.NumberOfCores, $cpu.NumberOfLogicalProcessors)
    Write-Host ('Socket:            {0}' -f $cpu.SocketDesignation)
    Write-Host ('MaxClockSpeed:     {0} MHz (WMI nominal max)' -f $cpu.MaxClockSpeed)
    Write-Host ('CurrentClockSpeed: {0} MHz (WMI; updates slowly)' -f $cpu.CurrentClockSpeed)
}

$logical = ($cpus | Measure-Object -Property NumberOfLogicalProcessors -Sum).Sum
Write-Host ('Logical processors for stress: {0}' -f $logical)

Write-Section '2. Idle baseline'
$idleMhz = Get-CpuFrequencyMhz
$idlePerf = Get-CpuPerformancePercent
$idleTemp = Get-ThermalCelsius
$idleWmiMhz = Get-WmiClockMhz

if ($null -ne $idleMhz) {
    Write-Host ('Processor Frequency counter: {0} MHz' -f $idleMhz)
}
else {
    Write-Warn 'Processor Frequency counter unavailable. Run as Administrator for best results.'
    if ($null -ne $idleWmiMhz) {
        Write-Host ('WMI CurrentClockSpeed (avg): {0} MHz' -f $idleWmiMhz)
    }
}

if ($null -ne $idlePerf) {
    Write-Host ('Processor Performance %:     {0}' -f $idlePerf)
}

if ($null -ne $idleTemp) {
    Write-Host ('Thermal (approx):              {0} C' -f $idleTemp)
}
else {
    Write-Warn 'No trustworthy CPU temp from Windows counters. Use cpu-temp-check.bat (LibreHardwareMonitor) or HWiNFO64.'
}

Write-Section '3. Single-thread benchmark (~3s)'
$single = Measure-SingleThreadScore -Seconds 3
Write-Host ('Score: {0} blocks/s ({1} blocks in {2}s). Compare on the same PC before/after cooling fixes.' -f $single.Score, $single.LoopBlocks, $single.Seconds)

Write-Section ('4. All-core stress ({0}s)' -f $DurationSeconds)
Write-Host ('Starting stress on {0} threads...' -f $logical)
Write-Host ''

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
    if ($null -eq $mhz) { $mhz = Get-WmiClockMhz }
    $perf = Get-CpuPerformancePercent
    $temp = Get-ThermalCelsius

    if ($null -ne $mhz) { $mhzTxt = ('{0} MHz' -f $mhz) } else { $mhzTxt = 'n/a' }
    if ($null -ne $perf) { $perfTxt = ('{0} pct' -f $perf) } else { $perfTxt = 'n/a' }
    if ($null -ne $temp) { $tempTxt = ('{0} C' -f $temp) } else { $tempTxt = 'n/a' }

    $line = ('{0}  Freq={1}  Perf={2}  Temp={3}' -f $ts, $mhzTxt, $perfTxt, $tempTxt)
    Write-Host $line
    [void]$log.Add($line)
    Start-Sleep -Seconds $SampleIntervalSeconds
}

$jobs | Wait-Job | Out-Null
$jobs | Remove-Job -Force

$mhzValues = New-Object System.Collections.Generic.List[int]
foreach ($line in $log) {
    if ($line -match 'Freq=([0-9]+) MHz') {
        [void]$mhzValues.Add([int]$Matches[1])
    }
}

if ($mhzValues.Count -gt 0) {
    $nominal = ($cpus | Measure-Object -Property MaxClockSpeed -Maximum).Maximum
    $forStability = @($mhzValues | Where-Object { $_ -ge [math]::Round($nominal * 0.75) })
    if ($forStability.Count -eq 0) { $forStability = @($mhzValues) }

    $minMhz = ($forStability | Measure-Object -Minimum).Minimum
    $maxMhz = ($mhzValues | Measure-Object -Maximum).Maximum
    $avgMhz = [math]::Round(($mhzValues | Measure-Object -Average).Average, 0)
    Write-Host ''
    Write-Host ('During stress: MHz min={0} max={1} avg={2}' -f $minMhz, $maxMhz, $avgMhz)

    if ($nominal -gt 0 -and $avgMhz -lt ($nominal * 0.75)) {
        Write-Warn ('Average MHz is well below WMI max ({0} MHz). Possible thermal or power throttling.' -f $nominal)
        Write-Host 'SPEED_TEST: WARN reason=low_average_MHz'
    }
    elseif ($minMhz -lt ($maxMhz * 0.85)) {
        Write-Warn 'MHz dropped more than ~15% during the run. Check temps, paste, dust, and fans.'
        Write-Host 'SPEED_TEST: WARN reason=Mhz_dropped_under_load'
    }
    else {
        Write-Ok 'MHz stayed fairly stable this run (still confirm temps with HWiNFO).'
        Write-Host ('SPEED_TEST: PASS avg_MHz={0} min={1} max={2}' -f $avgMhz, $minMhz, $maxMhz)
    }
}
else {
    Write-Warn 'No MHz samples in log. Re-run as Administrator.'
    Write-Host 'SPEED_TEST: FAIL reason=no_frequency_samples'
}

Write-Section '5. Interpretation'
Write-Host '- Bad paste / poor cooler contact: high temps, MHz or Performance pct falls while CPU stays at 100%.'
Write-Host '- Healthy: temps below throttle, MHz fairly flat for the whole stress window.'
Write-Host '- Save log: powershell -ExecutionPolicy Bypass -File cpu-health-check.ps1 *> report.txt'

Write-Section 'Done'
