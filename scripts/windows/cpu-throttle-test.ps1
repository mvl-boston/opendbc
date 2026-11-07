#Requires -Version 5.1
param(
    [int] $DurationSeconds = 180,
    [int] $SampleIntervalSeconds = 5
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

function Get-CpuFrequencyMhz {
    try {
        $c = Get-Counter -Counter '\Processor Information(_Total)\Processor Frequency' -ErrorAction Stop
        return [int][math]::Round($c.CounterSamples[0].CookedValue, 0)
    }
    catch { return $null }
}

function Get-CpuPerformancePercent {
    try {
        $c = Get-Counter -Counter '\Processor Information(_Total)\% Processor Performance' -ErrorAction Stop
        return [math]::Round($c.CounterSamples[0].CookedValue, 1)
    }
    catch { return $null }
}

function Get-CpuUtilization {
    try {
        $c = Get-Counter -Counter '\Processor(_Total)\% Processor Time' -ErrorAction Stop
        return [math]::Round($c.CounterSamples[0].CookedValue, 1)
    }
    catch { return $null }
}

function Get-Median {
    param([double[]] $Values)
    if (-not $Values -or $Values.Count -eq 0) { return 0 }
    $sorted = $Values | Sort-Object
    $n = $sorted.Count
    if ($n % 2 -eq 1) { return [double]$sorted[[int](($n - 1) / 2)] }
    return ([double]$sorted[$n / 2 - 1] + [double]$sorted[$n / 2]) / 2.0
}

if ($DurationSeconds -lt 60) { $DurationSeconds = 60 }
$threads = [Environment]::ProcessorCount
$runId = Get-Date -Format 'yyyyMMdd_HHmmss'
$workDir = Join-Path $env:TEMP ('cpu-throttle-' + $runId)
New-Item -ItemType Directory -Path $workDir -Force | Out-Null

Write-Section 'Driver-free thermal throttle test'
Write-Host 'No third-party tools, no kernel drivers. Uses Windows performance counters plus'
Write-Host 'measured work throughput. Thermal throttling (PROCHOT at ~100 C on this CPU)'
Write-Host 'shows up as a sustained drop in MHz and/or work-per-second under constant load.'
Write-Host ''
$cpu = Get-CimInstance -ClassName Win32_Processor | Select-Object -First 1
$nominal = [int]$cpu.MaxClockSpeed
Write-Host ('CPU:        {0}' -f $cpu.Name)
Write-Host ('Threads:    {0}   Nominal: {1} MHz' -f $threads, $nominal)
Write-Host ('Duration:   {0}s   Interval: {1}s' -f $DurationSeconds, $SampleIntervalSeconds)
Write-Host ('Work dir:   {0}' -f $workDir)

Write-Section 'Recent firmware / thermal limit events (Event Log)'
try {
    $since = (Get-Date).AddDays(-7)
    $evts = Get-WinEvent -FilterHashtable @{
        LogName      = 'System'
        ProviderName = 'Microsoft-Windows-Kernel-Processor-Power'
        Id           = 37
        StartTime    = $since
    } -ErrorAction Stop
    if ($evts) {
        Write-Warn ('{0} event(s) in last 7 days: "processor speed is being limited by system firmware" (Id 37).' -f $evts.Count)
        $evts | Select-Object -First 3 | ForEach-Object { Write-Host ('  {0}' -f $_.TimeCreated) }
        Write-Host ('THROTTLE_EVENTS: FOUND count={0}' -f @($evts).Count)
    }
    else {
        Write-Ok 'No Kernel-Processor-Power Id 37 (firmware speed limit) events in last 7 days.'
        Write-Host 'THROTTLE_EVENTS: NONE'
    }
}
catch {
    if ($_.Exception.Message -match 'No events were found') {
        Write-Ok 'No Kernel-Processor-Power Id 37 (firmware speed limit) events in last 7 days.'
        Write-Host 'THROTTLE_EVENTS: NONE'
    }
    else {
        Write-Warn ('Could not query Event Log: {0}' -f $_.Exception.Message)
        Write-Host 'THROTTLE_EVENTS: UNKNOWN'
    }
}

Write-Section ('All-core stress with throughput tracking ({0}s)' -f $DurationSeconds)
$endUtc = [datetime]::UtcNow.AddSeconds($DurationSeconds + 8 + $SampleIntervalSeconds + 5)
$jobs = @()
for ($t = 0; $t -lt $threads; $t++) {
    $counterFile = Join-Path $workDir ('t{0}.txt' -f $t)
    $jobs += Start-Job -ScriptBlock {
        param($EndUtc, $CounterFile, $Seed)
        $x = 0.0
        [long]$blocks = 0
        $lastWrite = [datetime]::UtcNow
        while ([datetime]::UtcNow -lt $EndUtc) {
            for ($i = 1; $i -le 20000; $i++) { $x += [math]::Sqrt($i + $Seed) }
            $blocks++
            if (([datetime]::UtcNow - $lastWrite).TotalMilliseconds -ge 500) {
                [System.IO.File]::WriteAllText($CounterFile, $blocks.ToString())
                $lastWrite = [datetime]::UtcNow
            }
        }
        [System.IO.File]::WriteAllText($CounterFile, $blocks.ToString())
    } -ArgumentList $endUtc, $counterFile, $t
}

function Read-TotalBlocks {
    [long]$sum = 0
    Get-ChildItem -LiteralPath $workDir -Filter 't*.txt' -ErrorAction SilentlyContinue | ForEach-Object {
        try {
            $txt = [System.IO.File]::ReadAllText($_.FullName)
            if ($txt -match '^\d+$') { $sum += [long]$txt }
        }
        catch { }
    }
    return $sum
}

Start-Sleep -Seconds 8
$prevBlocks = Read-TotalBlocks
$prevTime = [datetime]::UtcNow

$samples = New-Object System.Collections.Generic.List[object]
$testEnd = [datetime]::UtcNow.AddSeconds($DurationSeconds)
$startTime = [datetime]::UtcNow
Write-Host ('{0,-9} {1,6} {2,9} {3,8} {4,8} {5,12}' -f 'time', 'elapsed', 'MHz', 'perf%', 'util%', 'blocks/sec')

while ([datetime]::UtcNow -lt $testEnd) {
    Start-Sleep -Seconds $SampleIntervalSeconds
    $now = [datetime]::UtcNow
    $blocks = Read-TotalBlocks
    $dt = ($now - $prevTime).TotalSeconds
    $rate = 0.0
    if ($dt -gt 0) { $rate = ($blocks - $prevBlocks) / $dt }
    $prevBlocks = $blocks
    $prevTime = $now

    $mhz = Get-CpuFrequencyMhz
    $perf = Get-CpuPerformancePercent
    $util = Get-CpuUtilization
    $elapsed = [int]($now - $startTime).TotalSeconds

    $samples.Add([pscustomobject]@{
            Elapsed = $elapsed
            Mhz     = $mhz
            Perf    = $perf
            Util    = $util
            Rate    = [math]::Round($rate, 1)
        })

    $mhzTxt = if ($null -ne $mhz) { $mhz } else { 'n/a' }
    $perfTxt = if ($null -ne $perf) { $perf } else { 'n/a' }
    $utilTxt = if ($null -ne $util) { $util } else { 'n/a' }
    Write-Host ('{0,-9} {1,5}s {2,9} {3,8} {4,8} {5,12}' -f (Get-Date -Format 'HH:mm:ss'), $elapsed, $mhzTxt, $perfTxt, $utilTxt, [math]::Round($rate, 1))
}

$jobs | Wait-Job | Out-Null
$jobs | Remove-Job -Force
Remove-Item -LiteralPath $workDir -Recurse -Force -ErrorAction SilentlyContinue

Write-Section 'Analysis'
$valid = @($samples | Where-Object { $_.Rate -gt 0 -and ($null -eq $_.Util -or $_.Util -ge 80) })
if ($valid.Count -lt 4) {
    Write-Warn 'Not enough samples to analyze.'
    Write-Host 'THROTTLE_TEST: FAIL reason=too_few_samples'
    exit 2
}

$baselineWindow = @($valid | Where-Object { $_.Elapsed -le 40 })
if ($baselineWindow.Count -lt 2) { $baselineWindow = @($valid | Select-Object -First 3) }
$baselineRate = Get-Median -Values ($baselineWindow | ForEach-Object { [double]$_.Rate })

$laterWindow = @($valid | Where-Object { $_.Elapsed -gt 40 })
if ($laterWindow.Count -lt 2) { $laterWindow = @($valid | Select-Object -Last 3) }
$laterRate = Get-Median -Values ($laterWindow | ForEach-Object { [double]$_.Rate })
$lastThird = @($valid | Select-Object -Last ([math]::Max(2, [int]($valid.Count / 3))))
$lastRate = Get-Median -Values ($lastThird | ForEach-Object { [double]$_.Rate })

$dropLater = 0.0
$dropLast = 0.0
if ($baselineRate -gt 0) {
    $dropLater = [math]::Round((1 - ($laterRate / $baselineRate)) * 100, 1)
    $dropLast = [math]::Round((1 - ($lastRate / $baselineRate)) * 100, 1)
}

$mhzSamples = @($valid | Where-Object { $null -ne $_.Mhz } | ForEach-Object { [double]$_.Mhz })
$avgMhz = 0; $minMhz = 0; $maxMhz = 0
if ($mhzSamples.Count -gt 0) {
    $avgMhz = [int][math]::Round(($mhzSamples | Measure-Object -Average).Average, 0)
    $minMhz = [int]($mhzSamples | Measure-Object -Minimum).Minimum
    $maxMhz = [int]($mhzSamples | Measure-Object -Maximum).Maximum
}

$utilSamples = @($valid | Where-Object { $null -ne $_.Util } | ForEach-Object { [double]$_.Util })
$avgUtil = 0
if ($utilSamples.Count -gt 0) { $avgUtil = [math]::Round(($utilSamples | Measure-Object -Average).Average, 1) }

Write-Host ('Baseline throughput (first ~40s): {0} blocks/sec' -f [math]::Round($baselineRate, 1))
Write-Host ('Later throughput (after 40s):     {0} blocks/sec  (change: -{1}%)' -f [math]::Round($laterRate, 1), $dropLater)
Write-Host ('Final third throughput:           {0} blocks/sec  (change: -{1}%)' -f [math]::Round($lastRate, 1), $dropLast)
Write-Host ('Clock under load: avg={0} min={1} max={2} MHz (nominal {3})' -f $avgMhz, $minMhz, $maxMhz, $nominal)
Write-Host ('Average CPU utilization: {0}%' -f $avgUtil)
Write-Host ''

$verdict = 'PASS'
$reasons = @()
if ($avgUtil -gt 0 -and $avgUtil -lt 85) {
    $reasons += 'load_not_saturated'
}
if ($nominal -gt 0 -and $avgMhz -gt 0 -and $avgMhz -lt ($nominal * 0.85)) {
    $verdict = 'FAIL'; $reasons += ('avg_MHz_low=' + $avgMhz)
}
if ($dropLast -ge 15) {
    $verdict = 'FAIL'; $reasons += ('throughput_drop=' + $dropLast + '%')
}
elseif ($dropLast -ge 8) {
    if ($verdict -eq 'PASS') { $verdict = 'WARN' }
    $reasons += ('mild_throughput_drop=' + $dropLast + '%')
}

switch ($verdict) {
    'PASS' {
        Write-Ok ('Sustained {0}s all-core load with no meaningful slowdown. No thermal throttling detected.' -f $DurationSeconds)
        Write-Host 'Interpretation: if thermal paste were gone, this 84W CPU would reach ~100 C within'
        Write-Host 'seconds to a minute of this load and MHz/throughput would fall noticeably. They did not.'
        Write-Host 'Cooling is doing its job under full load. Slowness (if any) is not CPU throttling.'
    }
    'WARN' {
        Write-Warn 'Small throughput decline. Could be background activity or mild power/thermal limiting.'
        Write-Host 'Re-run with a longer duration (run-all-cpu-checks.bat 300) and close other apps.'
    }
    'FAIL' {
        Write-Warn 'Sustained slowdown under constant load. This pattern matches thermal or power throttling.'
        Write-Host 'Check: cooler mount / thermal paste, dust in heatsink, CPU fan spinning, BIOS fan curve.'
    }
}

$reasonTxt = if ($reasons.Count -gt 0) { ($reasons -join ',') } else { 'none' }
Write-Host ''
Write-Host ('THROTTLE_TEST: {0} duration={1}s avg_MHz={2} min_MHz={3} throughput_drop={4}% reasons={5}' -f $verdict, $DurationSeconds, $avgMhz, $minMhz, $dropLast, $reasonTxt)
if ($verdict -eq 'FAIL') { exit 2 }
exit 0
