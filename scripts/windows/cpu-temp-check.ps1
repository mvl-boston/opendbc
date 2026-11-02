#Requires -Version 5.1
param(
    [int] $MonitorSeconds = 30,
    [double] $SampleIntervalSeconds = 5,
    [switch] $StressCpu,
    [switch] $RequireHWiNFO
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

function Get-HWiNFOExe {
    $candidates = @(
        (Join-Path $PSScriptRoot 'hwinfo\HWiNFO64.exe'),
        (Join-Path $PSScriptRoot 'HWiNFO64.exe'),
        "${env:ProgramFiles}\HWiNFO64\HWiNFO64.exe",
        "${env:ProgramFiles(x86)}\HWiNFO64\HWiNFO64.exe"
    )
    foreach ($p in $candidates) {
        if (Test-Path -LiteralPath $p) { return (Resolve-Path -LiteralPath $p).Path }
    }
    return $null
}

function Install-HWiNFOViaWinget {
    if (-not (Get-Command winget -ErrorAction SilentlyContinue)) { return $null }
    Write-Host 'Trying winget install REALiX.HWiNFO (official installer)...'
    try {
        $proc = Start-Process -FilePath 'winget.exe' -ArgumentList @(
            'install', '-e', '--id', 'REALiX.HWiNFO',
            '--accept-package-agreements', '--accept-source-agreements', '--silent'
        ) -Wait -PassThru -WindowStyle Hidden
        Start-Sleep -Seconds 5
        if ($proc.ExitCode -le 1) {
            $exe = Get-HWiNFOExe
            if ($exe) { return $exe }
        }
    }
    catch {
        Write-Warn $_.Exception.Message
    }
    return $null
}

function Install-HWiNFOPortable {
    $root = Join-Path $PSScriptRoot 'hwinfo'
    $exe = Join-Path $root 'HWiNFO64.exe'
    if (Test-Path -LiteralPath $exe) { return $exe }

    $zipPath = Join-Path $env:TEMP 'hw64.zip'
    $urls = @(
        'https://www.hwinfo.com/files/hw64.zip',
        'https://www.ftp.hwinfo.com/files/hw64.zip'
    )
    $ua = 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/120.0.0.0 Safari/537.36'

    New-Item -ItemType Directory -Path $root -Force | Out-Null
    foreach ($url in $urls) {
        try {
            Write-Host ('Trying download: {0}' -f $url)
            Invoke-WebRequest -Uri $url -OutFile $zipPath -UseBasicParsing -UserAgent $ua -TimeoutSec 120
            if ((Get-Item $zipPath).Length -lt 1000000) { throw 'Download too small (not a zip?)' }
            Expand-Archive -LiteralPath $zipPath -DestinationPath $root -Force
            Remove-Item $zipPath -Force -ErrorAction SilentlyContinue
            if (Test-Path -LiteralPath $exe) { return $exe }
            $found = Get-ChildItem -LiteralPath $root -Filter 'HWiNFO64.exe' -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1
            if ($found) { return $found.FullName }
        }
        catch {
            Write-Warn $_.Exception.Message
        }
    }
    return $null
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

function Get-CpuTempsFromHwinfoCsv {
    param([string] $CsvPath)
    if (-not (Test-Path -LiteralPath $CsvPath)) { return @() }

    $rows = Import-Csv -LiteralPath $CsvPath -ErrorAction SilentlyContinue
    if (-not $rows) {
        $lines = Get-Content -LiteralPath $CsvPath -ErrorAction SilentlyContinue
        $temps = @()
        foreach ($line in $lines) {
            if ($line -notmatch 'CPU|Core|Package|Tctl|CCD') { continue }
            if ($line -notmatch '([0-9]{1,3}[.,][0-9]+)\s*°?C') { continue }
            $v = [double]($Matches[1] -replace ',', '.')
            if ($v -ge 25 -and $v -le 115) {
                $temps += [pscustomobject]@{ Line = $line; Celsius = $v }
            }
        }
        return $temps
    }

    $out = @()
    foreach ($r in $rows) {
        $text = ($r | Out-String)
        if ($text -notmatch 'CPU|Core|Package|Tctl|CCD|Processor') { continue }
        foreach ($prop in $r.PSObject.Properties) {
            $val = "$($prop.Value)"
            if ($val -match '^([0-9]{1,3}[.,][0-9]+)') {
                $v = [double]($Matches[1] -replace ',', '.')
                if ($v -ge 25 -and $v -le 115) {
                    $out += [pscustomobject]@{
                        Sensor  = $prop.Name
                        Celsius = $v
                        Raw     = $text
                    }
                }
            }
        }
    }
    return $out
}

function Invoke-HWiNFOCsvSnapshot {
    param(
        [string] $HwinfoExe,
        [string] $OutCsv
    )
    if (Test-Path -LiteralPath $OutCsv) { Remove-Item -LiteralPath $OutCsv -Force -ErrorAction SilentlyContinue }

    $dir = Split-Path -Parent $OutCsv
    New-Item -ItemType Directory -Path $dir -Force | Out-Null

    $argSets = @(
        @('/RPT', ('CSV:{0}' -f $OutCsv), '/SKIPSPLASH', '/SENSORSONLY', '/SILENT', '/CLOSE'),
        @('/RPT', ('CSV:{0}' -f $OutCsv), '/SKIPSPLASH', '/SILENT', '/CLOSE'),
        @('/CSV', $OutCsv, '/SKIPSPLASH', '/SENSORSONLY', '/SILENT')
    )

    foreach ($argList in $argSets) {
        try {
            $p = Start-Process -FilePath $HwinfoExe -ArgumentList $argList -PassThru -Wait -WindowStyle Hidden -ErrorAction Stop
            Start-Sleep -Seconds 2
            if ((Test-Path -LiteralPath $OutCsv) -and (Get-Item $OutCsv).Length -gt 50) {
                return $true
            }
        }
        catch {
            # try next
        }
    }
    return $false
}

Write-Section 'CPU temperature (HWiNFO64 - Defender safe)'
Write-Host 'Uses HWiNFO64 (signed). Does NOT use LibreHardwareMonitor / WinRing0.'
Write-Host ''

$hwi = Get-HWiNFOExe
if (-not $hwi) { $hwi = Install-HWiNFOViaWinget }
if (-not $hwi) {
    Write-Host 'HWiNFO64 not found locally. Attempting portable download...'
    $hwi = Install-HWiNFOPortable
}

if (-not $hwi) {
    Write-Host 'CPU_TEMP_TEST: FAIL reason=HWiNFO64_not_installed'
    Write-Host 'NEXT_STEP: run-all-cpu-checks.bat will open the download page - extract to hwinfo\HWiNFO64.exe'
    if ($RequireHWiNFO) { exit 2 }
    Write-Warn 'Could not install HWiNFO automatically.'
    Write-Host 'Falling back to Windows ACPI zones (NOT CPU die temp)...'
    & (Join-Path $PSScriptRoot 'cpu-temp-check-safe.ps1') -MonitorSeconds $MonitorSeconds -SampleIntervalSeconds $SampleIntervalSeconds -StressCpu:$StressCpu
    Write-Host 'CPU_TEMP_TEST: FAIL reason=ACPI_fallback_not_CPU_die'
    exit 2
}

Write-Host ('HWiNFO64: {0}' -f $hwi)

$stressJobs = $null
$duration = $MonitorSeconds
if ($duration -lt 20) { $duration = 20 }

if ($StressCpu) {
    Write-Host ('Starting {0}s CPU stress...' -f ($duration + 10))
    $stressJobs = Start-StressJobs -Seconds ($duration + 15)
    Start-Sleep -Seconds 12
}

$logDir = Join-Path $PSScriptRoot 'hwinfo-logs'
New-Item -ItemType Directory -Path $logDir -Force | Out-Null

$peak = 0.0
$sampleCount = 0
$end = [datetime]::UtcNow.AddSeconds($duration)

while ([datetime]::UtcNow -lt $end) {
    $ts = Get-Date -Format 'HH:mm:ss'
    $csv = Join-Path $logDir ('sample_{0}.csv' -f (Get-Date -Format 'yyyyMMdd_HHmmss'))
    $ok = Invoke-HWiNFOCsvSnapshot -HwinfoExe $hwi -OutCsv $csv
    if ($ok) {
        $temps = Get-CpuTempsFromHwinfoCsv -CsvPath $csv
        if ($temps.Count -gt 0) {
            $max = ($temps.Celsius | Measure-Object -Maximum).Maximum
            $avg = [math]::Round(($temps.Celsius | Measure-Object -Average).Average, 1)
            if ($max -gt $peak) { $peak = $max }
            $sampleCount++
            Write-Host ('{0}  CPU sensors: max={1} C  avg={2} C  (samples={3})' -f $ts, [math]::Round($max, 1), $avg, $temps.Count)
        }
        else {
            Write-Host ('{0}  CSV created but no CPU temp rows parsed - open HWiNFO Sensors once manually' -f $ts)
        }
    }
    else {
        Write-Host ('{0}  HWiNFO snapshot failed - run HWiNFO64.exe once as Admin to accept license' -f $ts)
    }
    Start-Sleep -Seconds $SampleIntervalSeconds
}

if ($stressJobs) {
    $stressJobs | Wait-Job | Out-Null
    $stressJobs | Remove-Job -Force
}

Write-Section 'Summary'
if ($peak -gt 0) {
    Write-Host ('Peak CPU-related reading this run: {0} C' -f [math]::Round($peak, 1))
    Write-Host ('CPU_TEMP_TEST: PASS peak={0}C samples={1}' -f [math]::Round($peak, 1), $sampleCount)
    if ($peak -ge 100) {
        Write-Warn 'Near typical Intel throttle (~100 C). Check paste, cooler mount, dust, fans.'
    }
    elseif ($peak -ge 85) {
        Write-Warn 'Warm under load. Monitor with longer tests or improve cooling if sustained.'
    }
    else {
        Write-Ok 'CPU temperature under load looks reasonable for a short stress test.'
    }
    exit 0
}

Write-Host 'CPU_TEMP_TEST: FAIL reason=no_CPU_readings'
Write-Host 'NEXT_STEP: run hwinfo\HWiNFO64.exe as Administrator once (accept license), then re-run run-all-cpu-checks.bat'
Write-Warn 'No CPU temps captured from HWiNFO CSV snapshots.'
exit 2
