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

function Get-HWiNFOLicenseMarker {
    return (Join-Path $env:TEMP 'cpu-check-hwinfo-license.ok')
}

function Test-HWiNFOLicenseAccepted {
    return (Test-Path -LiteralPath (Get-HWiNFOLicenseMarker))
}

function Set-HWiNFOLicenseAccepted {
    Set-Content -LiteralPath (Get-HWiNFOLicenseMarker) -Value ('ok ' + (Get-Date)) -Force
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

function Get-CpuTempsFromTextFile {
    param([string] $Path)
    if (-not (Test-Path -LiteralPath $Path)) { return @() }
    $temps = @()
    foreach ($line in (Get-Content -LiteralPath $Path -ErrorAction SilentlyContinue)) {
        if ($line -notmatch 'CPU|Core\s*#|Package|Tctl|CCD|Processor') { continue }
        if ($line -match '(?i)(CPU Package|Core\s*#\d+|Tctl|Tdie).*?(?<t>\d{1,3}[.,]\d+)') {
            $v = [double]($Matches['t'] -replace ',', '.')
        }
        elseif ($line -match '(?<t>\d{1,3}[.,]\d+)\s*°?\s*C') {
            $v = [double]($Matches['t'] -replace ',', '.')
        }
        else { continue }
        if ($v -ge 30 -and $v -le 115) {
            $temps += [pscustomobject]@{ Line = $line; Celsius = $v }
        }
    }
    return $temps
}

function Get-CpuTempsFromHwinfoCsv {
    param([string] $CsvPath)
    return Get-CpuTempsFromTextFile -Path $CsvPath
}

function Start-HWiNFOSensorsProcess {
    param(
        [string] $HwinfoExe,
        [string] $LogFile
    )
    if (Test-Path -LiteralPath $LogFile) { Remove-Item -LiteralPath $LogFile -Force -ErrorAction SilentlyContinue }

    $argList = '/SENSORSONLY /SKIPSPLASH /MINIMIZE'
    if ($LogFile) { $argList += (' /LOG="{0}"' -f $LogFile) }

    try {
        $psi = New-Object System.Diagnostics.ProcessStartInfo
        $psi.FileName = $HwinfoExe
        $psi.Arguments = $argList
        $psi.UseShellExecute = $true
        $psi.WindowStyle = [System.Diagnostics.ProcessWindowStyle]::Minimized
        $p = [System.Diagnostics.Process]::Start($psi)
        Start-Sleep -Seconds 10
        if ($p -and -not $p.HasExited) { return $p }
    }
    catch {
        Write-Warn $_.Exception.Message
    }

    try {
        $cmd = 'start "" /MIN "' + $HwinfoExe + '" ' + $argList.Replace('/', '/')
        cmd.exe /c $cmd
        Start-Sleep -Seconds 10
        $p = Get-Process -Name 'HWiNFO64' -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($p) { return $p }
    }
    catch {
        Write-Warn $_.Exception.Message
    }

    return $null
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

$marker = Get-HWiNFOLicenseMarker
if (-not (Test-HWiNFOLicenseAccepted)) {
    Write-Host 'First HWiNFO launch: if a license window appears, click OK/Accept.'
    try {
        $psi = New-Object System.Diagnostics.ProcessStartInfo
        $psi.FileName = $hwi
        $psi.Arguments = '/SKIPSPLASH /SENSORSONLY'
        $psi.UseShellExecute = $true
        $null = [System.Diagnostics.Process]::Start($psi)
        Start-Sleep -Seconds 15
        Set-HWiNFOLicenseAccepted
    }
    catch {
        Write-Warn ('Could not auto-open HWiNFO for license: {0}' -f $_.Exception.Message)
        Write-Host 'NEXT_STEP: Start Menu - open HWiNFO64, accept license, open Sensors, close it.'
    }
}

$stressJobs = $null
$duration = $MonitorSeconds
if ($duration -lt 20) { $duration = 20 }

$logDir = Join-Path $PSScriptRoot 'hwinfo-logs'
New-Item -ItemType Directory -Path $logDir -Force | Out-Null
$liveLog = Join-Path $logDir 'hwinfo-live.csv'

$hwiProc = Start-HWiNFOSensorsProcess -HwinfoExe $hwi -LogFile $liveLog
if (-not $hwiProc) {
    Write-Host 'CPU_TEMP_TEST: FAIL reason=HWiNFO_did_not_start'
    Write-Host 'NEXT_STEP: Start Menu - run HWiNFO64 as Administrator, open Sensors, close, re-run run-all-cpu-checks.bat'
    exit 2
}

if ($StressCpu) {
    Write-Host ('Starting {0}s CPU stress...' -f ($duration + 10))
    $stressJobs = Start-StressJobs -Seconds ($duration + 15)
    Start-Sleep -Seconds 10
}

$peak = 0.0
$sampleCount = 0
$end = [datetime]::UtcNow.AddSeconds($duration)

while ([datetime]::UtcNow -lt $end) {
    $ts = Get-Date -Format 'HH:mm:ss'
    $temps = @()
    if (Test-Path -LiteralPath $liveLog) {
        $temps = Get-CpuTempsFromTextFile -Path $liveLog
    }
    if ($temps.Count -eq 0) {
        $snap = Join-Path $logDir ('snap_{0}.csv' -f (Get-Date -Format 'yyyyMMdd_HHmmss'))
        if (Invoke-HWiNFOCsvSnapshot -HwinfoExe $hwi -OutCsv $snap) {
            $temps = Get-CpuTempsFromTextFile -Path $snap
        }
    }
    if ($temps.Count -gt 0) {
        $max = ($temps.Celsius | Measure-Object -Maximum).Maximum
        $avg = [math]::Round(($temps.Celsius | Measure-Object -Average).Average, 1)
        if ($max -gt $peak) { $peak = $max }
        $sampleCount++
        Write-Host ('{0}  CPU sensors: max={1} C  avg={2} C  (readings={3})' -f $ts, [math]::Round($max, 1), $avg, $temps.Count)
    }
    else {
        Write-Host ('{0}  waiting for HWiNFO sensor data (log={1})' -f $ts, $liveLog)
    }
    Start-Sleep -Seconds $SampleIntervalSeconds
}

if ($hwiProc -and -not $hwiProc.HasExited) {
    Stop-Process -Id $hwiProc.Id -Force -ErrorAction SilentlyContinue
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
Write-Host 'NEXT_STEP: Start Menu - run HWiNFO64 as Administrator, enable Sensors + logging, re-run run-all-cpu-checks.bat'
Write-Warn 'No CPU temps captured from HWiNFO CSV snapshots.'
exit 2
