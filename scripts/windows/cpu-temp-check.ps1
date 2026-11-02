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

function Get-LocalToolRoot {
    return (Join-Path $env:LOCALAPPDATA 'CpuHealthCheck')
}

function Get-LocalHwinfoDir {
    return (Join-Path (Get-LocalToolRoot) 'hwinfo')
}

function Sync-HWiNFOToLocal {
    $localDir = Get-LocalHwinfoDir
    New-Item -ItemType Directory -Path $localDir -Force | Out-Null
    $localExe = Join-Path $localDir 'HWiNFO64.exe'
    if (Test-Path -LiteralPath $localExe) { return $localExe }

    $sources = @(
        (Join-Path $env:ProgramFiles 'HWiNFO64'),
        (Join-Path $PSScriptRoot 'hwinfo')
    )
    foreach ($src in $sources) {
        $srcExe = Join-Path $src 'HWiNFO64.exe'
        if (-not (Test-Path -LiteralPath $srcExe)) { continue }
        Write-Host ('Copying HWiNFO64 from {0} to local profile (fixes referral error)...' -f $src)
        Copy-Item -Path (Join-Path $src '*') -Destination $localDir -Recurse -Force -ErrorAction SilentlyContinue
        if (Test-Path -LiteralPath $localExe) { return $localExe }
    }
    return $null
}

function Get-HWiNFOExe {
    $localExe = Sync-HWiNFOToLocal
    if ($localExe) { return $localExe }
    return $null
}

function Get-IntelPowerGadgetExe {
    $candidates = @(
        (Join-Path (Get-LocalToolRoot) 'PowerGadget3.0.exe'),
        "${env:ProgramFiles(x86)}\Intel\Power Gadget 3.5\PowerGadget3.0.exe",
        "${env:ProgramFiles}\Intel\Power Gadget 3.5\PowerGadget3.0.exe"
    )
    foreach ($p in $candidates) {
        if (Test-Path -LiteralPath $p) { return $p }
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
        if ($line -notmatch 'CPU|Core\s*#|Package|Tctl|CCD|Processor|Temperature') { continue }
        if ($line -match '(?i)(CPU Package|Core\s*#\d+|Tctl|Tdie|Temperature).*?(?<t>\d{1,3}[.,]\d+)') {
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

function Invoke-HWiNFOCsvSnapshot {
    param([string] $HwinfoExe, [string] $OutCsv)
    if (Test-Path -LiteralPath $OutCsv) { Remove-Item -LiteralPath $OutCsv -Force -ErrorAction SilentlyContinue }
    $argSets = @(
        @('/RPT', ('CSV:{0}' -f $OutCsv), '/SKIPSPLASH', '/SENSORSONLY', '/SILENT', '/CLOSE'),
        @('/RPT', ('CSV:{0}' -f $OutCsv), '/SKIPSPLASH', '/SILENT', '/CLOSE')
    )
    foreach ($argList in $argSets) {
        try {
            $psi = New-Object System.Diagnostics.ProcessStartInfo
            $psi.FileName = $HwinfoExe
            $psi.Arguments = ($argList -join ' ')
            $psi.UseShellExecute = $true
            $null = [System.Diagnostics.Process]::Start($psi)
            Start-Sleep -Seconds 4
            if ((Test-Path -LiteralPath $OutCsv) -and (Get-Item $OutCsv).Length -gt 50) { return $true }
        }
        catch { }
    }
    return $false
}

function Start-HWiNFOSensorsProcess {
    param([string] $HwinfoExe, [string] $LogFile)
    if ($LogFile -and (Test-Path -LiteralPath $LogFile)) {
        Remove-Item -LiteralPath $LogFile -Force -ErrorAction SilentlyContinue
    }
    $args = '/SENSORSONLY /SKIPSPLASH /MINIMIZE'
    if ($LogFile) { $args += (' /LOG="{0}"' -f $LogFile) }
    try {
        $psi = New-Object System.Diagnostics.ProcessStartInfo
        $psi.FileName = $HwinfoExe
        $psi.Arguments = $args
        $psi.UseShellExecute = $true
        $psi.WorkingDirectory = Split-Path -Parent $HwinfoExe
        $psi.WindowStyle = [System.Diagnostics.ProcessWindowStyle]::Minimized
        $p = [System.Diagnostics.Process]::Start($psi)
        Start-Sleep -Seconds 10
        if ($p -and -not $p.HasExited) { return $p }
    }
    catch {
        Write-Warn $_.Exception.Message
    }
    try {
        cmd.exe /c ('start "" /MIN /D "{0}" "{1}" /SENSORSONLY /SKIPSPLASH /MINIMIZE' -f (Split-Path $HwinfoExe), $HwinfoExe)
        Start-Sleep -Seconds 10
        $p = Get-Process -Name 'HWiNFO64' -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($p) { return $p }
    }
    catch {
        Write-Warn $_.Exception.Message
    }
    return $null
}

function Invoke-IntelPowerGadgetTempTest {
    param([int] $DurationSeconds)
    $exe = Get-IntelPowerGadgetExe
    if (-not $exe) {
        Write-Host 'Intel Power Gadget not found (optional Intel CPU fallback).'
        Write-Host 'Download: https://www.intel.com/content/www/us/en/developer/articles/tool/power-gadget.html'
        Write-Host ('Copy PowerGadget3.0.exe to: {0}' -f (Get-LocalToolRoot))
        return $null
    }
    Write-Host ('Intel Power Gadget: {0}' -f $exe)
    $stress = $null
    if ($StressCpu) { $stress = Start-StressJobs -Seconds ($DurationSeconds + 5) }
    Start-Sleep -Seconds 5
    try {
        $psi = New-Object System.Diagnostics.ProcessStartInfo
        $psi.FileName = $exe
        $psi.Arguments = ('-duration {0}' -f $DurationSeconds)
        $psi.UseShellExecute = $false
        $psi.RedirectStandardOutput = $true
        $psi.RedirectStandardError = $true
        $psi.WorkingDirectory = Split-Path -Parent $exe
        $p = [System.Diagnostics.Process]::Start($psi)
        $out = $p.StandardOutput.ReadToEnd()
        $p.WaitForExit()
        $peak = 0.0
        foreach ($line in ($out -split "`n")) {
            if ($line -match '(?i)(temp|temperature|package).*?(?<t>\d{1,3}[.,]\d+)') {
                $v = [double]($Matches['t'] -replace ',', '.')
                if ($v -ge 30 -and $v -le 115 -and $v -gt $peak) { $peak = $v }
            }
            elseif ($line -match '(?<t>\d{1,3}[.,]\d+)') {
                $v = [double]($Matches['t'] -replace ',', '.')
                if ($v -ge 45 -and $v -le 115 -and $v -gt $peak) { $peak = $v }
            }
        }
        if ($stress) { $stress | Wait-Job | Out-Null; $stress | Remove-Job -Force }
        if ($peak -gt 0) { return $peak }
    }
    catch {
        Write-Warn $_.Exception.Message
    }
    return $null
}

Write-Section 'CPU temperature (Defender-safe tools)'
Write-Host 'Uses local HWiNFO64 copy under AppData, or Intel Power Gadget on Intel CPUs.'
Write-Host 'Does NOT use LibreHardwareMonitor / WinRing0.'
Write-Host ''

$hwi = Get-HWiNFOExe
$peak = 0.0
$sampleCount = 0
$usedTool = 'none'

if ($hwi) {
    Write-Host ('HWiNFO64 (local): {0}' -f $hwi)
    $duration = $MonitorSeconds
    if ($duration -lt 20) { $duration = 20 }

    $logDir = Join-Path (Get-LocalToolRoot) 'hwinfo-logs'
    New-Item -ItemType Directory -Path $logDir -Force | Out-Null
    $liveLog = Join-Path $logDir 'hwinfo-live.csv'

    $hwiProc = Start-HWiNFOSensorsProcess -HwinfoExe $hwi -LogFile $liveLog
    if ($hwiProc) {
        $usedTool = 'HWiNFO'
        $stressJobs = $null
        if ($StressCpu) {
            Write-Host ('Starting {0}s CPU stress...' -f ($duration + 10))
            $stressJobs = Start-StressJobs -Seconds ($duration + 15)
            Start-Sleep -Seconds 10
        }
        $end = [datetime]::UtcNow.AddSeconds($duration)
        while ([datetime]::UtcNow -lt $end) {
            $ts = Get-Date -Format 'HH:mm:ss'
            $temps = @()
            if (Test-Path -LiteralPath $liveLog) { $temps = Get-CpuTempsFromTextFile -Path $liveLog }
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
                Write-Host ('{0}  CPU sensors: max={1} C  avg={2} C' -f $ts, [math]::Round($max, 1), $avg)
            }
            else {
                Write-Host ('{0}  waiting for HWiNFO sensor data...' -f $ts)
            }
            Start-Sleep -Seconds $SampleIntervalSeconds
        }
        if ($stressJobs) { $stressJobs | Wait-Job | Out-Null; $stressJobs | Remove-Job -Force }
        if (-not $hwiProc.HasExited) { Stop-Process -Id $hwiProc.Id -Force -ErrorAction SilentlyContinue }
    }
    else {
        Write-Warn 'HWiNFO did not start from local copy.'
    }
}
else {
    Write-Warn 'HWiNFO64 not in local AppData yet.'
    Write-Host 'NEXT_STEP: run-all will copy from Program Files or ask for portable ZIP to:'
    Write-Host ('  {0}' -f (Get-LocalHwinfoDir))
}

if ($peak -le 0) {
    Write-Host ''
    Write-Host 'Trying Intel Power Gadget fallback (i7 Haswell and other Intel CPUs)...'
    $pgPeak = Invoke-IntelPowerGadgetTempTest -DurationSeconds $MonitorSeconds
    if ($pgPeak -gt 0) {
        $peak = $pgPeak
        $usedTool = 'IntelPowerGadget'
        $sampleCount = 1
        Write-Host ('Intel Power Gadget peak reading: {0} C' -f [math]::Round($peak, 1))
    }
}

Write-Section 'Summary'
if ($peak -gt 0) {
    Write-Host ('Peak CPU temperature ({0}): {1} C' -f $usedTool, [math]::Round($peak, 1))
    Write-Host ('CPU_TEMP_TEST: PASS peak={0}C tool={1}' -f [math]::Round($peak, 1), $usedTool)
    if ($peak -ge 100) { Write-Warn 'Near Intel throttle (~100 C). Check cooling/paste.' }
    elseif ($peak -ge 85) { Write-Warn 'Warm under load.' }
    else { Write-Ok 'Temperature under load looks reasonable for a short test.' }
    exit 0
}

Write-Host 'CPU_TEMP_TEST: FAIL reason=no_CPU_readings'
Write-Host 'If you saw "A referral was returned from the server" on HWiNFO in Program Files:'
Write-Host ('  1. Extract HWiNFO PORTABLE zip to: {0}' -f (Get-LocalHwinfoDir))
Write-Host '  2. Or install Intel Power Gadget and copy PowerGadget3.0.exe to AppData\CpuHealthCheck\'
Write-Host '  3. Re-run run-all-cpu-checks.bat'
exit 2
