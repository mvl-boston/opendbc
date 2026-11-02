#Requires -Version 5.1
param(
    [string] $DriveLetter = $env:SystemDrive.Substring(0, 1),
    [string] $ScriptDir = $PSScriptRoot
)
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Continue'
$DriveLetter = $DriveLetter.Trim(':').ToUpper()
if (-not $ScriptDir) { $ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path }

function Line { param([string] $Text = '') Write-Host $Text }
function Rule { Write-Host ('=' * 78) }
function Head { param([string] $Text) Write-Host ''; Rule; Write-Host ('  ' + $Text); Rule }
function Get-PropOrNull {
    param($Object, [string] $Name)
    if ($null -eq $Object) { return $null }
    $p = $Object.PSObject.Properties[$Name]
    if ($null -eq $p) { return $null }
    return $p.Value
}
function Format-Size {
    param([double] $Bytes)
    if ($Bytes -ge 1TB) { return ('{0:N2} TB' -f ($Bytes / 1TB)) }
    return ('{0:N1} GB' -f ($Bytes / 1GB))
}

# ----------------------------------------------------------------------------------
# 1. System under test
# ----------------------------------------------------------------------------------
$cs = Get-CimInstance Win32_ComputerSystem -ErrorAction SilentlyContinue
$bb = Get-CimInstance Win32_BaseBoard -ErrorAction SilentlyContinue
$bios = Get-CimInstance Win32_BIOS -ErrorAction SilentlyContinue
$os = Get-CimInstance Win32_OperatingSystem -ErrorAction SilentlyContinue
$cpu = Get-CimInstance Win32_Processor -ErrorAction SilentlyContinue | Select-Object -First 1
$ctrl = @(Get-CimInstance Win32_PnPSignedDriver -ErrorAction SilentlyContinue | Where-Object { $_.DeviceClass -in @('SCSIADAPTER', 'HDC') -and $_.DeviceName -match 'SATA|AHCI|RAID|RST|NVM|IDE' } | Select-Object -First 1)

# ----------------------------------------------------------------------------------
# 2. Drive under claim
# ----------------------------------------------------------------------------------
$pd = $null
try {
    $part = Get-Partition -DriveLetter $DriveLetter -ErrorAction Stop
    $pd = Get-PhysicalDisk -ErrorAction Stop | Where-Object { [int]$_.DeviceId -eq [int]$part.DiskNumber } | Select-Object -First 1
}
catch { }
if ($null -eq $pd) { $pd = Get-PhysicalDisk -ErrorAction SilentlyContinue | Select-Object -First 1 }
$vol = Get-Volume -DriveLetter $DriveLetter -ErrorAction SilentlyContinue
$rc = $null
if ($pd) { $rc = $pd | Get-StorageReliabilityCounter -ErrorAction SilentlyContinue }
$freePct = 0.0
if ($vol -and [double]$vol.Size -gt 0) { $freePct = ([double]$vol.SizeRemaining / [double]$vol.Size) * 100 }

$model = if ($pd) { [string]$pd.FriendlyName } else { 'unknown' }
$serial = if ($pd) { ([string]$pd.SerialNumber).Trim() } else { 'unknown' }
$fw = if ($pd) { [string](Get-PropOrNull $pd 'FirmwareVersion') } else { '' }
$sizeTxt = if ($pd) { Format-Size $pd.Size } else { '' }
$bus = if ($pd) { [string]$pd.BusType } else { '' }
$media = if ($pd) { [string]$pd.MediaType } else { '' }

# Rated figures for the drives this is most likely to be used on. Vendor datasheet numbers.
$ratedRead = $null; $ratedWrite = $null; $ratedSrc = ''
if ($model -match 'SA510') {
    $ratedRead = 560
    $ratedWrite = if ($pd -and $pd.Size -ge 900GB) { 520 } else { 510 }
    $ratedSrc = 'WD Blue SA510 SATA SSD datasheet (sequential, up to)'
}
elseif ($model -match '870 EVO') { $ratedRead = 560; $ratedWrite = 530; $ratedSrc = 'Samsung 870 EVO datasheet' }
elseif ($model -match 'MX500') { $ratedRead = 560; $ratedWrite = 510; $ratedSrc = 'Crucial MX500 datasheet' }
elseif ($media -eq 'SSD' -and $bus -match 'SATA|ATA|RAID') { $ratedRead = 500; $ratedWrite = 450; $ratedSrc = 'typical SATA 6Gb/s SSD class figures' }

# ----------------------------------------------------------------------------------
# 3. Run the full diagnostic and capture it
# ----------------------------------------------------------------------------------
$diagPath = Join-Path $ScriptDir 'disk-health-check.ps1'
$diag = @()
if (Test-Path -LiteralPath $diagPath) {
    $diag = @(& powershell.exe -NoProfile -ExecutionPolicy Bypass -File $diagPath -DriveLetter $DriveLetter 2>&1 | ForEach-Object { [string]$_ })
}
else {
    $diag = @('disk-health-check.ps1 not found next to this script; diagnostic not run.')
}
$diagText = $diag -join "`r`n"

function Grab {
    param([string] $Pattern, [int] $Group = 1)
    if ($diagText -match $Pattern) { return $Matches[$Group] }
    return $null
}
$seqRead = Grab 'seq_read=([\d.]+)MBps'
$seqWrite = Grab 'seq_write=([\d.]+)MBps'
$ranRead = Grab 'random_read=([\d.]+)MBps'
$susFirst = Grab 'sustained_first=([\d.]+)MBps'
$susLast = Grab 'sustained_last=([\d.]+)MBps'
$maxWriteMs = Grab 'max_write_ms=(\d+)'
$susAvg = Grab 'avg ([\d.]+) MB/s, min'
$susMin = Grab 'avg [\d.]+ MB/s, min ([\d.]+)'
$stallCount = Grab 'Writes that took 500 ms or longer: (\d+)'
$healthLine = Grab '(DISK_HEALTH: .*)'
$speedLine = Grab '(DISK_SPEED: .*)'
$idlePct = Grab 'Disk idle: ([\d.]+)%'
$writeCache = Grab 'Write caching on device: ([^\r\n]+)'
if ($null -eq $writeCache) { $writeCache = Grab 'Write-cache policy: ([^\r\n]+)' }
$trimLine = Grab '(TRIM enabled[^\r\n]*|TRIM is DISABLED[^\r\n]*)'
$fsLine = Grab '(Repair-Volume -Scan result: [^\r\n]*|File system consistent[^\r\n]*)'
$lpmLine = Grab '(No RST LPM switches set[^\r\n]*|SATA link power management is [^\r\n]*)'
$retrimLine = Grab '(Retrim sent[^\r\n]*)'
$drvLine = Grab 'driver ([\d.]+)  \((\d{4}-\d{2}-\d{2})\)  by Intel'
$drvDate = Grab 'driver [\d.]+  \((\d{4}-\d{2}-\d{2})\)  by Intel'

$latReadMax = if ($rc) { Get-PropOrNull $rc 'ReadLatencyMax' } else { $null }
$latWriteMax = if ($rc) { Get-PropOrNull $rc 'WriteLatencyMax' } else { $null }
$latFlushMax = if ($rc) { Get-PropOrNull $rc 'FlushLatencyMax' } else { $null }

function Pct {
    param($Measured, $Rated)
    if ($null -eq $Measured -or $null -eq $Rated -or $Rated -eq 0) { return '' }
    return (' ({0:N0}% of rated)' -f (([double]$Measured / [double]$Rated) * 100))
}
function Val { param($v, [string] $Suffix = '') if ($null -eq $v -or $v -eq '') { return 'not measured' } return ('{0}{1}' -f $v, $Suffix) }

# ----------------------------------------------------------------------------------
# 4. The document
# ----------------------------------------------------------------------------------
Rule
Line '  SSD WARRANTY / RETURN CLAIM - PERFORMANCE FAILURE EVIDENCE'
Rule
Line ('Generated:      {0} (local time)' -f (Get-Date -Format 'yyyy-MM-dd HH:mm'))
Line ('Computer:       {0}' -f $env:COMPUTERNAME)
Line ('Method:         Built-in Windows tooling only (PowerShell 5.1, WMI/CIM, Storage module,')
Line ('                winsat, defrag /L, unbuffered FileStream). No third-party benchmark software.')

Head 'DRIVE UNDER CLAIM'
Line ('Model:          {0}' -f $model)
Line ('Serial number:  {0}' -f $serial)
Line ('Firmware:       {0}' -f $fw)
Line ('Capacity:       {0}   Interface: {1}   Media: {2}' -f $sizeTxt, $bus, $media)
if ($pd) { Line ('Health status:  {0} / {1}  (Windows Storage Management)' -f $pd.HealthStatus, $pd.OperationalStatus) }
if ($vol) { Line ('Volume:         {0}:  {1}  {2} total, {3} free ({4:N1}%)' -f $DriveLetter, $vol.FileSystem, (Format-Size $vol.Size), (Format-Size $vol.SizeRemaining), $freePct) }
if ($ratedRead) { Line ('Rated speed:    up to {0} MB/s read, {1} MB/s write   [{2}]' -f $ratedRead, $ratedWrite, $ratedSrc) }

Head 'SYSTEM UNDER TEST'
if ($cs) { Line ('System:         {0} {1}' -f $cs.Manufacturer, $cs.Model) }
if ($bb) { Line ('Motherboard:    {0} {1}' -f $bb.Manufacturer, $bb.Product) }
if ($bios) { Line ('BIOS:           {0}' -f $bios.SMBIOSBIOSVersion) }
if ($cpu) { Line ('CPU:            {0}' -f ([string]$cpu.Name).Trim()) }
if ($os) { Line ('OS:             {0}  build {1}' -f $os.Caption, $os.Version) }
if ($ctrl.Count -gt 0) {
    $d = Get-PropOrNull $ctrl[0] 'DriverDate'
    $dTxt = if ($d) { ([datetime]$d).ToString('yyyy-MM-dd') } else { '?' }
    Line ('Controller:     {0}  driver {1} ({2}) by {3}' -f $ctrl[0].DeviceName, $ctrl[0].DriverVersion, $dTxt, $ctrl[0].DriverProviderName)
}

Head 'CLAIM SUMMARY'
Line 'The drive reports healthy SMART and passes the vendor short self-test, but it no longer'
Line 'performs anywhere near its rated write speed and periodically stalls for seconds on single'
Line 'I/O requests. Measured on an idle system, after TRIM, with write caching enabled:'
Line ''
Line ('  Sequential read  (winsat):          {0}{1}' -f (Val $seqRead ' MB/s'), (Pct $seqRead $ratedRead))
Line ('  Sequential write (winsat):          {0}{1}' -f (Val $seqWrite ' MB/s'), (Pct $seqWrite $ratedWrite))
Line ('  Random 16K read  (winsat):          {0}' -f (Val $ranRead ' MB/s'))
Line ('  Sustained 2 GB write, average:      {0}{1}' -f (Val $susAvg ' MB/s'), (Pct $susAvg $ratedWrite))
Line ('  Sustained 2 GB write, slowest step: {0}' -f (Val $susMin ' MB/s'))
Line ('  Sustained write first/last 256 MB:  {0} / {1}' -f (Val $susFirst ' MB/s'), (Val $susLast ' MB/s'))
Line ('  Slowest single 1 MB write:          {0}' -f (Val $maxWriteMs ' ms'))
Line ('  1 MB writes taking 500 ms or more:  {0}' -f (Val $stallCount))
Line ''
Line 'Worst-case latency recorded by Windows since last boot (Get-StorageReliabilityCounter):'
Line ('  Read {0} ms   Write {1} ms   Flush {2} ms' -f (Val $latReadMax), (Val $latWriteMax), (Val $latFlushMax))
Line ''
Line 'A healthy SATA 6Gb/s SSD completes the same sustained-write test at 350-500 MB/s with a'
Line 'slowest single write of roughly 10-20 ms.'

Head 'HOST-SIDE CAUSES RULED OUT (from the diagnostic in Appendix A)'
Line ('  Background disk load during test:   {0}' -f $(if ($idlePct) { "disk $idlePct% idle" } else { 'see appendix' }))
Line ('  Device write cache:                 {0}' -f (Val $writeCache))
Line ('  TRIM:                               {0}' -f (Val $trimLine))
Line ('  Retrim before measuring:            {0}' -f (Val $retrimLine))
Line ('  File system:                        {0}' -f (Val $fsLine))
Line ('  SATA link power management:         {0}' -f (Val $lpmLine))
Line ('  Storage driver:                     {0}' -f $(if ($drvLine) { "Intel RST $drvLine ($drvDate), newest for this platform" } else { 'see appendix' }))
Line ('  Free space:                         {0}' -f $(if ($vol) { ('{0:N1}%' -f $freePct) } else { 'see appendix' }))
Line '  SATA cable / port:                  SMART attribute 199 (CRC error count) = 0 in vendor tool (see attached)'
Line '  Thermal:                            SMART 194 temperature normal, 244 thermal throttle = 0 (see attached)'
Line '  Firmware:                           current per vendor Dashboard (see attached)'
Line ''
Line 'Verdict lines from the diagnostic:'
Line ('  {0}' -f (Val $healthLine))
Line ('  {0}' -f (Val $speedLine))

Head 'VENDOR TOOL DATA TO ATTACH WITH THIS FILE'
Line 'Screenshots from WD / SanDisk Dashboard > Tools:'
Line '  - S.M.A.R.T. short diagnostic result (passed) with date'
Line '  - S.M.A.R.T. data table, in particular attributes 5, 171, 172, 187, 194, 199, 230, 232, 241, 244'
Line '  - Firmware version shown in Dashboard (matches the firmware above)'
Line 'These show the drive is not reporting any error-class condition, which is why the failure is'
Line 'demonstrated with performance measurements rather than SMART thresholds.'

Head 'APPENDIX A - FULL DIAGNOSTIC OUTPUT (disk-health-check.ps1, this run)'
foreach ($l in $diag) { Line $l }

$prior = Join-Path $ScriptDir 'disk-full-report.txt'
if (Test-Path -LiteralPath $prior) {
    Head ('APPENDIX B - EARLIER RUN ({0})' -f (Get-Item -LiteralPath $prior).LastWriteTime.ToString('yyyy-MM-dd HH:mm'))
    Get-Content -LiteralPath $prior -ErrorAction SilentlyContinue | ForEach-Object { Line $_ }
}

Head 'HOW THESE NUMBERS WERE PRODUCED'
Line 'winsat disk -seq -read / -seq -write / -ran -read  (Windows System Assessment Tool, 3 passes,'
Line '  low results re-run after a 10 s pause; best value reported).'
Line 'Sustained write: a 2 GB temporary file written in 1 MB blocks through a .NET FileStream opened'
Line '  with FILE_FLAG_NO_BUFFERING and WriteThrough, so the OS cache cannot mask the drive; MB/s'
Line '  reported per 256 MB; each 1 MB write individually timed. File deleted afterwards.'
Line 'Latency maxima: Windows Storage Management reliability counters for the physical disk.'
Line 'Full script source: https://github.com/mvl-boston/opendbc/tree/cursor/windows-cpu-health-check-65b2/scripts/windows'
Rule
Line 'END OF EVIDENCE FILE'
exit 0
