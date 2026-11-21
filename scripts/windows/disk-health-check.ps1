#Requires -Version 5.1
param(
    [string] $DriveLetter = $env:SystemDrive.TrimEnd(':'),
    [switch] $SkipVolumeScan,
    [switch] $SkipSpeedTest,
    [switch] $SkipRetrim
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Continue'

function Write-Section {
    param([string] $Title)
    Write-Host ''
    Write-Host ('========== {0} ==========' -f $Title) -ForegroundColor Cyan
}
function Write-Warn { param([string] $Message) Write-Host ('WARN: {0}' -f $Message) -ForegroundColor Yellow }
function Write-Bad { param([string] $Message) Write-Host ('FAIL: {0}' -f $Message) -ForegroundColor Red }
function Write-Ok { param([string] $Message) Write-Host ('OK: {0}' -f $Message) -ForegroundColor Green }

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
    if ($Bytes -ge 1GB) { return ('{0:N1} GB' -f ($Bytes / 1GB)) }
    return ('{0:N0} MB' -f ($Bytes / 1MB))
}

$DriveLetter = $DriveLetter.TrimEnd(':').ToUpper()
$healthVerdict = 'PASS'
$healthReasons = New-Object System.Collections.Generic.List[string]
function Add-HealthIssue {
    param([string] $Level, [string] $Reason)
    $script:healthReasons.Add($Reason)
    if ($Level -eq 'FAIL') { $script:healthVerdict = 'FAIL' }
    elseif ($Level -eq 'WARN' -and $script:healthVerdict -ne 'FAIL') { $script:healthVerdict = 'WARN' }
}

Write-Section ('Drive inventory (testing {0}:)' -f $DriveLetter)
$physicalDisks = @(Get-PhysicalDisk -ErrorAction SilentlyContinue)
if ($physicalDisks.Count -eq 0) {
    Write-Bad 'Get-PhysicalDisk returned nothing. Run as Administrator.'
    Write-Host 'DISK_HEALTH: FAIL reason=no_disk_info'
    exit 2
}

$targetDiskNumber = $null
try {
    $part = Get-Partition -DriveLetter $DriveLetter -ErrorAction Stop
    $targetDiskNumber = $part.DiskNumber
}
catch {
    Write-Warn ('Could not map {0}: to a disk: {1}' -f $DriveLetter, $_.Exception.Message)
}

$targetPd = $null
foreach ($pd in $physicalDisks) {
    $isTarget = ($null -ne $targetDiskNumber -and [string]$pd.DeviceId -eq [string]$targetDiskNumber)
    if ($isTarget) { $targetPd = $pd }
    $marker = if ($isTarget) { '  <== ' + $DriveLetter + ':' } else { '' }
    Write-Host ('Disk {0}: {1}' -f $pd.DeviceId, $pd.FriendlyName) -NoNewline
    Write-Host $marker -ForegroundColor Cyan
    Write-Host ('   Type: {0}   Bus: {1}   Size: {2}   Health: {3}   Status: {4}' -f $pd.MediaType, $pd.BusType, (Format-Size $pd.Size), $pd.HealthStatus, ($pd.OperationalStatus -join ','))
    $fw = Get-PropOrNull $pd 'FirmwareVersion'
    $sn = Get-PropOrNull $pd 'SerialNumber'
    if ($fw -or $sn) { Write-Host ('   Firmware: {0}   Serial: {1}' -f $fw, $sn) }
    if ($pd.HealthStatus -ne 'Healthy') {
        Add-HealthIssue 'FAIL' ('disk{0}_health={1}' -f $pd.DeviceId, $pd.HealthStatus)
    }
}

if (-not $targetPd) {
    $targetPd = $physicalDisks | Select-Object -First 1
    Write-Warn ('Using Disk {0} as target.' -f $targetPd.DeviceId)
}
$mediaType = [string]$targetPd.MediaType
$busType = [string]$targetPd.BusType
Write-Host ''
Write-Host ('Target: Disk {0} = {1} ({2}, {3})' -f $targetPd.DeviceId, $targetPd.FriendlyName, $mediaType, $busType)

Write-Section 'Volume / free space / TRIM'
try {
    $vol = Get-Volume -DriveLetter $DriveLetter -ErrorAction Stop
    $freePct = 0
    if ($vol.Size -gt 0) { $freePct = [math]::Round(($vol.SizeRemaining / $vol.Size) * 100, 1) }
    Write-Host ('{0}:  {1}  {2} total, {3} free ({4}%)  Health: {5}' -f $DriveLetter, $vol.FileSystem, (Format-Size $vol.Size), (Format-Size $vol.SizeRemaining), $freePct, $vol.HealthStatus)
    if ($vol.HealthStatus -ne 'Healthy') { Add-HealthIssue 'FAIL' ('volume_health=' + $vol.HealthStatus) }
    if ($freePct -lt 10) {
        Write-Warn 'Less than 10% free. SSDs slow down and Windows struggles when nearly full.'
        Add-HealthIssue 'WARN' ('low_free_space=' + $freePct + '%')
    }
}
catch { Write-Warn ('Get-Volume failed: {0}' -f $_.Exception.Message) }

$dirty = (& fsutil dirty query ("{0}:" -f $DriveLetter) 2>&1 | Out-String).Trim()
Write-Host ('Dirty bit: {0}' -f $dirty)
if ($dirty -match 'is Dirty') {
    Write-Warn 'Volume is marked dirty - Windows wants to run chkdsk at next boot.'
    Add-HealthIssue 'WARN' 'volume_dirty_bit_set'
}

if ($mediaType -eq 'SSD') {
    $trim = (& fsutil behavior query DisableDeleteNotify 2>&1 | Out-String)
    if ($trim -match 'NTFS DisableDeleteNotify = (\d)') {
        if ($Matches[1] -eq '0') { Write-Ok 'TRIM enabled (NTFS DisableDeleteNotify = 0).' }
        else {
            Write-Warn 'TRIM is DISABLED on NTFS. SSD will slow down over time. Enable: fsutil behavior set DisableDeleteNotify 0'
            Add-HealthIssue 'WARN' 'trim_disabled'
        }
    }
    elseif ($trim -match 'DisableDeleteNotify = (\d)') {
        if ($Matches[1] -eq '0') { Write-Ok 'TRIM enabled.' } else { Write-Warn 'TRIM disabled.'; Add-HealthIssue 'WARN' 'trim_disabled' }
    }
}

Write-Section 'SMART / reliability counters (built-in Windows)'
$rc = $null
try { $rc = $targetPd | Get-StorageReliabilityCounter -ErrorAction Stop } catch { Write-Warn ('Reliability counters unavailable: {0}' -f $_.Exception.Message) }
if ($rc) {
    $fields = [ordered]@{
        'Temperature (C)'          = Get-PropOrNull $rc 'Temperature'
        'Temperature max (C)'      = Get-PropOrNull $rc 'TemperatureMax'
        'Wear (% used)'            = Get-PropOrNull $rc 'Wear'
        'Power-on hours'           = Get-PropOrNull $rc 'PowerOnHours'
        'Start/stop cycles'        = Get-PropOrNull $rc 'StartStopCycleCount'
        'Read errors total'        = Get-PropOrNull $rc 'ReadErrorsTotal'
        'Read errors corrected'    = Get-PropOrNull $rc 'ReadErrorsCorrected'
        'Read errors UNCORRECTED'  = Get-PropOrNull $rc 'ReadErrorsUncorrected'
        'Write errors total'       = Get-PropOrNull $rc 'WriteErrorsTotal'
        'Write errors corrected'   = Get-PropOrNull $rc 'WriteErrorsCorrected'
        'Write errors UNCORRECTED' = Get-PropOrNull $rc 'WriteErrorsUncorrected'
        'Read latency max (ms)'    = Get-PropOrNull $rc 'ReadLatencyMax'
        'Write latency max (ms)'   = Get-PropOrNull $rc 'WriteLatencyMax'
        'Flush latency max (ms)'   = Get-PropOrNull $rc 'FlushLatencyMax'
    }
    foreach ($k in $fields.Keys) {
        $v = $fields[$k]
        if ($null -ne $v) { Write-Host ('  {0,-26} {1}' -f $k, $v) }
    }
    $uncRead = Get-PropOrNull $rc 'ReadErrorsUncorrected'
    $uncWrite = Get-PropOrNull $rc 'WriteErrorsUncorrected'
    $wear = Get-PropOrNull $rc 'Wear'
    $temp = Get-PropOrNull $rc 'Temperature'
    if ($uncRead -gt 0 -or $uncWrite -gt 0) {
        Write-Bad 'Uncorrectable read/write errors reported by the drive. Back up now; drive is failing.'
        Add-HealthIssue 'FAIL' ('uncorrected_errors_read={0}_write={1}' -f $uncRead, $uncWrite)
    }
    if ($null -ne $wear) {
        if ($wear -ge 90) { Write-Bad ('SSD wear {0}% - near end of rated life.' -f $wear); Add-HealthIssue 'FAIL' ('wear=' + $wear + '%') }
        elseif ($wear -ge 70) { Write-Warn ('SSD wear {0}% - plan replacement.' -f $wear); Add-HealthIssue 'WARN' ('wear=' + $wear + '%') }
        else { Write-Ok ('SSD wear {0}% used.' -f $wear) }
    }
    if ($null -ne $temp -and $temp -ge 65) {
        Write-Warn ('Drive temperature {0} C is high.' -f $temp)
        Add-HealthIssue 'WARN' ('drive_temp=' + $temp)
    }
    if (($null -eq $temp -or $temp -eq 0) -and $busType -eq 'RAID') {
        Write-Host 'Note: temperature/wear read as 0 because the Intel RST "RAID" controller mode hides SMART from Windows.'
        Write-Host '      Use the drive vendor tool (e.g. WD Dashboard) for true SMART and firmware updates.'
    }
    $latMax = 0
    foreach ($n in 'ReadLatencyMax', 'WriteLatencyMax', 'FlushLatencyMax') {
        $v = Get-PropOrNull $rc $n
        if ($null -ne $v -and $v -gt $latMax) { $latMax = [int]$v }
    }
    if ($latMax -ge 1000) {
        Write-Warn ('Worst-case I/O latency since boot: {0} ms. Single I/Os took over a second - that is a visible "freeze".' -f $latMax)
        Write-Host '      Stalls like this (not raw MB/s) are what make a PC feel slow. Causes: SSD firmware bugs, SLC cache'
        Write-Host '      exhaustion on DRAM-less SSDs, old storage driver, bad SATA cable, or Windows write-cache flushing.'
        Add-HealthIssue 'WARN' ('io_stall_max_ms=' + $latMax)
    }
    elseif ($latMax -gt 0) { Write-Ok ('Worst-case I/O latency since boot: {0} ms.' -f $latMax) }
}

Write-Section 'Storage controller / driver / cache settings'
try {
    $ctrls = @(Get-CimInstance Win32_PnPSignedDriver -ErrorAction Stop | Where-Object { $_.DeviceClass -in @('SCSIADAPTER', 'HDC') -and $_.DeviceName -match 'SATA|AHCI|RAID|RST|NVM|Storage|IDE' })
    foreach ($c in $ctrls) {
        $d = Get-PropOrNull $c 'DriverDate'
        $dTxt = if ($d) { ([datetime]$d).ToString('yyyy-MM-dd') } else { '?' }
        Write-Host ('  {0}' -f $c.DeviceName)
        Write-Host ('     driver {0}  ({1})  by {2}' -f $c.DriverVersion, $dTxt, $c.DriverProviderName)
        if ($c.DriverProviderName -match 'Intel' -and $c.DriverVersion -match '^(\d+)\.(\d+)\.') {
            $rstMajor = [int]$Matches[1]; $rstMinor = [int]$Matches[2]
            if ($rstMajor -lt 14 -or ($rstMajor -eq 14 -and $rstMinor -lt 8)) {
                Write-Warn '     Intel RST older than 14.8. For 8/9-series (Haswell) chipsets the newest usable lines are 14.8.x,'
                Write-Host '       15.9.x and 15.44.x (all 2016-2018). Updating from 12.x/13.x is reasonable; nothing newer exists.'
            }
            else {
                Write-Host '       (this is already the newest RST line available for Haswell-era chipsets; no update to chase)'
            }
        }
    }
    if ($ctrls.Count -eq 0) { Write-Host '  (no SATA/RAID/NVMe controller driver entries matched)' }
}
catch { Write-Warn ('Driver query failed: {0}' -f $_.Exception.Message) }

# SATA link power management (LPM / HIPM / DIPM). Aggressive LPM is a classic cause of random
# multi-second SSD stalls: the link sleeps, the drive takes too long to wake, I/O hangs.
try {
    $lpmFound = $false
    $lpmOn = $false
    # Intel RST keeps its own per-port switches (Controller0Phy0LPM / LPMDSTATE / DIPM, 1 = on).
    # storahci's "NoLPM" is a *list of device IDs excluded from LPM*, not a switch, so it is ignored.
    $svcKeys = @(Get-ChildItem 'HKLM:\SYSTEM\CurrentControlSet\Services' -ErrorAction Stop | Where-Object { $_.PSChildName -match '^iaStor' })
    foreach ($sk in $svcKeys) {
        $svcProps = Get-ItemProperty -LiteralPath $sk.PSPath -ErrorAction SilentlyContinue
        $imgPath = Get-PropOrNull $svcProps 'ImagePath'
        $startV = Get-PropOrNull $svcProps 'Start'
        $startTxt = switch ($startV) { 0 { 'boot' } 1 { 'system' } 2 { 'auto' } 3 { 'manual' } 4 { 'disabled' } default { "$startV" } }
        Write-Host ('  RST service {0}: start={1}  {2}' -f $sk.PSChildName, $startTxt, $imgPath)
        $devKey = 'HKLM:\SYSTEM\CurrentControlSet\Services\{0}\Parameters\Device' -f $sk.PSChildName
        if (-not (Test-Path -LiteralPath $devKey)) { Write-Host '     (no Parameters\Device key - driver defaults in effect)'; continue }
        $dp = Get-ItemProperty -LiteralPath $devKey -ErrorAction SilentlyContinue
        if ($null -eq $dp) { continue }
        $numProps = @($dp.PSObject.Properties | Where-Object { $_.Name -notmatch '^PS' -and $_.Value -is [int] } | Select-Object -First 24)
        if ($numProps.Count -eq 0) { Write-Host '     (no numeric driver parameters set - driver defaults in effect)'; continue }
        foreach ($p in $numProps) {
            $flag = ''
            if ($p.Name -match '^Controller\d+Phy\d+(LPM|DIPM)$') {
                $lpmFound = $true
                if ($p.Value -ne 0) { $lpmOn = $true; $flag = '  <-- LPM on' }
            }
            Write-Host ('     {0,-32} = {1}{2}' -f $p.Name, $p.Value, $flag)
        }
    }
    if ($busType -eq 'RAID') {
        Write-Host '  (Bus mode RAID: the Windows power-plan AHCI LPM setting does not apply; the RST driver decides.'
        Write-Host '   RST default on desktop boards is LPM off; it is only on if the values above say so.)'
    }
    else {
        $sub = '0012ee47-9041-4b5d-9b77-535fba8b1442'; $set = '0b2d69d7-a2a1-449c-9680-f91c70521c60'
        & powercfg -attributes $sub $set -ATTRIB_HIDE 2>&1 | Out-Null
        $pc = & powercfg /q SCHEME_CURRENT $sub $set 2>&1 | Out-String
        $acIdx = $null
        if ($pc -match 'Current AC Power Setting Index:\s*0x([0-9a-fA-F]+)') { $acIdx = [Convert]::ToInt32($Matches[1], 16) }
        else {
            $schemeOut = & powercfg /getactivescheme 2>&1 | Out-String
            if ($schemeOut -match '([0-9a-fA-F-]{36})') {
                $k = 'HKLM:\SYSTEM\CurrentControlSet\Control\Power\User\PowerSchemes\{0}\{1}\{2}' -f $Matches[1], $sub, $set
                $v = Get-PropOrNull (Get-ItemProperty -LiteralPath $k -ErrorAction SilentlyContinue) 'ACSettingIndex'
                if ($null -ne $v) { $acIdx = [int]$v }
            }
        }
        if ($null -ne $acIdx) {
            $names = @{ 0 = 'Active (LPM off)'; 1 = 'HIPM'; 2 = 'HIPM+DIPM'; 3 = 'DIPM'; 4 = 'Lowest' }
            $acTxt = if ($names.ContainsKey($acIdx)) { $names[$acIdx] } else { "index $acIdx" }
            Write-Host ('  Power plan "AHCI Link Power Management - HIPM/DIPM" (plugged in): {0}' -f $acTxt)
            $lpmFound = $true
            if ($acIdx -ne 0) { $lpmOn = $true }
        }
        else { Write-Host '  Power plan AHCI LPM setting: not set explicitly (Windows default = HIPM on Balanced).' }
    }
    if ($lpmFound -and $lpmOn) {
        Write-Warn '  SATA link power management is ON. On SATA SSDs this is a known cause of random freezes of'
        Write-Host '       1-10 seconds (the link naps, the drive is slow to wake, Windows waits). It saves almost no'
        Write-Host '       power on a desktop. Try turning it off and re-run this check:'
        Write-Host '         Admin cmd:  powercfg /setacvalueindex SCHEME_CURRENT 0012ee47-9041-4b5d-9b77-535fba8b1442 0b2d69d7-a2a1-449c-9680-f91c70521c60 0'
        Write-Host '                     powercfg /setactive SCHEME_CURRENT'
        Write-Host '         Intel RST:  Intel Optane Memory and Storage Management / RST app > Performance tab >'
        Write-Host '                     Link Power Management = Disabled  (or set the *LPM registry values above to 0, then reboot)'
        Add-HealthIssue 'WARN' 'sata_lpm_enabled'
    }
    elseif ($lpmFound) { Write-Ok '  SATA link power management is off (no LPM-induced stalls).' }
    elseif ($busType -eq 'RAID') { Write-Ok '  No RST LPM switches set; driver default applies (LPM off on desktop boards). Not a stall cause here.' }
    else { Write-Host '  Link power management: no LPM settings found to read.' }
}
catch { Write-Host ('  Link power management: could not read ({0})' -f $_.Exception.Message) }

try {
    $dd = Get-CimInstance Win32_DiskDrive -ErrorAction Stop | Where-Object { $_.Index -eq [int]$targetPd.DeviceId } | Select-Object -First 1
    if ($dd) {
        $regPath = 'HKLM:\SYSTEM\CurrentControlSet\Enum\{0}\Device Parameters\Disk' -f $dd.PNPDeviceID
        if (Test-Path -LiteralPath $regPath) {
            $rp = Get-ItemProperty -LiteralPath $regPath -ErrorAction SilentlyContinue
            $uwc = Get-PropOrNull $rp 'UserWriteCacheSetting'
            $cpp = Get-PropOrNull $rp 'CacheIsPowerProtected'
            $uwcTxt = if ($null -eq $uwc) { 'default (enabled)' } elseif ($uwc -eq 0) { 'DISABLED' } else { 'enabled' }
            Write-Host ('  Write caching on device: {0}' -f $uwcTxt)
            if ($uwc -eq 0) {
                Write-Warn '  Write caching is turned off - this alone can cut SSD write speed to a fraction.'
                Write-Host '  Device Manager > Disk drives > drive > Properties > Policies > Enable write caching.'
                Add-HealthIssue 'WARN' 'write_cache_disabled'
            }
            Write-Host ('  "Turn off Windows write-cache buffer flushing": {0}' -f $(if ($cpp -eq 1) { 'ON (risky without UPS)' } else { 'off (normal)' }))
        }
        else { Write-Host '  Write-cache policy: default (enabled)' }
    }
}
catch { Write-Host ('  Write-cache policy: could not read ({0})' -f $_.Exception.Message) }

try {
    $defragTask = Get-ScheduledTask -TaskPath '\Microsoft\Windows\Defrag\' -TaskName 'ScheduledDefrag' -ErrorAction Stop
    $info = $defragTask | Get-ScheduledTaskInfo -ErrorAction SilentlyContinue
    $lastRun = Get-PropOrNull $info 'LastRunTime'
    Write-Host ('  Optimize Drives (TRIM) task: {0}, last run {1}' -f $defragTask.State, $lastRun)
    if ($defragTask.State -eq 'Disabled') {
        Write-Warn '  Scheduled optimization is disabled; SSD never gets periodic retrim. Re-enable in "Defragment and Optimize Drives".'
        Add-HealthIssue 'WARN' 'optimize_task_disabled'
    }
}
catch { }

Write-Section 'Background disk load right now (10s sample, before speed test)'
try {
    $cs = Get-Counter -Counter @('\PhysicalDisk(_Total)\% Idle Time', '\PhysicalDisk(_Total)\Avg. Disk sec/Transfer', '\PhysicalDisk(_Total)\Disk Transfers/sec', '\PhysicalDisk(_Total)\Disk Bytes/sec') -SampleInterval 2 -MaxSamples 5 -ErrorAction Stop
    $idle = [math]::Round((($cs | ForEach-Object { ($_.CounterSamples | Where-Object { $_.Path -like '*Idle Time' }).CookedValue }) | Measure-Object -Average).Average, 1)
    $lat = [math]::Round(((($cs | ForEach-Object { ($_.CounterSamples | Where-Object { $_.Path -like '*sec/Transfer' }).CookedValue }) | Measure-Object -Average).Average) * 1000, 1)
    $tps = [math]::Round((($cs | ForEach-Object { ($_.CounterSamples | Where-Object { $_.Path -like '*Transfers/sec' }).CookedValue }) | Measure-Object -Average).Average, 0)
    $bps = [math]::Round(((($cs | ForEach-Object { ($_.CounterSamples | Where-Object { $_.Path -like '*Bytes/sec' }).CookedValue }) | Measure-Object -Average).Average) / 1MB, 1)
    Write-Host ('  Disk idle: {0}%   avg latency: {1} ms   {2} IO/s   {3} MB/s' -f $idle, $lat, $tps, $bps)
    if ($idle -lt 80) {
        Write-Warn '  Something is already using the disk heavily. Speed results below will read low.'
        Add-HealthIssue 'WARN' ('background_disk_busy_idle=' + $idle + '%')
    }
    if ($lat -gt 20) { Write-Warn ('  Average I/O latency {0} ms at idle is high for an SSD (healthy: under 1-5 ms).' -f $lat) }
    $procs = @(Get-Counter -Counter '\Process(*)\IO Data Bytes/sec' -ErrorAction Stop).CounterSamples | Where-Object { $_.InstanceName -notin @('_total', 'idle') -and $_.CookedValue -gt 100KB } | Sort-Object CookedValue -Descending | Select-Object -First 6
    if ($procs) {
        Write-Host '  Top I/O processes:'
        foreach ($p in $procs) { Write-Host ('     {0,-28} {1,8:N2} MB/s' -f $p.InstanceName, ($p.CookedValue / 1MB)) }
        if (($procs | Where-Object { $_.InstanceName -match 'OneDrive|MsMpEng|SearchIndexer|CompatTelRunner|TiWorker' })) {
            Write-Host '  (OneDrive sync, Defender scan, Search indexing, or Windows Update servicing is active)'
        }
    }
}
catch { Write-Host ('  Could not sample disk counters: {0}' -f $_.Exception.Message) }

try {
    $fp = @(Get-CimInstance -Namespace root\wmi -ClassName MSStorageDriver_FailurePredictStatus -ErrorAction Stop)
    foreach ($f in $fp) {
        $pred = Get-PropOrNull $f 'PredictFailure'
        $inst = Get-PropOrNull $f 'InstanceName'
        if ($pred) {
            Write-Bad ('SMART PREDICTS FAILURE: {0}' -f $inst)
            Add-HealthIssue 'FAIL' 'smart_predict_failure'
        }
        else { Write-Ok ('SMART predict-failure flag clear: {0}' -f ($inst -replace '\\\\.*$', '')) }
    }
}
catch { Write-Host 'SMART predict-failure status not exposed by this driver (normal for NVMe).' }

Write-Section 'Disk error events (System log, last 30 days)'
$since = (Get-Date).AddDays(-30)
$eventChecks = @(
    @{ Label = 'disk: bad block (7)';                 Provider = 'disk';      Ids = @(7);        Level = 'FAIL' },
    @{ Label = 'disk: controller error (11)';         Provider = 'disk';      Ids = @(11);       Level = 'WARN' },
    @{ Label = 'disk: paging I/O error (51)';         Provider = 'disk';      Ids = @(51);       Level = 'WARN' },
    @{ Label = 'disk: I/O retried (153)';             Provider = 'disk';      Ids = @(153);      Level = 'WARN' },
    @{ Label = 'Ntfs: corruption found (55/137/140)'; Provider = 'Ntfs';      Ids = @(55,137,140); Level = 'FAIL' },
    @{ Label = 'Ntfs: self-repair completed (130)';   Provider = 'Ntfs';      Ids = @(130);      Level = 'WARN' },
    @{ Label = 'storahci: device reset (129)';        Provider = 'storahci';  Ids = @(129);      Level = 'WARN' },
    @{ Label = 'stornvme: device reset (129)';        Provider = 'stornvme';  Ids = @(129);      Level = 'WARN' },
    @{ Label = 'iaStorA/iaStorAC: reset (129)';       Provider = @('iaStorA','iaStorAC','iaStorAVC'); Ids = @(129); Level = 'WARN' }
)
$totalDiskEvents = 0
foreach ($chk in $eventChecks) {
    $count = 0
    $last = $null
    $evts = @()
    try {
        $evts = @(Get-WinEvent -FilterHashtable @{ LogName = 'System'; ProviderName = $chk.Provider; Id = $chk.Ids; StartTime = $since } -ErrorAction Stop)
        $count = $evts.Count
        if ($count -gt 0) { $last = ($evts | Sort-Object TimeCreated -Descending | Select-Object -First 1).TimeCreated }
    }
    catch { $count = 0 }
    if ($count -gt 0) {
        $totalDiskEvents += $count
        Write-Host ('  {0,-42} {1,4}   last: {2}' -f $chk.Label, $count, $last) -ForegroundColor Yellow
        Add-HealthIssue $chk.Level ('events_' + ($chk.Label -replace '[^A-Za-z0-9]', '_') + '=' + $count)
        foreach ($e in ($evts | Sort-Object TimeCreated -Descending | Select-Object -First 3)) {
            $msg = [string](Get-PropOrNull $e 'Message')
            $msg = ($msg -replace '\s+', ' ').Trim()
            if ($msg.Length -gt 220) { $msg = $msg.Substring(0, 220) + '...' }
            Write-Host ('       [{0}] Id {1}: {2}' -f $e.TimeCreated, $e.Id, $msg)
        }
    }
    else {
        Write-Host ('  {0,-42}    0' -f $chk.Label)
    }
}
if ($totalDiskEvents -eq 0) { Write-Ok 'No disk/NTFS/controller error events in the last 30 days.' }
else { Write-Warn ('{0} disk-related error events in 30 days. Repeated 129/153 events often mean a failing drive or bad SATA cable.' -f $totalDiskEvents) }

if (-not $SkipVolumeScan) {
    Write-Section ('Online file-system scan of {0}: (read-only, 1-3 min on SSD)' -f $DriveLetter)
    try {
        $scan = Repair-Volume -DriveLetter $DriveLetter -Scan -ErrorAction Stop
        Write-Host ('Repair-Volume -Scan result: {0}' -f $scan)
        if ("$scan" -match 'NoErrorsFound') { Write-Ok 'File system consistent.' }
        else {
            Write-Warn ('File-system issues flagged. Run in Admin cmd:  chkdsk {0}: /f   (reboot required)' -f $DriveLetter)
            Add-HealthIssue 'WARN' ('fs_scan=' + $scan)
        }
    }
    catch {
        Write-Host ('Repair-Volume scan not available ({0}). Falling back to chkdsk /scan (online, read-only)...' -f ($_.Exception.Message -split "`r?`n")[0])
        $ck = & chkdsk ("{0}:" -f $DriveLetter) /scan 2>&1 | Out-String
        $ckLines = ($ck -split "`r?`n" | Where-Object { $_ -match 'found no problems|found problems|corrupt|Errors found|Windows has made corrections|total|free' } | Select-Object -First 6)
        foreach ($l in $ckLines) { Write-Host ('  ' + $l.Trim()) }
        if ($ck -match 'found no problems') { Write-Ok 'File system consistent (chkdsk /scan).' }
        elseif ($ck -match 'found problems|corrupt|Errors found') {
            Write-Warn ('File-system problems found. Run in Admin cmd:  chkdsk {0}: /f   (reboot required)' -f $DriveLetter)
            Add-HealthIssue 'WARN' 'fs_scan_found_problems'
        }
        else { Write-Warn 'chkdsk /scan result unclear - see lines above.' }
    }
}

Write-Host ''
$reasonTxt = if ($healthReasons.Count -gt 0) { $healthReasons -join ',' } else { 'none' }
Write-Host ('DISK_HEALTH: {0} disk={1} type={2} bus={3} reasons={4}' -f $healthVerdict, $targetPd.FriendlyName, $mediaType, $busType, $reasonTxt)

if ($SkipSpeedTest) { exit 0 }

if ($mediaType -eq 'SSD' -and -not $SkipRetrim) {
    Write-Section 'TRIM now + 60s idle so the SSD can garbage-collect before measuring'
    try {
        $dOut = & defrag ("{0}:" -f $DriveLetter) /L 2>&1 | Out-String
        $dLine = ($dOut -split "`r?`n" | Where-Object { $_ -match 'Retrim|Trim|complete|Error|error|not supported' } | Select-Object -First 3)
        foreach ($l in $dLine) { Write-Host ('  ' + $l.Trim()) }
        if ($dOut -match 'Error|error|not supported') { Write-Warn '  Retrim did not run. In RAID mode this can mean TRIM is not reaching the drive at all.' }
        else { Write-Ok '  Retrim sent. Waiting 60 s with the disk idle...' }
        Start-Sleep -Seconds 60
    }
    catch { Write-Warn ('  defrag /L failed: {0}' -f $_.Exception.Message) }
}

Write-Section ('Speed test on {0}: (Windows built-in winsat)' -f $DriveLetter)
$seqRead = $null; $seqWrite = $null; $ranRead = $null
$winsat = Join-Path $env:SystemRoot 'System32\WinSAT.exe'
if (Test-Path -LiteralPath $winsat) {
    $tests = @(
        @{ Args = @('disk', '-drive', $DriveLetter, '-seq', '-read', '-count', '3');  Name = 'Sequential read' },
        @{ Args = @('disk', '-drive', $DriveLetter, '-seq', '-write', '-count', '3'); Name = 'Sequential write' },
        @{ Args = @('disk', '-drive', $DriveLetter, '-ran', '-read', '-count', '3');  Name = 'Random 16K read' }
    )
    function Invoke-WinsatMbps {
        param([string[]] $WsArgs)
        $out = & $winsat @WsArgs 2>&1 | Out-String
        foreach ($line in ($out -split "`r?`n")) {
            if ($line -match '>\s*Disk\s+(Sequential|Random)\s+[\d.]+\s+(Read|Write)\s+([\d.]+)\s+MB/s') { return [double]$Matches[3] }
        }
        $snippet = ($out -split "`r?`n" | Where-Object { $_ -match 'Error|error|denied|battery|not supported' } | Select-Object -First 2) -join ' | '
        if ($snippet) { Write-Warn ('winsat: {0}' -f $snippet) }
        return $null
    }
    $retryBelow = @{ 'Sequential read' = 350; 'Sequential write' = 250; 'Random 16K read' = 80 }
    foreach ($t in $tests) {
        Write-Host ('Running winsat {0}...' -f $t.Name)
        $mbps = Invoke-WinsatMbps -WsArgs $t.Args
        if ($null -eq $mbps) { Write-Warn ('{0}: could not parse result.' -f $t.Name); continue }
        if ($mbps -lt $retryBelow[$t.Name]) {
            Write-Host ('  {0,-18} {1,8:N1} MB/s  (low - retrying after 10s pause to rule out background activity)' -f $t.Name, $mbps)
            Start-Sleep -Seconds 10
            $second = Invoke-WinsatMbps -WsArgs $t.Args
            if ($null -ne $second) {
                Write-Host ('  {0,-18} {1,8:N1} MB/s  (retry)' -f $t.Name, $second)
                if ($second -gt $mbps) { $mbps = $second }
            }
        }
        Write-Host ('  {0,-18} {1,8:N1} MB/s  <-- best' -f $t.Name, $mbps)
        switch ($t.Name) {
            'Sequential read' { $seqRead = $mbps }
            'Sequential write' { $seqWrite = $mbps }
            'Random 16K read' { $ranRead = $mbps }
        }
    }
}
else { Write-Warn 'winsat.exe not found.' }

if ($null -eq $seqRead) {
    Write-Host 'Falling back to PowerShell file test (1 GB, unbuffered)...'
    try {
        $testFile = Join-Path ("{0}:\" -f $DriveLetter) ('disk-speed-test-{0}.tmp' -f ([guid]::NewGuid().ToString('N')))
        $bufSize = 1MB
        $total = 1GB
        $buf = New-Object byte[] $bufSize
        (New-Object Random).NextBytes($buf)
        $noBuffering = [System.IO.FileOptions]0x20000000
        $sw = [System.Diagnostics.Stopwatch]::StartNew()
        $fs = New-Object System.IO.FileStream($testFile, [System.IO.FileMode]::Create, [System.IO.FileAccess]::Write, [System.IO.FileShare]::None, $bufSize, ($noBuffering -bor [System.IO.FileOptions]::WriteThrough))
        for ($w = 0; $w -lt $total; $w += $bufSize) { $fs.Write($buf, 0, $bufSize) }
        $fs.Flush($true); $fs.Dispose()
        $sw.Stop()
        $seqWrite = [math]::Round(($total / 1MB) / $sw.Elapsed.TotalSeconds, 1)
        $sw = [System.Diagnostics.Stopwatch]::StartNew()
        $fs = New-Object System.IO.FileStream($testFile, [System.IO.FileMode]::Open, [System.IO.FileAccess]::Read, [System.IO.FileShare]::None, $bufSize, $noBuffering)
        while ($fs.Read($buf, 0, $bufSize) -gt 0) { }
        $fs.Dispose()
        $sw.Stop()
        $seqRead = [math]::Round(($total / 1MB) / $sw.Elapsed.TotalSeconds, 1)
        Remove-Item -LiteralPath $testFile -Force -ErrorAction SilentlyContinue
        Write-Host ('  Sequential write    {0,8:N1} MB/s' -f $seqWrite)
        Write-Host ('  Sequential read     {0,8:N1} MB/s' -f $seqRead)
    }
    catch { Write-Warn ('Fallback speed test failed: {0}' -f $_.Exception.Message) }
}

# Sustained write profile. winsat reports one number; this shows the *shape*: a healthy drive is flat,
# a DRAM-less SSD drops once its SLC cache fills, a sick drive is slow everywhere and stalls mid-write.
$susFirst = $null; $susLast = $null; $susMaxMs = $null; $susStalls = 0
$volForTest = Get-Volume -DriveLetter $DriveLetter -ErrorAction SilentlyContinue
if ($null -ne $volForTest -and $volForTest.SizeRemaining -gt 6GB) {
    Write-Section ('Sustained write profile on {0}: (2 GB, unbuffered, 256 MB steps)' -f $DriveLetter)
    $testFile = Join-Path ("{0}:\" -f $DriveLetter) ('disk-sustained-{0}.tmp' -f ([guid]::NewGuid().ToString('N')))
    $fs = $null
    try {
        $bufSize = 1MB
        $stepBytes = 256MB
        $totalBytes = 2GB
        $buf = New-Object byte[] $bufSize
        (New-Object Random).NextBytes($buf)
        $noBuffering = [System.IO.FileOptions]0x20000000
        $fs = New-Object System.IO.FileStream($testFile, [System.IO.FileMode]::Create, [System.IO.FileAccess]::Write, [System.IO.FileShare]::None, $bufSize, ($noBuffering -bor [System.IO.FileOptions]::WriteThrough))
        $stepRates = New-Object System.Collections.Generic.List[double]
        $maxMs = 0.0
        $stallCount = 0
        $written = 0L
        $stepSw = [System.Diagnostics.Stopwatch]::StartNew()
        $ioSw = New-Object System.Diagnostics.Stopwatch
        $stepStart = 0L
        while ($written -lt $totalBytes) {
            $ioSw.Restart()
            $fs.Write($buf, 0, $bufSize)
            $ioSw.Stop()
            $ms = $ioSw.Elapsed.TotalMilliseconds
            if ($ms -gt $maxMs) { $maxMs = $ms }
            if ($ms -ge 500) { $stallCount++ }
            $written += $bufSize
            if (($written - $stepStart) -ge $stepBytes) {
                $rate = [math]::Round((($written - $stepStart) / 1MB) / $stepSw.Elapsed.TotalSeconds, 1)
                $stepRates.Add($rate)
                $bar = '#' * [math]::Min(60, [int][math]::Floor($rate / 10))
                Write-Host ('  {0,5} MB  {1,7:N1} MB/s  {2}' -f ($written / 1MB), $rate, $bar)
                $stepStart = $written
                $stepSw.Restart()
            }
        }
        $fs.Flush($true); $fs.Dispose()
        $susFirst = $stepRates[0]
        $susLast = $stepRates[$stepRates.Count - 1]
        $susMaxMs = [math]::Round($maxMs, 0)
        $susStalls = $stallCount
        Write-Host ('  Slowest single 1 MB write: {0} ms.  Writes that took 500 ms or longer: {1}' -f $susMaxMs, $susStalls)
        $susMin = ($stepRates | Measure-Object -Minimum).Minimum
        $susAvg = [math]::Round(($stepRates | Measure-Object -Average).Average, 1)
        if ($mediaType -eq 'SSD') {
            if ($susFirst -ge 300 -and $susLast -lt ($susFirst * 0.5)) {
                Write-Host ('  Shape: fast start ({0} MB/s) then drop to {1} MB/s = SLC cache filled. Normal for a DRAM-less SSD;' -f $susFirst, $susLast)
                Write-Host '         the drive is working, it just has a small cache.'
            }
            elseif ($susAvg -lt 150) {
                Write-Warn ('  Shape: slow from the first step (avg {0} MB/s, min {1}). The cache never engaged. A SATA SSD that cannot' -f $susAvg, $susMin)
                Write-Host '        exceed ~150 MB/s even on a fresh 256 MB with an idle disk and healthy SMART is controller/firmware-side'
                Write-Host '        trouble (stuck garbage collection, worn SLC region, or defective unit), not a Windows setting.'
            }
            else { Write-Ok ('  Shape: steady (avg {0} MB/s, min {1}).' -f $susAvg, $susMin) }
            if ($susMaxMs -ge 1000) {
                Write-Warn ('  A single 1 MB write stalled for {0} ms during the test. That is the freeze you feel, reproduced on demand.' -f $susMaxMs)
            }
        }
    }
    catch { Write-Warn ('Sustained write test failed: {0}' -f $_.Exception.Message) }
    finally {
        if ($null -ne $fs) { try { $fs.Dispose() } catch { } }
        Remove-Item -LiteralPath $testFile -Force -ErrorAction SilentlyContinue
    }
}

Write-Section 'Expected vs measured'
$expLabel = ''
$readWarn = 0; $readFail = 0; $writeWarn = 0; $ranWarn = 0
switch -Regex ($mediaType + '|' + $busType) {
    '^SSD\|NVMe' { $expLabel = 'NVMe SSD: 1500-7000 MB/s read, 1000-6000 write, 200-700 random';      $readWarn = 900; $readFail = 400; $writeWarn = 600; $ranWarn = 150 }
    '^SSD\|(SATA|ATA|RAID|SAS)' { $expLabel = 'SATA SSD: 450-560 MB/s read, 400-530 write, 150-400 random';  $readWarn = 350; $readFail = 150; $writeWarn = 250; $ranWarn = 80 }
    '^SSD\|USB' { $expLabel = 'USB SSD: 300-1000 MB/s depending on port (USB 3.0 ~400)';             $readWarn = 150; $readFail = 40;  $writeWarn = 100; $ranWarn = 20 }
    '^HDD' { $expLabel = 'Hard drive: 80-220 MB/s sequential, 0.5-2 MB/s random (HDDs are slow by nature)'; $readWarn = 60; $readFail = 30; $writeWarn = 50; $ranWarn = 0.3 }
    default { $expLabel = ('Unknown media ({0}/{1}) - SATA SSD expectations used' -f $mediaType, $busType); $readWarn = 350; $readFail = 150; $writeWarn = 250; $ranWarn = 80 }
}
Write-Host ('Expected for this drive: {0}' -f $expLabel)

$speedVerdict = 'PASS'
$speedReasons = New-Object System.Collections.Generic.List[string]
if ($null -ne $seqRead) {
    if ($seqRead -lt $readFail) { $speedVerdict = 'FAIL'; $speedReasons.Add('seq_read=' + $seqRead) }
    elseif ($seqRead -lt $readWarn) { if ($speedVerdict -ne 'FAIL') { $speedVerdict = 'WARN' }; $speedReasons.Add('seq_read_low=' + $seqRead) }
}
else { $speedVerdict = 'UNKNOWN'; $speedReasons.Add('no_read_measurement') }
if ($null -ne $seqWrite -and $seqWrite -lt $writeWarn) { if ($speedVerdict -eq 'PASS') { $speedVerdict = 'WARN' }; $speedReasons.Add('seq_write_low=' + $seqWrite) }
if ($null -ne $ranRead -and $ranRead -lt $ranWarn) { if ($speedVerdict -eq 'PASS') { $speedVerdict = 'WARN' }; $speedReasons.Add('random_read_low=' + $ranRead) }
if ($null -ne $susMaxMs -and $susMaxMs -ge 1000) { if ($speedVerdict -eq 'PASS') { $speedVerdict = 'WARN' }; $speedReasons.Add('write_stall_ms=' + $susMaxMs) }
if ($mediaType -eq 'SSD' -and $null -ne $susFirst -and $susFirst -lt 150 -and $null -ne $susLast -and $susLast -lt 150) { if ($speedVerdict -eq 'PASS') { $speedVerdict = 'WARN' }; $speedReasons.Add('sustained_write_low_first=' + $susFirst + '_last=' + $susLast) }

switch ($speedVerdict) {
    'PASS' { Write-Ok 'Drive speed is in the normal range for its type.' }
    'WARN' {
        Write-Warn 'Slower than expected for this drive type.'
        if ($mediaType -eq 'SSD' -and $busType -match 'SATA|ATA|RAID') {
            Write-Host 'Common causes, in order of likelihood for a SATA SSD:'
            Write-Host '  1. DRAM-less SSD (WD Blue SA510, Crucial BX500, Kingston A400...) past its SLC cache -> writes fall'
            Write-Host '     to 100-150 MB/s and random reads suffer. Vendor firmware updates often fix stalls on these.'
            Write-Host '  2. Old Intel RST / chipset storage driver (RAID mode). Update from the board/Intel site.'
            Write-Host '  3. Write caching disabled on the device (see settings section above).'
            Write-Host '  4. Background load during test (OneDrive, Defender, indexing) - see 10s sample above.'
            Write-Host '  5. SATA 3Gb/s port (caps ~270 MB/s), bad/loose SATA cable, or drive overheating.'
            Write-Host '     (cable check: in the vendor tool look at SMART "UDMA CRC Error Count" / attribute 199 - any'
            Write-Host '      value above 0 that keeps rising means cable or port, not the drive.)'
            Write-Host '  6. The SSD itself is degrading. Writes under ~100 MB/s plus multi-second stalls on current'
            Write-Host '     firmware, with the cable ruled out, points at the drive - use the warranty.'
        }
        if ($mediaType -eq 'HDD') { Write-Host 'A slow HDD as the Windows drive is the #1 cause of a sluggish PC. An SSD fixes it.' }
    }
    'FAIL' { Write-Bad 'Far below expected. Check SATA mode/port/cable; drive may be failing or in a degraded state.' }
    default { Write-Warn 'Speed could not be measured.' }
}
if ($mediaType -eq 'HDD') { Write-Host 'Note: Windows on an HDD will feel slow even if the drive is healthy and "passes".' }

$sr = if ($null -ne $seqRead) { $seqRead } else { 'n/a' }
$swr = if ($null -ne $seqWrite) { $seqWrite } else { 'n/a' }
$rr = if ($null -ne $ranRead) { $ranRead } else { 'n/a' }
$spReason = if ($speedReasons.Count -gt 0) { $speedReasons -join ',' } else { 'none' }
Write-Host ''
$sf = if ($null -ne $susFirst) { $susFirst } else { 'n/a' }
$sl = if ($null -ne $susLast) { $susLast } else { 'n/a' }
$sm = if ($null -ne $susMaxMs) { $susMaxMs } else { 'n/a' }
Write-Host ('DISK_SPEED: {0} drive={1}: type={2} bus={3} seq_read={4}MBps seq_write={5}MBps random_read={6}MBps sustained_first={8}MBps sustained_last={9}MBps max_write_ms={10} reasons={7}' -f $speedVerdict, $DriveLetter, $mediaType, $busType, $sr, $swr, $rr, $spReason, $sf, $sl, $sm)

if ($healthVerdict -eq 'FAIL' -or $speedVerdict -eq 'FAIL') { exit 2 }
exit 0
