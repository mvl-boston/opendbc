#Requires -Version 5.1
param(
    [string] $DriveLetter = $env:SystemDrive.TrimEnd(':'),
    [switch] $SkipVolumeScan,
    [switch] $SkipSpeedTest
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
}

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
    @{ Label = 'Ntfs: corruption (55/130/137/140)';   Provider = 'Ntfs';      Ids = @(55,130,137,140); Level = 'FAIL' },
    @{ Label = 'storahci: device reset (129)';        Provider = 'storahci';  Ids = @(129);      Level = 'WARN' },
    @{ Label = 'stornvme: device reset (129)';        Provider = 'stornvme';  Ids = @(129);      Level = 'WARN' },
    @{ Label = 'iaStorA/iaStorAC: reset (129)';       Provider = @('iaStorA','iaStorAC','iaStorAVC'); Ids = @(129); Level = 'WARN' }
)
$totalDiskEvents = 0
foreach ($chk in $eventChecks) {
    $count = 0
    $last = $null
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
    catch { Write-Warn ('Repair-Volume scan failed: {0}' -f $_.Exception.Message) }
}

Write-Host ''
$reasonTxt = if ($healthReasons.Count -gt 0) { $healthReasons -join ',' } else { 'none' }
Write-Host ('DISK_HEALTH: {0} disk={1} type={2} bus={3} reasons={4}' -f $healthVerdict, $targetPd.FriendlyName, $mediaType, $busType, $reasonTxt)

if ($SkipSpeedTest) { exit 0 }

Write-Section ('Speed test on {0}: (Windows built-in winsat)' -f $DriveLetter)
$seqRead = $null; $seqWrite = $null; $ranRead = $null
$winsat = Join-Path $env:SystemRoot 'System32\WinSAT.exe'
if (Test-Path -LiteralPath $winsat) {
    $tests = @(
        @{ Args = @('disk', '-drive', $DriveLetter, '-seq', '-read', '-count', '3');  Name = 'Sequential read' },
        @{ Args = @('disk', '-drive', $DriveLetter, '-seq', '-write', '-count', '3'); Name = 'Sequential write' },
        @{ Args = @('disk', '-drive', $DriveLetter, '-ran', '-read', '-count', '3');  Name = 'Random 16K read' }
    )
    foreach ($t in $tests) {
        Write-Host ('Running winsat {0}...' -f $t.Name)
        $out = & $winsat @($t.Args) 2>&1 | Out-String
        $mbps = $null
        foreach ($line in ($out -split "`r?`n")) {
            if ($line -match '>\s*Disk\s+(Sequential|Random)\s+[\d.]+\s+(Read|Write)\s+([\d.]+)\s+MB/s') {
                $mbps = [double]$Matches[3]
            }
        }
        if ($null -eq $mbps) {
            $snippet = ($out -split "`r?`n" | Where-Object { $_ -match 'Error|error|denied|battery|not supported' } | Select-Object -First 2) -join ' | '
            Write-Warn ('{0}: could not parse result. {1}' -f $t.Name, $snippet)
            continue
        }
        Write-Host ('  {0,-18} {1,8:N1} MB/s' -f $t.Name, $mbps)
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

switch ($speedVerdict) {
    'PASS' { Write-Ok 'Drive speed is in the normal range for its type.' }
    'WARN' {
        Write-Warn 'Slower than expected for this drive type.'
        if ($mediaType -eq 'SSD' -and $busType -match 'SATA|ATA') {
            Write-Host 'Common causes: drive on a SATA 3Gb/s port (cap ~270 MB/s), nearly full SSD, TRIM off,'
            Write-Host 'AHCI not enabled in BIOS (IDE mode), worn SSD, or bad SATA cable.'
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
Write-Host ('DISK_SPEED: {0} drive={1}: type={2} bus={3} seq_read={4}MBps seq_write={5}MBps random_read={6}MBps reasons={7}' -f $speedVerdict, $DriveLetter, $mediaType, $busType, $sr, $swr, $rr, $spReason)

if ($healthVerdict -eq 'FAIL' -or $speedVerdict -eq 'FAIL') { exit 2 }
exit 0
