Windows CPU health check (driver-free)
======================================

One file does everything:

  run-all-cpu-checks.bat          (double-click, approve Administrator)
  run-all-cpu-checks.bat 300      (longer 5-minute throttle test)

It writes cpu-full-report.txt next to itself. Key result lines:

  SPEED_TEST: PASS ...        CPU holds its rated clock under 45s all-core load
  THROTTLE_EVENTS: NONE       Windows has not logged firmware speed-limit events
  THROTTLE_TEST: PASS ...     No slowdown during a long sustained all-core load

Why no temperature number?
--------------------------
Windows has no supported, driver-free way to read the CPU die temperature.
Every tool that shows it (HWiNFO, LibreHardwareMonitor, Intel Power Gadget)
installs a kernel driver, and that is what Windows Defender / policy fights.

Instead this suite detects the *effect* of overheating: when a CPU hits its
thermal limit it throttles, so MHz and work-per-second fall under constant
load. If that does not happen over several minutes of full load, the cooler
and thermal paste are doing their job. To see an actual temperature number,
check your BIOS/UEFI "Hardware Monitor" page at boot (idle temperature only).

Disk / SSD check (also driver-free)
-----------------------------------
  run-disk-check.bat              (Windows drive)
  run-disk-check.bat D            (another drive letter)
  make-rma-evidence.bat           (writes rma-evidence.txt: a warranty-claim document for a slow SSD;
                                   attach it with your vendor Dashboard SMART screenshots)

Writes disk-full-report.txt. Key lines:
  DISK_HEALTH: PASS ...   SMART/reliability counters, error events, file system
  DISK_SPEED:  PASS ...   winsat sequential/random MB/s vs expected for the drive type

Removing earlier third-party tools
----------------------------------
  cleanup-cpu-tools.bat       (uninstalls HWiNFO, Intel Power Gadget, removes
                               LibreHardwareMonitor folder and WinRing0 driver
                               service, deletes downloaded helper files)

Raw download link for the one-click file:
  https://raw.githubusercontent.com/mvl-boston/opendbc/cursor/windows-cpu-health-check-65b2/scripts/windows/run-all-cpu-checks.bat
