#!/usr/bin/env python3
"""
find_0x1be_bulk.py — exhaustive 0x1BE search for all Honda/Acura segments.

Phase 1: per-file hf_hub_download of every Honda/Acura rlog.zst listed in
         database.json into LOCAL_DIR. Anonymous (no HF token).
Phase 2: local-only scan, no network.

LOCAL_DIR is wiped at startup AND in a finally: block at exit. Drive cannot
fill up across runs.

Ctrl-C handling:
  1st Ctrl-C: cooperative stop. Worker threads wake from any retry sleep,
              finish or abort their current in-flight download (capped by
              socket timeout), the run unwinds, and LOCAL_DIR is wiped in
              the finally block.
  2nd Ctrl-C: force exit (os._exit). LOCAL_DIR will be wiped on the next
              run's startup.

Requires: pip install huggingface_hub  (use `uv pip install huggingface_hub`
if you get an externally-managed-environment error on Debian/Ubuntu).
"""

import os

os.environ.setdefault("HF_HUB_DISABLE_PROGRESS_BARS", "1")
os.environ.setdefault("HF_HUB_DISABLE_TELEMETRY", "1")

import gc
import itertools
import json
import logging
import multiprocessing as mp
import shutil
import signal
import socket
import sys
import threading
import time
import traceback
from concurrent.futures import FIRST_COMPLETED, ProcessPoolExecutor
from concurrent.futures import ThreadPoolExecutor, as_completed, wait
from pathlib import Path
from urllib.request import urlretrieve

socket.setdefaulttimeout(60)

LOCAL_DIR = Path("/tmp/ccs_local")
DB_PATH = Path("database.json")
LOG_PATH = Path("find_0x1be_debug.log")
HITS_PATH = Path("find_0x1be_hits.txt")
INCOMPLETE_PATH = Path("find_0x1be_incomplete.txt")
FAILED_PREFETCH_PATH = Path("find_0x1be_failed_prefetch.txt")

DB_URL = "https://huggingface.co/datasets/commaai/commaCarSegments/raw/main/database.json"
REPO_ID = "commaai/commaCarSegments"

TARGET_ADDR = 0x1BE
MAX_MSGS_PER_LOG = 4
MAX_ROUTES_PER_FP = 99999
MAX_SEGS_PER_ROUTE = 2

PREFETCH_MAX_WORKERS = 4
PREFETCH_MAX_ATTEMPTS_PER_FILE = 200
PREFETCH_RETRY_BASE_SEC = 5
PREFETCH_RETRY_MAX_SEC = 60
PROGRESS_LOG_EVERY_SEC = 10

SCAN_MAX_WORKERS = 4
MAX_RETRIES_PER_SEG = 5
STALL_TIMEOUT_SEC = 30
HEARTBEAT_SEC = 10


# ---------- cooperative-shutdown plumbing ----------

_stop_event = threading.Event()
_sigint_count = 0


def _install_sigint_handler(log: logging.Logger) -> None:
    def _h(signum, frame):
        global _sigint_count
        _sigint_count += 1
        if _sigint_count == 1:
            log.warning(
                "*** SIGINT received — stopping cooperatively. "
                "Workers will finish current in-flight downloads (≤60s each), "
                "then the run will exit and LOCAL_DIR will be wiped. "
                "Press Ctrl-C again to force exit immediately."
            )
            _stop_event.set()
        else:
            log.warning(
                "*** Second SIGINT — force exit. LOCAL_DIR will be wiped on next run startup."
            )
            os._exit(130)

    signal.signal(signal.SIGINT, _h)


# ---------- logging / housekeeping ----------

class FlushingFileHandler(logging.FileHandler):
    def emit(self, record):
        super().emit(record)
        self.flush()
        try:
            os.fsync(self.stream.fileno())
        except (OSError, ValueError):
            pass


def setup_logging() -> logging.Logger:
    log = logging.getLogger("find_0x1be")
    log.setLevel(logging.DEBUG)
    log.handlers.clear()
    fh = FlushingFileHandler(LOG_PATH, mode="w")
    fh.setFormatter(logging.Formatter(
        "%(asctime)s.%(msecs)03d %(levelname)-5s %(message)s", "%H:%M:%S"))
    sh = logging.StreamHandler(sys.stdout)
    sh.setFormatter(logging.Formatter("%(message)s"))
    log.addHandler(fh)
    log.addHandler(sh)
    log.propagate = False
    return log


def rss_mb() -> float:
    try:
        with open(f"/proc/{os.getpid()}/status") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1]) / 1024.0
    except OSError:
        pass
    return -1.0


def dir_size_gb(path: Path) -> float:
    if not path.exists():
        return 0.0
    total = 0
    for p in path.rglob("*"):
        try:
            total += p.stat().st_size
        except OSError:
            pass
    return total / 1e9


def wipe_local_dir(log: logging.Logger | None = None) -> None:
    if LOCAL_DIR.exists():
        size = dir_size_gb(LOCAL_DIR)
        if log:
            log.info(f"wiping {LOCAL_DIR} ({size:.2f} GB)")
        shutil.rmtree(LOCAL_DIR, ignore_errors=True)


def fsync_safe(f) -> None:
    try:
        os.fsync(f.fileno())
    except OSError:
        pass


# ---------- phase 1: per-file prefetch ----------

def load_database(log: logging.Logger) -> dict:
    if not DB_PATH.exists():
        log.info(f"downloading database.json from {DB_URL}")
        urlretrieve(DB_URL, DB_PATH)
    log.info(f"database.json size = {DB_PATH.stat().st_size} bytes")
    with DB_PATH.open() as f:
        return json.load(f)


def build_targets(data: dict, log: logging.Logger):
    """Returns (rel_paths_to_download, fp_to_local_paths)."""
    rel_paths: list[str] = []
    fp_to_paths: dict[str, list[Path]] = {}
    for fp, entries in data.items():
        if "HONDA" not in fp and "ACURA" not in fp:
            continue
        by_route: dict[str, list[str]] = {}
        for entry in entries:
            parts = entry.split("/")
            if len(parts) < 3:
                continue
            base, seg = "/".join(parts[:2]), parts[2]
            segs = by_route.setdefault(base, [])
            if seg not in segs and len(segs) < MAX_SEGS_PER_ROUTE:
                segs.append(seg)
        local_paths: list[Path] = []
        for base, segs in list(by_route.items())[:MAX_ROUTES_PER_FP]:
            for seg in segs:
                rel = f"segments/{base}/{seg}/rlog.zst"
                rel_paths.append(rel)
                local_paths.append(LOCAL_DIR / rel)
        fp_to_paths[fp] = local_paths
    log.info(
        f"selected {len(rel_paths)} files across {len(fp_to_paths)} fingerprints "
        f"(~{len(rel_paths)} MB raw)"
    )
    return rel_paths, fp_to_paths


def prefetch(rel_paths: list[str], log: logging.Logger) -> bool:
    """Download each file individually via hf_hub_download. Returns True if
    completed normally, False if interrupted by SIGINT."""
    from huggingface_hub import hf_hub_download
    from huggingface_hub.utils import HfHubHTTPError

    LOCAL_DIR.mkdir(parents=True, exist_ok=True)

    def download_one(rel: str) -> tuple[str, str, str]:
        if _stop_event.is_set():
            return rel, "cancelled", ""
        local_file = LOCAL_DIR / rel
        if local_file.exists() and local_file.stat().st_size > 0:
            return rel, "skipped", ""
        attempt = 0
        last_err = ""
        while not _stop_event.is_set():
            attempt += 1
            try:
                hf_hub_download(
                    repo_id=REPO_ID,
                    repo_type="dataset",
                    filename=rel,
                    local_dir=str(LOCAL_DIR),
                )
                if attempt > 1:
                    log.info(f"  recovered on attempt {attempt}: {rel}")
                return rel, "ok", ""
            except (HfHubHTTPError, ConnectionError, TimeoutError, OSError) as e:
                last_err = f"{type(e).__name__}: {str(e)[:120]}"
                if attempt >= PREFETCH_MAX_ATTEMPTS_PER_FILE:
                    return rel, "failed", last_err
                wait_s = min(
                    PREFETCH_RETRY_MAX_SEC,
                    PREFETCH_RETRY_BASE_SEC * (2 ** (attempt - 1)),
                )
                log.info(
                    f"  retry {attempt}/{PREFETCH_MAX_ATTEMPTS_PER_FILE} "
                    f"({wait_s}s wait) on {rel}: {last_err}"
                )
                if _stop_event.wait(timeout=wait_s):
                    return rel, "cancelled", ""
            except Exception as e:
                return rel, "failed", f"{type(e).__name__}: {str(e)[:120]}"
        return rel, "cancelled", ""

    t0 = time.perf_counter()
    n_total = len(rel_paths)
    n_done = n_ok = n_skipped = n_failed = n_cancelled = 0
    failures: list[tuple[str, str]] = []
    last_failed_count_announced = 0

    log.info(
        f"prefetching {n_total} files with {PREFETCH_MAX_WORKERS} workers "
        f"via hf_hub_download "
        f"(per-file, idempotent, up to {PREFETCH_MAX_ATTEMPTS_PER_FILE} attempts/file, "
        f"backoff capped at {PREFETCH_RETRY_MAX_SEC}s)"
    )

    ex = ThreadPoolExecutor(max_workers=PREFETCH_MAX_WORKERS, thread_name_prefix="prefetch")
    futures: dict = {}
    last_log = time.monotonic()
    try:
        futures = {ex.submit(download_one, r): r for r in rel_paths}
        for fut in as_completed(futures):
            if _stop_event.is_set():
                break
            try:
                rel, status, err = fut.result()
            except Exception as e:
                n_done += 1
                n_failed += 1
                failures.append(("<task-crash>", f"{type(e).__name__}: {e}"))
                log.error(f"  task crashed: {type(e).__name__}: {e}")
                continue
            n_done += 1
            if status == "ok":
                n_ok += 1
            elif status == "skipped":
                n_skipped += 1
            elif status == "cancelled":
                n_cancelled += 1
            else:
                n_failed += 1
                failures.append((rel, err))
                log.warning(f"  FAILED ({n_failed}): {rel} — {err}")

            now = time.monotonic()
            elapsed = now - t0
            need_progress_line = (
                now - last_log >= PROGRESS_LOG_EVERY_SEC
                or n_done == n_total
                or n_failed > last_failed_count_announced
            )
            if need_progress_line:
                last_log = now
                last_failed_count_announced = n_failed
                size_gb = dir_size_gb(LOCAL_DIR)
                rate = n_done / max(0.001, elapsed)
                eta_s = (n_total - n_done) / max(0.001, rate)
                fail_tag = f"  ⚠ failed={n_failed}" if n_failed else "  failed=0"
                log.info(
                    f"  progress: {n_done}/{n_total}  "
                    f"ok={n_ok} skipped={n_skipped}{fail_tag}  "
                    f"{size_gb:.2f} GB on disk  "
                    f"{rate:.2f} files/s  ETA {eta_s/60:.1f} min"
                )
    finally:
        ex.shutdown(wait=not _stop_event.is_set(), cancel_futures=True)

    dt = time.perf_counter() - t0
    n_files = sum(1 for _ in LOCAL_DIR.rglob("rlog.zst"))
    size_gb = dir_size_gb(LOCAL_DIR)
    if _stop_event.is_set():
        log.warning(
            f"prefetch INTERRUPTED after {dt:.1f}s: "
            f"{n_files} files on disk, {size_gb:.2f} GB, "
            f"ok={n_ok} skipped={n_skipped} failed={n_failed} cancelled={n_cancelled}"
        )
    else:
        log.info(
            f"prefetch done in {dt:.1f}s: {n_files} files on disk, "
            f"{size_gb:.2f} GB, failures={n_failed}"
        )

    if failures:
        with FAILED_PREFETCH_PATH.open("w") as f:
            for rel, err in failures:
                f.write(f"{rel}\t{err}\n")
        log.warning(
            f"⚠ {len(failures)} files exhausted retries — see {FAILED_PREFETCH_PATH}"
        )

    return not _stop_event.is_set()


# ---------- phase 2: local scan ----------

def _scan_worker(local_path: str) -> tuple[str, str, float, str]:
    """Returns (path, status, elapsed_s, err).
    status in {'hit', 'miss', 'missing', 'load_err', 'crash'}."""
    t0 = time.perf_counter()
    try:
        from openpilot.tools.lib.logreader import LogReader
        lr = LogReader(local_path)
    except FileNotFoundError as e:
        return local_path, "missing", time.perf_counter() - t0, f"FileNotFoundError: {e}"
    except Exception as e:
        return local_path, "load_err", time.perf_counter() - t0, f"{type(e).__name__}: {e}"
    try:
        for msg in itertools.islice(lr, MAX_MSGS_PER_LOG):
            try:
                if msg.which() != "can":
                    continue
            except Exception:
                continue
            for frame in msg.can:
                if frame.address == TARGET_ADDR:
                    return local_path, "hit", time.perf_counter() - t0, ""
        return local_path, "miss", time.perf_counter() - t0, ""
    except Exception as e:
        return local_path, "crash", time.perf_counter() - t0, f"{type(e).__name__}: {e}"


def make_pool() -> ProcessPoolExecutor:
    ctx = mp.get_context("forkserver")
    return ProcessPoolExecutor(max_workers=SCAN_MAX_WORKERS, mp_context=ctx)


def kill_pool(pool: ProcessPoolExecutor) -> None:
    try:
        procs = list(getattr(pool, "_processes", {}).values())
    except Exception:
        procs = []
    for p in procs:
        try:
            p.kill()
        except Exception:
            pass
    try:
        pool.shutdown(wait=False, cancel_futures=True)
    except Exception:
        pass


def scan_fp(fp: str, paths: list[Path], log: logging.Logger):
    counts = {"miss": 0, "missing": 0, "load_err": 0, "crash": 0}
    if not paths:
        return None, counts, []

    pool = make_pool()
    str_paths = [str(p) for p in paths]
    attempts: dict[str, int] = {p: 0 for p in str_paths}
    given_up: set[str] = set()
    queue: list[str] = list(str_paths)
    hit: str | None = None

    try:
        while queue and hit is None and not _stop_event.is_set():
            current = queue
            queue = []
            futures = {pool.submit(_scan_worker, p): p for p in current}
            pending = set(futures)
            last_completion = time.monotonic()

            while pending and hit is None and not _stop_event.is_set():
                now = time.monotonic()
                since = now - last_completion
                if since > STALL_TIMEOUT_SEC:
                    log.warning(
                        f"  STALL: no completion for {since:.0f}s, "
                        f"{len(pending)} pending; recreating pool"
                    )
                    for f in pending:
                        queue.append(futures[f])
                    kill_pool(pool)
                    pool = make_pool()
                    break

                wait_t = max(0.05, min(STALL_TIMEOUT_SEC - since + 0.1, HEARTBEAT_SEC))
                done, pending = wait(pending, timeout=wait_t, return_when=FIRST_COMPLETED)

                if not done:
                    log.info(
                        f"  heartbeat: {len(pending)} pending, "
                        f"{since:.0f}s since last completion"
                    )
                    continue
                last_completion = time.monotonic()

                for fut in done:
                    path = futures[fut]
                    try:
                        local_path, status, dt, err = fut.result()
                    except Exception as e:
                        log.error(f"  fut.result(): {type(e).__name__}: {e}")
                        attempts[path] += 1
                        if attempts[path] <= MAX_RETRIES_PER_SEG:
                            queue.append(path)
                        else:
                            given_up.add(path)
                        continue

                    if status == "hit":
                        log.debug(f"  hit       {dt:5.3f}s  {local_path}")
                        hit = local_path
                        break
                    elif status == "miss":
                        counts["miss"] += 1
                    elif status == "missing":
                        counts["missing"] += 1
                        log.debug(f"  missing   {local_path}")
                    else:
                        counts[status if status in counts else "load_err"] += 1
                        attempts[path] += 1
                        if attempts[path] <= MAX_RETRIES_PER_SEG:
                            queue.append(path)
                        else:
                            given_up.add(path)
                            log.warning(f"  GAVE UP {path}: {err}")

            if hit is not None:
                for f in pending:
                    f.cancel()
                break
    finally:
        kill_pool(pool)

    unresolved = sorted(given_up)
    return hit, counts, unresolved


def scan(fp_to_paths: dict, log: logging.Logger) -> None:
    fps = list(fp_to_paths.items())
    log.info(f"--- Phase 2: scan {len(fps)} fingerprints ---")
    t0 = time.perf_counter()
    found = 0
    incomplete_count = 0

    with HITS_PATH.open("w", buffering=1) as hits_f, \
         INCOMPLETE_PATH.open("w", buffering=1) as inc_f:
        for i, (fp, paths) in enumerate(fps, 1):
            if _stop_event.is_set():
                log.warning(f"scan interrupted before fp [{i}/{len(fps)}]")
                break
            fp_t0 = time.perf_counter()
            log.info(
                f"[{i:3d}/{len(fps)}]  start  {fp:30s}  "
                f"({len(paths)} candidates)  rss={rss_mb():.1f}MB"
            )
            try:
                hit, counts, unresolved = scan_fp(fp, paths, log)
            except Exception:
                log.error(f"[{i:3d}/{len(fps)}]  ERROR in {fp}:\n{traceback.format_exc()}")
                hit, counts, unresolved = None, {
                    "miss": 0, "missing": 0, "load_err": 0, "crash": 0
                }, []

            dt = time.perf_counter() - fp_t0
            summary = (
                f"miss={counts['miss']} missing={counts['missing']} "
                f"load_err={counts['load_err']} crash={counts['crash']}"
            )

            if hit is not None:
                found += 1
                hits_f.write(f"{fp}\t{hit}\n")
                hits_f.flush()
                fsync_safe(hits_f)
                log.info(
                    f"[{i:3d}/{len(fps)}]  {dt:6.1f}s  HIT          {fp:30s}  "
                    f"{hit}   {summary}   rss={rss_mb():.1f}MB"
                )
            elif unresolved:
                incomplete_count += 1
                for p in unresolved:
                    inc_f.write(f"{fp}\t{p}\n")
                inc_f.flush()
                fsync_safe(inc_f)
                log.warning(
                    f"[{i:3d}/{len(fps)}]  {dt:6.1f}s  INCOMPLETE   {fp:30s}  "
                    f"({len(unresolved)} unresolved)   {summary}   rss={rss_mb():.1f}MB"
                )
            else:
                log.info(
                    f"[{i:3d}/{len(fps)}]  {dt:6.1f}s  miss         {fp:30s}  "
                    f"{summary}   rss={rss_mb():.1f}MB"
                )
            gc.collect()

    total = time.perf_counter() - t0
    log.info(
        f"scan done in {total:.1f}s  hits={found}  incomplete={incomplete_count}  "
        f"of {len(fps)} fingerprints"
    )


# ---------- main driver ----------

def main() -> None:
    log = setup_logging()
    _install_sigint_handler(log)

    log.info(f"python={sys.version.split()[0]}  cwd={os.getcwd()}")
    log.info(
        f"TARGET_ADDR=0x{TARGET_ADDR:X}  MAX_MSGS_PER_LOG={MAX_MSGS_PER_LOG}  "
        f"MAX_ROUTES_PER_FP={MAX_ROUTES_PER_FP}  MAX_SEGS_PER_ROUTE={MAX_SEGS_PER_ROUTE}"
    )
    log.info(
        f"PREFETCH_MAX_WORKERS={PREFETCH_MAX_WORKERS}  "
        f"PREFETCH_MAX_ATTEMPTS_PER_FILE={PREFETCH_MAX_ATTEMPTS_PER_FILE}  "
        f"PREFETCH_RETRY_BASE={PREFETCH_RETRY_BASE_SEC}s  "
        f"PREFETCH_RETRY_MAX={PREFETCH_RETRY_MAX_SEC}s  "
        f"PROGRESS_LOG_EVERY={PROGRESS_LOG_EVERY_SEC}s  "
        f"socket_timeout=60s"
    )
    log.info(
        f"SCAN_MAX_WORKERS={SCAN_MAX_WORKERS}  MAX_RETRIES_PER_SEG={MAX_RETRIES_PER_SEG}"
    )
    log.info(
        f"local_dir={LOCAL_DIR}  log={LOG_PATH}  "
        f"hits={HITS_PATH}  incomplete={INCOMPLETE_PATH}  "
        f"failed_prefetch={FAILED_PREFETCH_PATH}"
    )

    log.info("wiping LOCAL_DIR at startup (drive-fill safety) ...")
    wipe_local_dir(log)
    if FAILED_PREFETCH_PATH.exists():
        FAILED_PREFETCH_PATH.unlink()

    t0 = time.perf_counter()
    try:
        data = load_database(log)
        rel_paths, fp_to_paths = build_targets(data, log)

        log.info("--- Phase 1: bulk prefetch from HuggingFace (anonymous, per-file) ---")
        completed = prefetch(rel_paths, log)

        if not completed or _stop_event.is_set():
            log.warning("prefetch was interrupted; skipping scan")
        else:
            scan(fp_to_paths, log)
    except KeyboardInterrupt:
        log.warning("KeyboardInterrupt propagated to main")
        _stop_event.set()
    except Exception:
        log.error(f"fatal error:\n{traceback.format_exc()}")
    finally:
        total = time.perf_counter() - t0
        log.info(f"total wall time = {total:.1f}s")
        log.info("wiping LOCAL_DIR at exit (drive-fill safety) ...")
        wipe_local_dir(log)
        log.info(f"final rss={rss_mb():.1f}MB  done.")


if __name__ == "__main__":
    main()
