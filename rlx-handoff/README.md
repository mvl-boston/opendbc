# RLX hand-off patches

Patch files and WSL instructions for repos the opendbc agent cannot push to directly.

| file | repo | base branch / commit | creates |
|---|---|---|---|
| `opendbc/*.patch` (5 files) | mvl-boston/opendbc | `mdx-pedaltest` | RLX port + hondaRlxForwarder safety mode |
| `panda-rlx-forwarder-flash.patch` | mvl-boston/panda | commaai/panda `615009cf` | `rlx-forwarder-flash` |
| `openpilot-rlx-forwarder-flash.patch` | mvl-boston/openpilot | `rlx-test1` | `rlx-forwarder-flash` |

See **[APPLY_PATCHES.md](APPLY_PATCHES.md)** for copy-paste WSL commands using SSH (`git@github.com:...`).

The opendbc driving changes are also in PR #689 (`cursor/rlx-steer-bus-forwarder-ddf5` → `mdx-pedaltest`).
