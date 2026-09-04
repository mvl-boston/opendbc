# RLX steer-bus bridge: apply patches with SSH (WSL)

Save this whole `rlx-handoff/` folder somewhere on your Windows drive, e.g.:

`C:\Users\you\rlx-handoff\`

In WSL that path is usually `/mnt/c/Users/you/rlx-handoff/`. All commands below use a
`PATCHES` variable — change the path to match where you saved the files.

```bash
export PATCHES=/mnt/c/Users/you/rlx-handoff
```

These patches use `git am` (not `git apply`). `git am` replays the commit metadata and
is what you want for the single-commit panda and openpilot patches. The opendbc series
is five commits in order; apply them with `git am opendbc/*.patch`.

SSH auth: clone with `git@github.com:mvl-boston/...` — your existing deploy keys / SSH
agent work the same as any other git push. No password prompt.

---

## 1. opendbc (regular driving branch)

**Goal:** get the hondaRlxForwarder safety mode + ACURA_RLX_HYBRID port onto `mdx-pedaltest`.

**Easiest:** merge [opendbc PR #689](https://github.com/mvl-boston/opendbc/pull/689) (branch
`cursor/rlx-steer-bus-forwarder-ddf5` → `mdx-pedaltest`). It is already on GitHub.

**Or apply the patch series locally:**

```bash
git clone git@github.com:mvl-boston/opendbc.git
cd opendbc
git checkout mdx-pedaltest
git pull origin mdx-pedaltest
git checkout -b rlx-steer-bus-forwarder
git am "$PATCHES"/opendbc/*.patch
# resolve any conflicts, then: git am --continue
git push -u origin rlx-steer-bus-forwarder
```

Result commit (tip of the series): `dc70509aecc8755719e3bc57b9720498142f7c58`

---

## 2. panda (red-panda bridge firmware)

**Goal:** branch `rlx-forwarder-flash` — red pandas boot into `SAFETY_HONDA_RLX_FORWARDER`.

The patch applies on top of commaai/panda `615009cf` (what openpilot `rlx-test1` pins).

```bash
git clone git@github.com:mvl-boston/panda.git
cd panda
git fetch https://github.com/commaai/panda.git 615009cf0f8fb8f3feadac160fbb0a07e4de171b
git checkout -b rlx-forwarder-flash 615009cf0f8fb8f3feadac160fbb0a07e4de171b
git am "$PATCHES"/panda-rlx-forwarder-flash.patch
git push -u origin rlx-forwarder-flash
```

Result commit: `48cb7ef79f870e799ff6f348657f0eb9d0bfda47`

If `git am` reports conflicts, fix `board/main.c`, then `git add board/main.c && git am --continue`.

---

## 3. openpilot (one-time flash branch)

**Goal:** branch `rlx-forwarder-flash` — old multi-panda `pandad` flashes the red panda when
you boot the comma on this branch with the red panda plugged into USB.

The patch applies on top of openpilot `rlx-test1`.

```bash
git clone --recurse-submodules=no git@github.com:mvl-boston/openpilot.git
cd openpilot
git checkout rlx-test1
git pull origin rlx-test1
git checkout -b rlx-forwarder-flash
git am "$PATCHES"/openpilot-rlx-forwarder-flash.patch
git push -u origin rlx-forwarder-flash
```

Result commit: `be8aa09adcfc34444fcd93aedd148f12bd19f175`

This commit re-pins submodules to:

| submodule | commit |
|---|---|
| `panda` | `48cb7ef79f870e799ff6f348657f0eb9d0bfda47` (step 2) |
| `opendbc_repo` | `8a8c0886f6b14f639aff06c9d883ef32c856bc7e` (`rlx-forwarder-flash` on opendbc) |

The opendbc `rlx-forwarder-flash` branch (`8a8c0886f`) is already on GitHub; you do not
need to push it unless you changed it.

---

## 4. opendbc `rlx-forwarder-flash` (only if you need to recreate it)

Already on GitHub at `8a8c0886f`. If you ever need to rebuild it from `rlx-step21` history,
check out that branch and cherry-pick the three hondaRlxForwarder safety commits from the
`opendbc/*.patch` series (commits 1, 4, and 5 of the five — or just merge PR #689 into a
branch based on your old `rlx-step21` line).

---

## Flash the red panda (end user, on the comma)

1. On the comma (or your dev machine): `git checkout rlx-forwarder-flash` in openpilot and
   let the first build finish.
2. Plug the red panda into the comma's second USB-C port; power-cycle.
3. Wait for the red panda **green LED solid on** (forwarder mode running).
4. Unplug the red panda, wire it on the car (camera intercept + powertrain tap), switch the
   comma back to your regular branch. The internal panda is reflashed by the regular branch.

---

## Troubleshooting

| problem | fix |
|---|---|
| `git am` says "patch does not apply" | you are not on the base commit listed above; `git checkout <base>` and retry |
| panda patch fails on `SAFETY_HONDA_RLX_FORWARDER` | opendbc safety enum not in this panda tree — use base `615009cf` from commaai, not `mvl-boston/panda` master |
| openpilot patch fails on submodule lines | make sure you started from `rlx-test1`, not `mdx-pedaltest` |
| SSH permission denied | `ssh -T git@github.com` — use the same key you use for your other forks |
