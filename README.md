# RLX bridge panda: one-time "flash the code" branches

This folder hands off two git branches that could not be pushed from the opendbc agent
(it only has access to the opendbc repo):

| repo | branch | commit | what it is |
|---|---|---|---|
| `mvl-boston/panda` | `rlx-forwarder-flash` | `48cb7ef79f870e799ff6f348657f0eb9d0bfda47` | commaai/panda `615009cf0` (what openpilot `rlx-test1` pins) + `board/main.c`: red pandas boot into `SAFETY_HONDA_RLX_FORWARDER`, heartbeat check off, green LED solid on |
| `mvl-boston/openpilot` | `rlx-forwarder-flash` | `be8aa09adcfc34444fcd93aedd148f12bd19f175` | openpilot `rlx-test1` with `panda` → the branch above and `opendbc_repo` → opendbc `rlx-forwarder-flash` (`8a8c0886f`) |

The opendbc half already exists: branch `rlx-forwarder-flash` in this repo.

## Push the two branches (once, from any computer with git)

```bash
bash push_flash_branches.sh
```

Git prompts for your GitHub login for the two pushes. The script clones shallowly, imports the
bundles, checks the commit hashes above and force-pushes `rlx-forwarder-flash` to both repos.
`bash push_flash_branches.sh --dry-run` does everything except the pushes.

If you would rather do it by hand, the same commits are also here as plain patches
(`*.patch`, apply with `git am`), and the bundles can be imported with
`git fetch <file>.bundle rlx-forwarder-flash:rlx-forwarder-flash`.

## Flash the red panda (the end user)

1. Switch the comma to branch `rlx-forwarder-flash` and let the first boot finish building.
2. Plug the red panda into the comma's second USB-C port and power-cycle the comma.
3. Wait until the red panda's **green LED stays solid on**. That only happens when it is running
   the forwarder firmware.
4. Unplug the red panda, install it on the car (harness between LKAS camera and EPS on bus 0/2,
   tap on the powertrain bus on bus 1) and switch the comma back to the regular branch.
   The regular branch reflashes the comma's internal panda by itself.

Re-flashing later (e.g. to change the gateway-relay param in `main.c`) is the same routine.
