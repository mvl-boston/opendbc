#!/usr/bin/env bash
# Pushes the "rlx-forwarder-flash" branches to mvl-boston/panda and mvl-boston/openpilot.
#
# Run this on any computer with git installed (Mac, Linux, or Windows "Git Bash"):
#   bash push_flash_branches.sh
# Git will ask you to log in to GitHub for the two pushes. The clones are shallow, so it's quick.
#
# Add --dry-run to do everything except the two pushes.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PUSH="git push --force"
if [ "${1:-}" = "--dry-run" ]; then
  PUSH="echo [dry-run] git push --force"
fi

PANDA_BASE=615009cf0f8fb8f3feadac160fbb0a07e4de171b     # commaai/panda commit that openpilot rlx-test1 pins
PANDA_FLASH=48cb7ef79f870e799ff6f348657f0eb9d0bfda47    # that + board/main.c forwarder boot
OPENPILOT_FLASH=be8aa09adcfc34444fcd93aedd148f12bd19f175 # openpilot rlx-test1 + submodule re-pins

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
cd "$WORK"

echo "== panda: creating branch rlx-forwarder-flash"
git clone -q --depth 1 --no-checkout https://github.com/mvl-boston/panda.git panda
cd panda
git fetch -q --depth 1 https://github.com/commaai/panda.git "$PANDA_BASE"
git fetch -q "$HERE/panda-rlx-forwarder-flash.bundle" rlx-forwarder-flash:rlx-forwarder-flash
test "$(git rev-parse rlx-forwarder-flash)" = "$PANDA_FLASH"
$PUSH origin rlx-forwarder-flash
cd ..

echo "== openpilot: creating branch rlx-forwarder-flash"
git clone -q --depth 1 --no-checkout --branch rlx-test1 --no-recurse-submodules https://github.com/mvl-boston/openpilot.git openpilot
cd openpilot
git fetch -q "$HERE/openpilot-rlx-forwarder-flash.bundle" rlx-forwarder-flash:rlx-forwarder-flash
test "$(git rev-parse rlx-forwarder-flash)" = "$OPENPILOT_FLASH"
$PUSH origin rlx-forwarder-flash
cd ..

echo
echo "Done. Both rlx-forwarder-flash branches are on GitHub."
echo "On the comma: switch to branch rlx-forwarder-flash, plug the red panda into the second USB-C port,"
echo "power-cycle, wait for the red panda's green LED to stay solid on, then unplug it and switch back."
