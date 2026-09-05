#!/bin/bash
set -e

BASEDIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"

# TODO: why doesn't uv do this?
export PYTHONPATH=$BASEDIR

# *** dependencies install ***
if ! command -v uv &>/dev/null; then
  echo "'uv' is not installed. Installing 'uv'..."
  curl -LsSf https://astral.sh/uv/install.sh | sh

  # must source this after install on some platforms
  if [ -f $HOME/.local/bin/env ]; then
    source $HOME/.local/bin/env
  fi
fi

export UV_PROJECT_ENVIRONMENT="$BASEDIR/.venv"
synced=0
for delay in 0 300 300 300 300; do
  [ "$delay" -gt 0 ] && echo "uv sync failed (likely Hugging Face rate limit), retrying in ${delay}s..." && sleep "$delay"
  if uv sync --all-extras --all-groups --inexact; then
    synced=1
    break
  fi
done
[ "$synced" -eq 1 ] || { echo "uv sync failed after retries"; exit 1; }
source "$PYTHONPATH/.venv/bin/activate"
