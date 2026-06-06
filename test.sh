#!/bin/bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
cd "$DIR"

uv lock --check

lefthook run test

GREEN='\033[0;32m'
NC='\033[0m'
printf "\n${GREEN}All good!${NC} Finished lint and test in ${SECONDS}s\n"
