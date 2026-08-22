#!/usr/bin/env bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
cd $DIR

if ! command -v "mull-runner-17" > /dev/null 2>&1; then
  sudo apt-get update && sudo apt-get install -y curl clang-17
  curl -1sLf -o /tmp/mull-17.deb 'https://github.com/mull-project/mull/releases/download/0.34.0/Mull-17-0.34.0-LLVM-17.0.6-ubuntu-amd64-24.04.deb'
  sudo apt-get install -y /tmp/mull-17.deb
fi
