#!/usr/bin/env bash
set -euo pipefail

root="${GITHUB_WORKSPACE:?}"
cereal_rel="${CEREAL_DIR:-openpilot/cereal}"
cereal_path="${root}/${cereal_rel}"
opendbc_car="${root}/opendbc_repo/opendbc/car"

mkdir -p "${cereal_path}"
rm -f "${cereal_path}/car.capnp"
sed 's|import "/include/c++.capnp"|import "./include/c++.capnp"|' \
  "${opendbc_car}/car.capnp" > "${cereal_path}/car.capnp"
rm -f "${opendbc_car}/car.capnp"
ln -sf "${cereal_path}/car.capnp" "${opendbc_car}/car.capnp"
rm -rf "${opendbc_car}/include"
ln -sf "${cereal_path}/include" "${opendbc_car}/include"
