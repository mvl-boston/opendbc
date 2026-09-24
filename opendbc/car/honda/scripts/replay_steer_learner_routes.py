#!/usr/bin/env python3
"""Replay comma routes through SteerTorqueLearner and compare legacy vs depart-frame learning.

Fetches public qlogs from api.comma.ai, walks carControl + carState, and measures whether
multi-pass table learning reduces a simple one-step lateral tracking residual proxy.
"""
from __future__ import annotations

import json
import math
import ssl
import urllib.request
from dataclasses import dataclass

import capnp
import zstandard as zstd

MAX_LAT_ACCEL = 1.8
PLANT_GAIN = 0.35  # one-step lat g nudge per unit shaped-vs-plan torque delta
PASSES = 4
ROUTES = (
  "3792d010590cb83a|000000ee--a9da3e6bde",
  "3792d010590cb83a|000000d9--3172dfdcf4",
)


def _ssl_ctx():
  ctx = ssl.create_default_context()
  ctx.check_hostname = False
  ctx.verify_mode = ssl.CERT_NONE
  return ctx


def decompress_stream(data: bytes) -> bytes:
  dctx = zstd.ZstdDecompressor()
  with dctx.stream_reader(data) as reader:
    return reader.read()


_LOG = None


def load_log_capnp():
  global _LOG
  if _LOG is not None:
    return _LOG
  capnp.remove_import_hook()
  _LOG = capnp.load(
    "/tmp/openpilot/openpilot/cereal/log.capnp",
    imports=["/tmp/openpilot/openpilot/cereal", "/workspace/opendbc/car"],
  )
  return _LOG


def fetch_qlog_urls(route_canonical: str) -> list[str]:
  ctx = _ssl_ctx()
  with urllib.request.urlopen(f"https://api.comma.ai/v1/route/{route_canonical}/files", context=ctx) as r:
    files = json.load(r)
  return list(files["qlogs"])


@dataclass
class Frame:
  torque: float
  desired_curv: float
  actual_curv: float
  v_ego: float
  lat_active: bool
  steer_control_active: bool
  steering_pressed: bool
  angle: float
  rate: float

  @property
  def desired_la(self) -> float:
    return self.desired_curv * self.v_ego * self.v_ego

  @property
  def actual_la(self) -> float:
    return self.actual_curv * self.v_ego * self.v_ego


def extract_frames(route: str) -> list[Frame]:
  log = load_log_capnp()
  ctx = _ssl_ctx()
  frames: list[Frame] = []
  for url in fetch_qlog_urls(route):
    with urllib.request.urlopen(url, context=ctx) as r:
      dat = decompress_stream(r.read())
    last_cc = None
    last_cs = None
    for e in log.Event.read_multiple_bytes(dat):
      w = e.which()
      if w == "carControl":
        last_cc = e.carControl
      elif w == "carState":
        last_cs = e.carState
      elif w == "carOutput" and last_cc and last_cs:
        cc, cs = last_cc, last_cs
        if not cc.latActive:
          continue
        t = float(cc.actuators.torque)
        if abs(t) < 0.08:
          continue
        frames.append(Frame(
          torque=t,
          desired_curv=float(cc.actuators.curvature),
          actual_curv=float(cc.currentCurvature),
          v_ego=float(cs.vEgo),
          lat_active=bool(cc.latActive),
          steer_control_active=True,
          steering_pressed=bool(cs.steeringPressed),
          angle=float(cs.steeringAngleDeg),
          rate=float(cs.steeringRateDeg),
        ))
  return frames


def _learner_api():
  from opendbc.car.honda.steer_torque_learner import (
    LAT_AXIS_FRAME_KEY,
    SteerTorqueLearner,
    lat_pct_depart_frame,
    path_learning_curv_err,
    _clip,
    _depart_center_sign,
  )
  return (LAT_AXIS_FRAME_KEY, SteerTorqueLearner, lat_pct_depart_frame, path_learning_curv_err, _clip, _depart_center_sign)


def legacy_lat_pct(torque: float, actual_la: float, clip) -> float:
  sign = 1.0 if torque > 0.0 else -1.0 if torque < 0.0 else 0.0
  return clip(sign * actual_la / MAX_LAT_ACCEL * 100.0, -100.0, 100.0)


def legacy_curv_err(torque: float, desired_la: float, actual_la: float, clip) -> float:
  sign = 1.0 if torque > 0.0 else -1.0
  return clip(sign * (desired_la - actual_la) / MAX_LAT_ACCEL, -1.0, 1.0)


def index_agreement(frames: list[Frame]) -> dict[str, float]:
  """How often legacy vs depart lat indexing disagrees on active steer."""
  _, _, lat_pct_depart_frame, path_learning_curv_err, clip, depart_sign_fn = _learner_api()
  n = 0
  disagree = 0
  legacy_err_flip = 0
  for f in frames:
    if abs(f.torque) < 0.12:
      continue
    dep = depart_sign_fn(f.angle, f.rate)
    if dep == 0.0:
      continue
    n += 1
    new_pct = lat_pct_depart_frame(f.actual_la, MAX_LAT_ACCEL, dep)
    old_pct = legacy_lat_pct(f.torque, f.actual_la, clip)
    if new_pct * old_pct < 0 and abs(new_pct) > 5 and abs(old_pct) > 5:
      disagree += 1
    e_new = path_learning_curv_err(f.desired_la, f.actual_la, MAX_LAT_ACCEL)
    e_old = legacy_curv_err(f.torque, f.desired_la, f.actual_la, clip)
    if abs(e_new) > 0.05 and abs(e_old) > 0.05 and e_new * e_old < 0:
      legacy_err_flip += 1
  return {
    "frames": n,
    "index_sign_disagree_pct": 100.0 * disagree / max(n, 1),
    "legacy_err_sign_flip_pct": 100.0 * legacy_err_flip / max(n, 1),
  }


def simulate_passes(frames: list[Frame], passes: int) -> dict[str, list[float]]:
  frame_key, SteerTorqueLearner, _, path_learning_curv_err, _, _ = _learner_api()
  learner = SteerTorqueLearner(MAX_LAT_ACCEL)
  residual_log: list[float] = []
  residual_sim: list[float] = []
  boost: list[float] = []

  for pass_i in range(passes):
    pass_res_log: list[float] = []
    pass_res_sim: list[float] = []
    pass_boost: list[float] = []
    for f in frames:
      if f.steering_pressed or f.v_ego < 1.0:
        continue
      shaped = learner.update(
        f.torque, learner.prev_output, f.lat_active, f.steer_control_active, f.steering_pressed,
        f.v_ego, f.desired_curv, f.actual_curv, f.angle, f.rate,
      )
      err_la = abs(f.desired_la - f.actual_la)
      pass_res_log.append(err_la)
      nudge = PLANT_GAIN * (shaped - f.torque) * MAX_LAT_ACCEL
      sim_la = f.actual_la + nudge
      pass_res_sim.append(abs(f.desired_la - sim_la))
      if path_learning_curv_err(f.desired_la, f.actual_la, MAX_LAT_ACCEL) > 0.05:
        pass_boost.append(abs(shaped) / max(abs(f.torque), 1e-3))
    if pass_res_log:
      residual_log.append(sum(pass_res_log) / len(pass_res_log))
      residual_sim.append(sum(pass_res_sim) / len(pass_res_sim))
      boost.append(sum(pass_boost) / len(pass_boost) if pass_boost else 1.0)
  lat_p50_a = learner.lat.alphas.get(50, 0.0)
  lat_n50_a = learner.lat.alphas.get(-50, 0.0)
  return {
    "mean_abs_lat_err_log": residual_log,
    "mean_abs_lat_err_sim": residual_sim,
    "undershoot_boost_ratio": boost,
    "lat_alpha_p50_final": lat_p50_a,
    "lat_alpha_n50_final": lat_n50_a,
    "frame_version": learner.learned_values().get(frame_key),
  }


def main():
  load_log_capnp()
  print("SteerTorqueLearner route replay (depart-frame branch)")
  print(f"plant_gain={PLANT_GAIN} passes={PASSES}\n")
  for route in ROUTES:
    print("=" * 72)
    print(route)
    frames = extract_frames(route)
    print(f"  active steer frames: {len(frames)}")
    if len(frames) < 50:
      print("  skip: too few frames")
      continue
    agree = index_agreement(frames)
    print(f"  legacy vs depart index disagree: {agree['index_sign_disagree_pct']:.1f}% "
          f"({agree['frames']} frames with depart sign)")
    print(f"  legacy vs path learning err sign flip: {agree['legacy_err_sign_flip_pct']:.1f}%")
    sim = simulate_passes(frames, PASSES)
    print("  multi-pass mean |desired-actual| lat (log, unchanged):",
          " -> ".join(f"{x:.4f}" for x in sim["mean_abs_lat_err_log"]))
    print("  multi-pass mean |desired-actual_sim| (shaped nudge):  ",
          " -> ".join(f"{x:.4f}" for x in sim["mean_abs_lat_err_sim"]))
    if len(sim["mean_abs_lat_err_sim"]) >= 2:
      imp = sim["mean_abs_lat_err_sim"][0] - sim["mean_abs_lat_err_sim"][-1]
      pct = 100.0 * imp / max(sim["mean_abs_lat_err_sim"][0], 1e-6)
      print(f"  sim residual improvement pass1->pass{PASSES}: {imp:.4f} m/s² ({pct:.1f}%)")
    print("  undershoot |shaped|/|plan| mean:",
          " -> ".join(f"{x:.3f}" for x in sim["undershoot_boost_ratio"]))
    print(f"  final lat α @ +50% / -50% slots: {sim['lat_alpha_p50_final']:.4f} / {sim['lat_alpha_n50_final']:.4f}")
  print()


if __name__ == "__main__":
  main()
