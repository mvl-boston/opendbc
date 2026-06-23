# Render OpenPilot's road lanes on the Honda Bosch radarless dash
import numpy as np

NUM_INDICES = 10
OFFSETS_PER_INDEX = 4
NUM_PTS = NUM_INDICES * OFFSETS_PER_INDEX  # 40 lateral offsets, near->far

# LANE_PATH carries 40 lateral offsets as 10 MUX indices x 4 offsets each. The camera repeats every index
# across 4 redundant banks: MUX = index + bank*16
# We cycle all 40 MUX values bank-major at ~50 Hz so each index refreshes evenly.
# MUX values: 1-10, 17-26, 33-42, 49-58
# logical index = (mux-1) % 16.
MUX_CYCLE = tuple(idx + bank * 16 for bank in range(4) for idx in range(1, NUM_INDICES + 1))

OFFSET_UNAVAILABLE = 2047  # camera's "no point" sentinel (12-bit signed max)
OFFSET_VALID_MAX = 2046    # clamp real offsets below the sentinel

D_NEAR = 2.0
D_MAX = 100.0
LOOKAHEAD = np.linspace(D_NEAR, D_MAX, NUM_PTS)               # look-ahead distance of each offset
GAIN = 6.27 + 0.0106 * LOOKAHEAD + 0.000354 * LOOKAHEAD ** 2  # raw units per meter of lateral, fit to the stock encoding

LANE_LINE_ON = 3            # LEFT_LANE / RIGHT_LANE shown value
LANE_LENGTH_MAX_VALUE = 33  # full dash reach (have not seen/tested higher)
LANE_WIDTH_DEFAULT = 32


def _encode(lat):
  # lane-center lateral (m, +left) at each LOOKAHEAD -> raw offsets; stock offset = -OP lateral
  raw = np.clip(np.round(-GAIN * np.asarray(lat, dtype=float)), -OFFSET_VALID_MAX, OFFSET_VALID_MAX)
  return [int(v) for v in raw]


def encode_lane_path(x, y):
  """OP lane center (x, y arrays, m, +left) -> 40 raw offsets. All-unavailable if the lane doesn't reach D_MAX."""
  x = np.asarray(x, dtype=float)
  y = np.asarray(y, dtype=float)
  if x.size < 2 or x.max() < D_MAX:
    return [OFFSET_UNAVAILABLE] * NUM_PTS
  return _encode(np.interp(LOOKAHEAD, x, y))


def encode_lane_path_poly(poly, valid=True):
  """OP lane-center cubic [c0, c1, c2, c3] (m, +left) -> 40 raw offsets. On-car path, fit upstream in controlsd."""
  if not valid or len(poly) == 0:
    return [OFFSET_UNAVAILABLE] * NUM_PTS
  return _encode(np.polyval(list(poly)[::-1], LOOKAHEAD))  # polyval wants highest-degree-first


def next_mux(mux):
  try:
    return MUX_CYCLE[(MUX_CYCLE.index(int(mux)) + 1) % len(MUX_CYCLE)]
  except ValueError:
    return MUX_CYCLE[0]


def create_lane_path(packer, bus, offsets, mux):
  """Pack one LANE_PATH frame for `mux` (one of MUX_CYCLE) from the 40-offset array."""
  base = ((mux - 1) % 16) * OFFSETS_PER_INDEX
  values = {
    "MUX": mux,
    "PATH_OFFSET_1": offsets[base + 0],
    "PATH_OFFSET_2": offsets[base + 1],
    "PATH_OFFSET_3": offsets[base + 2],
    "PATH_OFFSET_4": offsets[base + 3],
  }
  return packer.make_can_msg("LANE_PATH", bus, values)


def create_lkas_hud_2(packer, bus, counter_2, reach=1.0, lane_cross=0, left_line=True, right_line=True):
  """Pack one LKAS_HUD_2 frame enabling the dash lane lines."""
  lane_length = max(0, min(LANE_LENGTH_MAX_VALUE, round(reach * LANE_LENGTH_MAX_VALUE)))
  shown = lane_length > 0   # no length -> drop the lane lines
  values = {
    "COUNTER_2": counter_2,
    "SET_ME_X01": 1,
    "LANE_WIDTH_MAYBE": LANE_WIDTH_DEFAULT,
    "LEFT_LANE": LANE_LINE_ON if (shown and left_line) else 0,
    "RIGHT_LANE": LANE_LINE_ON if (shown and right_line) else 0,
    "LEFT_LANE_CROSSED": 1 if (shown and lane_cross < 0) else 0,
    "RIGHT_LANE_CROSSED": 1 if (shown and lane_cross > 0) else 0,
    "LANE_LENGTH": lane_length,
  }
  return packer.make_can_msg("LKAS_HUD_2", bus, values)
