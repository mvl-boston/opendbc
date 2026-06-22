# Author OpenPilot's lead car on the Honda Bosch radarless dash via HUD_OBJECTS.
# OP's lead goes in slot 0 (replacing the camera's lead), and the camera's
# Adjacent non-lead cars are forwarded in slots 1-9
import math

NUM_SLOTS = 10

# TRACK_INDEX cycle, mirroring lane_path.MUX_CYCLE: 10 slots x 4 redundant banks, bank-major. Slot = (track_index - 1) % 16.
TRACK_INDEX_CYCLE = tuple(slot + bank * 16 for bank in range(4) for slot in range(1, NUM_SLOTS + 1))

# Byte-faithful empty-slot values (decoded from stock HUD_OBJECTS); an inconsistent frame risks the dash rejecting it.
INACTIVE = {
  "OBJECT_ID": 0,
  "IS_LEAD_CAR": 0,
  "CAR_TYPE": -1,
  "ROTATION": -128,
  "LONG_DIST": 196.9,    # raw 1023 = empty
  "LAT_DIST": 204.7,     # max
}

CAR_TYPE_CAR = 7
LONG_DIST_MAX_M = 194.0  # keep an active lead below the 196.9 empty sentinel
LAT_DIST_LIM_M = 204.7   # 12-bit signed @0.1 -> ±204.x m

# The dash renders LAT_DIST in the ego frame but under-scales it ~0.3x
# So scale OP's lead yRel by LAT_SCALE to land the lead car marker on our lane
LAT_SCALE = 0.35

# Used when no rotation available from camera (OP disengaged)
ROT_BAND_M = 2.5         # m of lateral offset per rotation step
ROT_MAX = 3              # ±3 -> ±30 deg

# OP has no persistent lead identity, so LeadObjectId mints a new OBJECT_ID on a fresh lead or a range discontinuity
# (a handoff to a different car). dRel is noisy, so instead of a per-sample range-rate test we run a leaky predictor
# (feed-forward vRel, leak toward dRel) and re-id only when the residual accumulates past REID_GAP_M.
REID_GAP_M = 8.0         # m, accumulated |dRel - predicted| above this = a different car
REID_TAU = 1.5           # s, predictor leak time-constant
REID_REFRACTORY = 1.5    # s, collapse a multi-frame transition into one re-id
MAX_OBJECT_ID = 31       # OBJECT_ID is 5-bit (1..31; 0 = empty)


class LeadObjectId:
  """Tracks a stable slot-0 OBJECT_ID for OP's lead, re-IDing on a fresh lead or a range discontinuity."""
  def __init__(self):
    self.object_id = 0
    self._on = False
    self._pred = 0.0      # leaky predicted dRel
    self._prev_t = 0.0
    self._reid_t = -1e9

  def update(self, status: bool, d_rel: float, v_rel: float, now: float) -> int:
    """Returns the OBJECT_ID to send for slot 0 (0 when there's no lead)."""
    if not status:
      self.object_id = 0
      self._on = False
      return 0

    new_lead = not self._on
    if self._on:
      dt = max(now - self._prev_t, 1e-3)
      self._pred += v_rel * dt                          # feed-forward: d(dRel)/dt ~= vRel
      self._pred += min(dt / REID_TAU, 1.0) * (d_rel - self._pred)   # leak toward the measurement
      if abs(d_rel - self._pred) > REID_GAP_M and now - self._reid_t > REID_REFRACTORY:
        new_lead = True
    self._prev_t = now

    if new_lead:
      self.object_id = self.object_id % MAX_OBJECT_ID + 1   # wrap 1..31, never 0
      self._reid_t = now
      self._pred = d_rel                                    # reset the predictor to the new lead
    self._on = True
    return self.object_id


# LeadSmoother stabilizes the rendered lead marker without lagging real motion: feed vRel forward for dRel (no lag on
# approach/pull-away) then leak toward the measurement to reject jitter, with the residual clamped against outlier
# spikes and feed-forward gated off near zero vRel (a stopped lead is a pure low-pass); yRel is a plain low-pass. It
# snaps (not smooths) on a lead identity change.
DREL_SMOOTH_TAU = 0.6     # s, dRel leak time-constant
YREL_SMOOTH_TAU = 0.5     # s, yRel low-pass time-constant
FF_VREL_MIN = 0.5         # m/s, feed vRel forward only above this
DREL_RESID_CLAMP = 1.5    # m, cap the leak's input residual so an outlier spike barely moves it


class LeadSmoother:
  """Smooths the lead's (dRel, yRel) for a stable dash marker. Call update() per TX tick; it snaps on a new id."""
  def __init__(self):
    self._id = 0
    self._d = 0.0
    self._y = 0.0
    self._t = 0.0

  def update(self, d_rel: float, y_rel: float, v_rel: float, object_id: int, now: float) -> tuple[float, float]:
    """Returns the smoothed (d_rel, y_rel) to render. `object_id` change = a new car -> snap to it."""
    if object_id != self._id:                 # fresh lead / handoff -> snap to it, don't slide across
      self._id, self._d, self._y, self._t = object_id, d_rel, y_rel, now
      return d_rel, y_rel
    dt = max(now - self._t, 1e-3)
    self._t = now
    if abs(v_rel) >= FF_VREL_MIN:              # feed-forward real motion (no lag); off near zero -> pure low-pass
      self._d += v_rel * dt
    resid = min(max(d_rel - self._d, -DREL_RESID_CLAMP), DREL_RESID_CLAMP)   # clamp -> spike reject
    self._d += (1.0 - math.exp(-dt / DREL_SMOOTH_TAU)) * resid
    self._y += (1.0 - math.exp(-dt / YREL_SMOOTH_TAU)) * (y_rel - self._y)
    return self._d, self._y


def lead_rotation(lateral_left_m: float) -> int:
  """Rotation from a lead's lateral offset. Used for OP's lead when camera isn't feeding a rotation (disengaged).
  Negative is rotating to left, positive is rotating to right.
  """
  magnitude = min(round(abs(lateral_left_m) / ROT_BAND_M), ROT_MAX)
  return -magnitude if lateral_left_m > 0 else magnitude


def create_hud_object(packer, bus, track_index, track):
  """Pack one HUD_OBJECTS frame for TRACK_INDEX `track_index`.

  `track` is None for an inactive slot, else a dict {d_rel, y_rel, object_id, is_lead_car, car_type, rotation}.
  CAR_TYPE/ROTATION are borrowed from the stock camera (OP doesn't provide them); CHECKSUM/COUNTER by the packer.
  """
  values = {"TRACK_INDEX": track_index}
  if track is None:
    values.update(INACTIVE)
  else:
    values.update({
      "OBJECT_ID": int(track["object_id"]),
      "IS_LEAD_CAR": int(track["is_lead_car"]),
      "CAR_TYPE": int(track["car_type"]),
      "ROTATION": int(track["rotation"]),
      "LONG_DIST": min(max(track["d_rel"], 0.0), LONG_DIST_MAX_M),
      "LAT_DIST": min(max(track["y_rel"], -LAT_DIST_LIM_M), LAT_DIST_LIM_M),
    })
  return packer.make_can_msg("HUD_OBJECTS", bus, values)


class HudObjectAuthor:
  """Authors HUD_OBJECTS: OP's lead in slot 0 (stable id via LeadObjectId + dRel/yRel smoothing via LeadSmoother),
  the camera's non-lead cars forwarded in slots 1-9, cycling TRACK_INDEX. The carcontroller calls update() once per
  ~50 Hz tick and sends the returned frame."""
  def __init__(self):
    self._track_id = LeadObjectId()
    self._smoother = LeadSmoother()
    self._lead_id = 0       # OBJECT_ID currently emitted for OP's lead (0 = none)
    self._prev_op_id = 0    # last LeadObjectId id, to detect a fresh lead / handoff

  def _lead_object_id(self, status: bool, op_id: int, stock_lead_id: int | None, in_use: set[int]) -> int:
    """OBJECT_ID to emit for OP's lead: prefer the camera's own lead id, else hold a minted id, re-picking out of
    the forwarded stock ids only on a fresh lead / handoff or a collision. 0 when there is no lead."""
    if not status:
      self._lead_id = 0
    elif stock_lead_id is not None:
      self._lead_id = stock_lead_id
    elif self._lead_id == 0 or op_id != self._prev_op_id or self._lead_id in in_use:
      self._lead_id = next((i for i in range(1, MAX_OBJECT_ID + 1) if i not in in_use), MAX_OBJECT_ID)
    self._prev_op_id = op_id
    return self._lead_id

  def update(self, packer, bus, lead, tracks, frame: int, now: float):
    """`lead` = carControlSP.leadOne; `tracks` = the camera's HudObject snapshot (may be None); `frame` = the
    carcontroller frame counter. Returns one packed HUD_OBJECTS frame for the slot the cycle lands on (OP's lead in
    slot 0, else a forwarded camera adjacent car — including in slot 0 when OP has no lead — else inactive). re-ID +
    smoothing run every tick so their state stays continuous across the non-lead frames."""
    op_id = self._track_id.update(lead.status, lead.dRel, lead.vRel, now)
    stock_lead, in_use = None, set()
    for t in (tracks or ()):
      if not t.valid:
        continue
      if t.is_lead_car:
        stock_lead = t                # the camera's lead -> dropped, but we borrow its id / car_type / rotation
      elif t.slot != 0:
        in_use.add(t.object_id)       # ids of the adjacent cars we forward -> OP's lead id must avoid these
    stock_lead_id = stock_lead.object_id if stock_lead is not None else None
    lead_id = self._lead_object_id(lead.status, op_id, stock_lead_id, in_use)

    d_rel, y_rel = self._smoother.update(lead.dRel, LAT_SCALE * lead.yRel, lead.vRel, lead_id, now)

    track_index = TRACK_INDEX_CYCLE[(frame // 2) % len(TRACK_INDEX_CYCLE)]
    slot = (track_index - 1) % 16
    if slot == 0 and lead.status:
      track = {"d_rel": d_rel, "y_rel": y_rel, "object_id": lead_id, "is_lead_car": 1,
               "car_type": stock_lead.car_type if stock_lead is not None else CAR_TYPE_CAR,
               # disengaged -> no camera rotation; calculate one from the lead's lateral
               "rotation": stock_lead.rotation if stock_lead is not None else lead_rotation(y_rel / LAT_SCALE)}
    # forward slots 1-9 and slot 0 when not a lead
    else:
      st = tracks[slot] if (tracks and slot < len(tracks)) else None
      track = ({"d_rel": st.d_rel, "y_rel": st.y_rel, "object_id": st.object_id, "is_lead_car": 0,
                "car_type": st.car_type, "rotation": st.rotation}
                # never forward the camera's lead: if OP has no lead, the HUD must not flag one OP isn't acting on
               if (st is not None and st.valid and not st.is_lead_car) else None)
    return create_hud_object(packer, bus, track_index, track)
