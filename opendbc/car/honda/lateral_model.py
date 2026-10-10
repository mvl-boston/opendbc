"""Honda lateral plant model: a speed-dependent lateral-accel-per-torque gain times a lateral-accel-
dependent centering shape, identified online and used as an additive feedforward correction on top of
openpilot's torque controller.

Why this exists
---------------
openpilot's torque controller assumes ``lat_accel = latAccelFactor * torque``, one constant for all
speeds. On the Nidec Hondas that is only true on the highway: the MDX 3G delivers ~0.4 m/s^2 per unit
wire torque at 7 m/s and ~2.2 at 28 m/s (routes 0000010e / 0000010f). The controller covers the gap
with a large low-speed proportional gain (``KP_INTERP``), i.e. with feedback that arrives late, which
is why turn entries and return-to-straight at town speeds lag until the driver helps.

What the model does
-------------------
``gain(v)`` is a per-speed table of the car's real m/s^2-per-unit-torque, seeded from the two routes
and refined online. ``shape(|lat_accel|)`` multiplies it: the EPS's self-aligning / return torque grows
with lateral acceleration, so a hard turn needs more wire torque per m/s^2 than a gentle one at the same
speed (the "centering force" the driver has to overcome near the lateral limit). The shape is 1.0
through the gentle band that defines what ``gain(v)`` means and a learned table above it. Each tick the
car controller adds

    correction = -desired_lat_accel / (gain(v) * shape(|desired_lat_accel|)) - (-desired_lat_accel / latAccelFactor)

to the controller's request, i.e. it swaps the controller's linear feedforward for the model's. The
P/I feedback path passes through with gain exactly 1, so the model cannot change the loop gain and
cannot destabilize the loop no matter what it has learned; a wrong table only costs tracking error,
which the controller's feedback then removes. This is the difference from a multiplicative shaper on
the summed output, which scales feedback and feedforward together (the highway limit cycle in route
0000010f was that).

How it learns
-------------
Plant identification rather than tracking-error integration: the measured lateral accel is regressed
on the wire torque that actually went to the EPS (delayed by the actuator delay, both low-passed at
``FILTER_TAU`` so only the quasi-static content is fitted), with a normalized LMS update spread over
the two neighboring speed bins. It identifies a bounded physical quantity, so it cannot drift the way
an integrator of delay-lag error does, and it is indifferent to rate limiting, clipping and saturation
because those all act on the wire torque it regresses against. The only unmodelled input is the
driver, so learning pauses while ``steeringPressed`` and for ``PRESS_HOLDOFF`` afterwards.

The two tables are trained on disjoint data so they cannot trade scale: the speed table learns only
inside the anchor band (gentle curves, where shape == 1 by definition) and the shape table only above
it, treating the gain as known. The shape is bounded at 1.0 and non-increasing in lateral accel: a
saturating EPS never delivers more per unit torque in a hard turn than in a gentle one. Route 0000011f
measured it at 29-36 mph: 2.7 m/s^2 per unit of lagged wire torque below 0.3, 1.9 at 0.4-0.5, 1.4 at
0.7-0.8, 1.27 at 0.8-0.9 (shape 1.0 -> 0.5 between 0.85 and 1.05 m/s^2), and 1.1-1.3 m/s^2 with the
wire pinned at 433 counts. The tables persisted before this bound held 1.5-1.6 there, which put the
feedforward *below* the linear one exactly where the car was undershooting with the wire pinned.

The shape path uses its own short filter (``SHAPE_FILTER_TAU``) on the lag-aligned pair rather than the
``FILTER_TAU`` one the speed table uses: hard-turn dwells between the driver's corrections last 0.5-1.5
s (route 0000011f: 136 pinned, unpressed dwells, median 0.06 s, none over 1.5 s), and a 2 s filter with
a quasi-static gate never admitted one (0 pinned samples in that route). The lag model does the work
instead; a trend gate on the filtered pair (``SHAPE_MAX_LEARN_JERK`` / ``SHAPE_MAX_LEARN_TORQUE_RATE``)
drops the S-bend transitions where it is least exact. Sub-threshold hand torque does not bias the
sample (pinned frames with |driver torque| 0-50 vs 200-300 counts measured 1.08 vs 1.14 m/s^2), and the
estimate does not move with a longer press hold-off (0.5-2.0 s), so ``PRESS_HOLDOFF`` stays short.

The correction is a function of the desired lateral accel and speed only, with no state of its own, so
the return from a hard turn does not depend on anything the torque controller's integrator accumulated
while the driver was overriding (openpilot freezes that integrator on ``steeringPressed``; the fork adds
a reset). With the shape below 1 above the anchor band the model's feedforward for a hard turn is larger
than the linear one and falls steeply as the desired lateral accel drops back through the knee, so the
request leaves the torque limit as soon as the planner asks for less than the car can deliver, before
the feedback has seen any error.

The correction is added to the controller's output, which is already clipped to unit torque. When that
output is saturated the P+I+F sum is beyond the clip, so a correction that opposes it (model gain above
``latAccelFactor``) would take torque away while the loop asks for more than exists; it fades out over
the last ``FF_SATURATION_FADE`` of request headroom.

Wire limits: ISO 11270 in lateral-accel space instead of a torque rate
-------------------------------------------------------------------
openpilot bounds lateral jerk and lateral accel to ISO 11270 (``opendbc/car/lateral.py``: 3.0 m/s^2,
5.0 m/s^3; the planner's ``clip_curvature`` uses the same two numbers; the angle-car safety check in
``opendbc/safety/lateral.h`` uses 3.0 + 0.06 g of road-roll tolerance for both). On torque cars that
target has been implemented as a per-car torque rate, ``STEER_DELTA_UP/DOWN``, and
``opendbc/car/tests/test_lateral_limits.py`` checks the rate against the jerk targets assuming the
plant is linear: ``lat_accel = MAX_LAT_ACCEL_MEASURED * torque`` at every speed. On the Nidec Hondas the
plant is not: the same torque rate is 0.3 m/s^3 at 5 mph (30x under the limit, so the wheel unwinds no
faster than the rate limiter lets it) and 7-8 m/s^3 at 35 mph (above it). The Honda rate limiter also
ran in normalized torque per tick (``STEER_DELTA * DT_CTRL`` = 0.03/tick, full scale in 0.33 s) while
the test evaluated it in CAN counts per tick (3/433), so the test never saw the number the car got.

``limit()`` replaces that rate limiter. It maps the last wire torque (through the EPS response shape
below, i.e. the part of it the EPS acted on) and the new request into lateral accel with the identified
speed gain, applies the ISO jerk and accel bounds there, and maps back through the inverse shape, all
before the torque is scaled into CAN counts. The bound is therefore a constant vehicle response at
every speed: the allowed effective torque rate is ``MAX_LAT_JERK / gain(v)`` per second, small on the
highway where a unit of torque buys 2.5 m/s^2 and large in town where it buys 0.3. Only the speed table
is used, not the centering shape: a shape error scales the bound directly, and whether the saturation
is a function of lateral accel (as the shape table assumes) or of torque (the response shape, which has
turned out to be the larger effect on both cars measured) is not settled per car. The wire torque is
not the vehicle's lateral accel: it goes through ``WIRE_DELAY`` and ``PLANT_TAU`` first, so the bound
applied here is on the quasi-static lateral accel the wire commands, which is how ``test_lateral_limits``
defines it and an upper bound on what the car does (route 0000011f measured lateral jerk p99 0.9 m/s^3
against a wire-implied p99 of 6.9 under the old limiter).

``WIRE_RATE_MAX`` is a separate backstop on the normalized torque rate, applied to the raw wire after
the inverse shape. It is not an ISO term: below ~15 mph the gain is small enough that the jerk bound
alone would let the wire swing full scale in a few ticks, and the EPS and the driver's hands see the
torque step itself. It permits a full swing in 0.1 s (the old limiter took 0.33 s) and never binds
above ~15 mph.

Reporting, and staying compatible with an unmodified openpilot
--------------------------------------------------------------
``gain_now`` (effective m/s^2 per unit torque at the current speed and desired lateral accel, i.e.
``gain(v) * shape``), ``applied_correction`` (torque actually added this tick), ``ceiling`` (start of
the wire band the EPS no longer answers to, 1.0 if there is none) and ``learning`` (identification ran
this tick) are exposed for the actuatorsOutput telemetry slots: gas is the gain, brake the correction,
speed the ceiling (0.4-1.0) plus 2.0 while learning.

controlsd freezes the torque controller's integrator whenever ``|actuators.torque - actuatorsOutput.torque|``
exceeds 0.01 (its ``steer_limited_by_safety``), and torqued fits ``latAccelFactor`` to
``-actuatorsOutput.torque``. If the car controller reported the raw wire torque, the correction would
trip that check on nearly every engaged tick (93-96% in routes 0000010e/0000010f, integrator |I| stuck
near 0.03) and torqued would learn the highway plant gain, which the model then has to fight. So the
car controller reports ``request + (effective_wire(wire) - corrected request)``: the request plus only
the limiting that actually happened, in effective torque (rate limiter, clips, and the part of the wire
the EPS did not answer to). The integrator then freezes only on real limiting, including the EPS's own,
and torqued sees the plant *as corrected by this model*, whose feedforward is ``latAccelFactor`` by
construction, so its live estimate settles on the same number the correction is computed against. The
effective wire torque is recoverable from the log as ``actuatorsOutput.torque + actuatorsOutput.brake``;
the raw wire is on the CAN (``torqueOutputCan`` on the Bosch cars).

One case is reported as the request and not as limiting (``reported_torque``): the controller's request is
itself at the unit clip and the wire is at the bound in force in its direction. The EPS's shortfall there
is real, but reporting it keeps ``steer_limited_by_safety`` set for as long as the car is at its limit, and
controlsd's ``_check_saturation`` does not count while that flag is set, so the ``steerSaturated`` alert
("turn exceeds steering limit") could never fire on a car whose response shape is below linear: Civic 2022
routes 729a2e65b1f6201d|00000040..43 held the wire at 5120 for 8-16 s with the request at 1.0 and the
torque reported at ``effective_wire(1.0)`` = 0.64-0.74 and no alert (route 42 at 11 m/s, desired 3.4 m/s^2
against 2.3 delivered, hands off). The integrator loses nothing: at its own clip the PID does not wind up
(``common/pid.py`` holds ``i`` wherever the sum is already past the limit), which is all the freeze was
for. torqued is unaffected, it takes samples at ``|torque| <= 0.5`` only. Below the clip the shortfall is
still reported as limiting (the integrator stays frozen while the wire is pinned and the request is not at
1.0; letting it wind there is the exit-reversal lag the freeze exists to avoid), and the alert then needs
the request to reach the clip on P and F alone.

EPS wire response shape
-----------------------
The EPS does not answer to the whole of the torque range the car controller can command, and not
linearly to the part it does answer to. On the MDX 3G the steer rate a wire increment buys, and the
angle and lateral accel the car settles at, all stop depending on the wire above ~0.5-0.55 of
STEER_MAX (215-240 of 433 counts; 233 is also the number the EPS faults above while braking): the same
bend at the same speed (route 00000127 21:32:25 vs 00000058) reached the same 17.5 deg / 1.3 m/s^2 with
the wire pinned at 433 as with the wire at 300-400. On the Acura Integra (route 00000139, STEER_MAX
5120) the steer rate a wire increment buys is linear to ~2500-3000 counts and a few percent of that from
there to 5120, in every steering-angle band including 0-4 deg where the self-aligning torque is
negligible, so it is a function of the command level and not of the angle. The lateral accel in that
drive keeps growing for 1-2 s after the wire pins at 5120 (525-532 s): that is the plant lag carrying
the car to the level the first ~2600-3000 counts command, not the torque above it buying more, and the
per-band fit below is what separates the two readings (a marginal response of 0.03-0.2 above 0.6 of
STEER_MAX, against 1.0 below it).

Whether that is a clamp in the EPS firmware or an assist limit the self-aligning torque balances against
makes no difference to the controller, but the shape of it does: a cutoff and a reduced slope want
different handling. Above a cutoff the wire buys nothing and should be bounded. Through a reduced slope
every unit of response costs more wire, and a controller that does not know it runs at a lower loop gain
there (sluggish, then the integrator winds up) and at the full one again as soon as the wire comes back
below the knee (the overshoot on the exit); and every layer that assumes full linear authority accounts
wrongly (the integrator only freezes at a wire of 1.0, the jerk limiter spends its budget ramping the
wire through torque the EPS ignores, the wire has to unwind through that band before the car feels
anything on the exit, torqued and the tables here regress on torque that was never delivered).

``effective_wire(w)`` is the model of it: the wire in bands of STEER_MAX (``WIRE_CUTS``: 0-0.4,
0.4-0.5, ... 0.9-1.0), each with a marginal response ``wire_gains[k]`` in [0, 1] relative to the first
band, whose 1.0 defines the unit the gain table is in; the gains are non-increasing in wire level (a
saturating EPS never delivers more per unit at a higher command than at a lower one). A clamp at 0.55
is (1, 1, 0.5, 0, 0, 0, 0); a linear car is all ones, which is the prior; the car controller seeds the
cars that have been measured. Everything downstream is in effective torque: the gain and centering
tables regress on it (the speed table skipping the samples the shape bent, ``GAIN_MAX_CLIP``); the
torque controller's output is taken as effective torque (torqued fits ``latAccelFactor`` to the
effective torque this reports, so that is the unit it already works in); and ``limit()`` bounds it in
lateral-accel space and maps it to the wire through the inverse shape, sending ``1 / wire_gains[k]`` of
wire per unit of effective torque in band k, so the loop gain the controller sees is the same in every
band. That inverse slope is capped at ``1 / WIRE_INVERT_MIN_GAIN`` (2x): the stability margin of this
loop is a factor of 3-4.5 (route 0000010f's limit cycle), a learned gain that is too low in a band the
car actually answers to is amplified into exactly that, and 2x is as far as the inversion goes; beyond
it the shortfall is reported as limiting and the integrator freezes (``limit()`` itself tracks the last
wire through the floored slopes, ``commanded_wire``, so the jerk budget walks the wire through a weak
band at the floor's rate instead of stalling at the first wire the band swallows). A band with a gain below
``WIRE_DEAD_GAIN`` is dead: ``ceiling`` is the start of the first dead band and the wire is bounded at
``ceiling + WIRE_PROBE``, the probe band keeping the learner able to see whether torque there does
something after all (on a car whose persisted shape is lower than its real one the band above the
ceiling then reads alive and the ceiling walks up; on a car whose band really is dead it costs
nothing). With no dead band the wire runs to 1.0: the steering power above the knee is kept, delivered
through the inverse at up to twice the wire per unit the linear part needs.

``_identify_wire`` learns the shape from the steer rate, which answers the wire within ``RATE_DELAY``
(far shorter than the lateral-accel lag, so transients are usable and a wire that never holds a level,
which is how the torque controller drives, is enough). It keeps exponentially weighted normal equations
(``RATE_TAU``) of ``steer_rate = sum_k beta_k * band_k(wire) + a * angle + b * lat_accel + d`` over the
ticks with the delayed wire above ``RATE_MIN_WIRE`` at road speed (``RATE_MIN_SPEED``), unpressed. The
angle and lateral-accel terms carry the centering, so a plant that saturates with lateral accel rather
than torque is explained by them and not by the bands. Every ``RATE_SCORE_INTERVAL`` ticks the system is
solved under ``beta_0 >= beta_1 >= ... >= 0`` (projected gradient with a pool-adjacent-violators
projection, warm-started from the last solution). Unconstrained, the band coefficients are
ill-conditioned: a wire that is pinned at 1.0 for part of a turn makes the top band an indicator of
"pinned", which a free fit uses to absorb whatever the centering terms left over; the constraint is what
makes them an estimate of the response. The fit has to explain a fair share of the steering
(``RATE_MIN_R2``) and the first band must turn the wheel its own way; a band moves only while the window
holds samples with the wire in or above it (``WIRE_MIN_BAND_ACTIVITY``) and the fit disagrees with it
by more than ``WIRE_TOL``, toward the fit at ``WIRE_LEARN_RATE`` per counted tick, and the bands above
an informed one inherit its saturation (non-increasing). Replays from the linear prior (see the tests
and the PR notes): the Integra's route 00000139 reads 1.0 to 0.6 of STEER_MAX and ~0.05-0.2 above, the
MDX's 00000127 1.0 to 0.5 and 0.3-0.4 to 0.7, the Integra's angle-PID drive 00000135 (R^2 0.12-0.25) is
rejected by the fit gate and holds.

The first version of this learned one clamp level, from settled dwells of lateral accel and from a bank
of clip candidates on the steer rate. The dwell bank counted zero samples on the Integra's drive (the
torque controller never holds the wire), and a clamp cannot say what a reduced slope does.
``HondaLatCeilingParams`` is still written (the dead-band start) and is read as a clamp when no shape
has been persisted yet, so the MDX's 0.538 carries over.

Known limits: ``steeringPressed`` excludes the driver, and on the MDX that flag is set by the EPS's own
torque-sensor oscillation in hard turns (route 00000127: 76% of the ticks with the wire above 0.42), so
the shape there learns from the gaps between; below ~10 m/s the fit degrades (large angles, the
centering terms least exact) and nothing is learned; one shape is learned for all speeds.

Persisting the tables needs the ``HondaLatGainNNParams``, ``HondaLatShapeNNParams``,
``HondaLatWireNNParams`` and ``HondaLatCeilingParams`` keys registered in openpilot's
``common/params_keys.h`` (``param_keys()`` lists them); unregistered keys are silently dropped by the
param writer and that table simply restarts from the priors each drive.
"""
from collections import deque
from math import copysign

import numpy as np

from opendbc.car import DT_CTRL
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.lateral import ISO_LATERAL_ACCEL, ISO_LATERAL_JERK

# speed bins (mph, for readable Params keys) and the seed gains in m/s^2 per unit wire torque, identified
# offline on routes 3792d010590cb83a|0000010e, |0000010f (torque controller) and |00000111 (PID controller,
# no shaper: 20 mph 1.0-1.3, 30 mph 2.2-2.4, 40 mph 2.6-2.9 agree with the torque routes within ~30%)
GAIN_BINS_MPH = (5, 10, 15, 20, 30, 40, 50, 60, 70)
GAIN_PRIOR = (0.10, 0.25, 0.45, 0.80, 1.90, 2.40, 2.10, 2.30, 2.60)
GAIN_BINS_MS = tuple(mph * CV.MPH_TO_MS for mph in GAIN_BINS_MPH)
GAIN_MIN = 0.10
GAIN_MAX = 3.00
GAIN_KEY_FMT = "HondaLatGain{slot:02d}Params"

# Centering shape: the EPS's self-aligning / return torque grows with lateral acceleration, so the m/s^2
# delivered per unit wire torque is not the same in a hard turn as in a gentle one. shape(|lat_accel|)
# multiplies gain(v): the effective plant is lat_accel = gain(v) * shape(|lat_accel|) * torque. The shape
# is anchored at 1.0 for |lat_accel| <= SHAPE_ANCHOR_LAT_ACCEL (that band defines what gain(v) means) and
# learned above it in the bins below, so the two tables cannot trade scale. Priors are neutral (1.0). The
# table is bounded at 1.0 and kept non-increasing in lateral accel (a saturating EPS: each unit of torque
# buys less in a hard turn than in a gentle one, never more), so the extra torque at high lateral accel
# is at most 1 / SHAPE_MIN times the linear feedforward. Route 0000011f measured 0.5-0.7 at 1.0-1.3
# m/s^2 (29-36 mph); a persisted value above 1.0 clips to 1.0 at load.
SHAPE_ANCHOR_LAT_ACCEL = 0.75                     # m/s^2, shape == 1.0 at and below this
SHAPE_BINS_LAT_ACCEL = (1.0, 1.5, 2.0, 2.5, 3.0)  # m/s^2, learned bins (keys use tenths: 10, 15, ...)
SHAPE_PRIOR = (1.0, 1.0, 1.0, 1.0, 1.0)
SHAPE_MIN = 0.40
SHAPE_MAX = 1.00
SHAPE_KEY_FMT = "HondaLatShape{slot:02d}Params"
SHAPE_LEARN_RATE = 0.005                          # normalized LMS step per tick (samples are scarce: ~100-300 per route)
SHAPE_BINS_INTERP = (SHAPE_ANCHOR_LAT_ACCEL,) + SHAPE_BINS_LAT_ACCEL
# shape samples come from the lag-aligned pair through a short filter with a trend gate (see the module
# docstring): hard-turn dwells between the driver's corrections are 0.5-1.5 s long
SHAPE_FILTER_TAU = 0.30                           # s
SHAPE_MAX_LEARN_JERK = 0.50                       # m/s^3, |filtered lat accel| trend allowed for a shape sample
SHAPE_MAX_LEARN_TORQUE_RATE = 0.50                # normalized torque per second, same for the lagged wire

# identification
FILTER_TAU = 2.0                                  # s, common low-pass on wire torque and measured lat accel
FILTER_ALPHA = DT_CTRL / (FILTER_TAU + DT_CTRL)
SHAPE_FILTER_ALPHA = DT_CTRL / (SHAPE_FILTER_TAU + DT_CTRL)
WIRE_DELAY = 0.30                                 # s, actuator delay applied to the wire before the plant lag
# The car's lateral accel follows the wire as a first-order lag, not instantly: a lag of 1.0-1.3 s (after the
# 0.3 s delay) maximizes the wire / lat-accel correlation at 10-16 m/s in routes 0000010e, 0000010f and
# 00000114, and 1.0 s does so above 16 m/s. Regressing on the lagged wire keeps a sample taken mid-turn
# honest; without it every turn entry read the gain low and every exit read it high.
PLANT_TAU = 1.0
PLANT_ALPHA = DT_CTRL / (PLANT_TAU + DT_CTRL)
LEARN_RATE = 0.002                                # normalized LMS step per tick
LEARN_NORM_EPS = 0.01                             # torque^2, keeps the normalized step finite near zero
MIN_LEARN_TORQUE = 0.15                           # |filtered wire| needed for excitation
MIN_LEARN_LAT_ACCEL = 0.15                        # m/s^2, |filtered lat accel| needed to be in a real curve
# coarse quasi-static gate: neither filtered signal may be changing by more than this fraction of itself
# per FILTER_TAU (guards the sign flips of an S-bend, where the lag model is least exact)
MAX_LEARN_CHANGE = 0.50
MIN_LEARN_SPEED = 2.0                             # m/s, curvature from steering angle is meaningless below
PRESS_HOLDOFF = 0.5                               # s, learning stays paused this long after steeringPressed

# feedforward correction
FF_CORRECTION_MAX = 0.75                          # normalized torque, bound on |model ff - controller ff|
# normalized torque; a correction that opposes the request fades out over this much request headroom below 1.0
FF_SATURATION_FADE = 0.10
DEFAULT_LAT_ACCEL_FACTOR = 1.8

# wire limits (see the module docstring): the same 3 m/s^2 / 5 m/s^3 the planner holds the desired
# curvature to for every car (selfdrive/controls/lib/drive_helpers.py clip_curvature), applied here to
# the wire so the limit does not depend on the controller upstream. Jerk is symmetric; test_lateral_limits'
# 2.5 m/s^3 up-rate is a comfort margin on top of the same linear calculation, and on this plant the
# 1 s response lag provides that margin. No roll allowance: the planner adds it to the desired curvature,
# and with GAIN_MAX at 3.0 a unit of wire torque cannot command more than 3 m/s^2 anyway.
MAX_LAT_ACCEL = ISO_LATERAL_ACCEL                 # m/s^2
MAX_LAT_JERK_UP = ISO_LATERAL_JERK                # m/s^3, |lat accel| increasing
MAX_LAT_JERK_DOWN = ISO_LATERAL_JERK              # m/s^3, |lat accel| decreasing (return to center)
WIRE_RATE_MAX = 10.0                              # normalized torque per second, EPS / hands-on backstop

# EPS wire response shape (see the module docstring): the marginal response of the EPS per unit of wire torque, in
# bands of STEER_MAX, relative to the first band (whose 1.0 is the unit the gain table is in). The upper edge of each
# band is a cut; band k is [WIRE_CUTS[k-1], WIRE_CUTS[k]). The cuts start where both measured cars are still linear
# (0.4) and the knees sit (MDX 3G 0.50-0.55, Integra 0.55-0.60), in steps a town drive's wire visits often enough
# to tell apart
WIRE_CUTS = (0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0)
WIRE_BAND_LO = (0.0,) + WIRE_CUTS[:-1]
WIRE_BAND_WIDTH = tuple(hi - lo for lo, hi in zip(WIRE_BAND_LO, WIRE_CUTS, strict=True))
WIRE_BANDS = len(WIRE_CUTS)
WIRE_PRIOR = (1.0,) * (WIRE_BANDS - 1)            # learned bands (all but the first): linear until the car shows otherwise
WIRE_GAIN_MIN = 0.0
WIRE_GAIN_MAX = 1.0
WIRE_KEY_FMT = "HondaLatWire{slot:d}Params"       # slot: the band's upper edge in percent of STEER_MAX (50 .. 100)
# a band whose marginal response is below this is dead: the wire is bounded at its start plus the probe band. Above it
# the wire runs through the band at the inverse gain; the cost of a dead band taken for alive is wire the EPS ignores
# (what the probe band spends anyway), the cost of an alive band taken for dead is authority
WIRE_DEAD_GAIN = 0.05
# the inverse shape never sends more than 1 / this of wire per unit of effective torque: a learned gain that is too
# low in a band the car answers to would otherwise be amplified into loop gain (route 0000010f's limit cycle sat at
# 3-4.5x). Below it the shortfall is reported as limiting and the integrator freezes, as for any saturation
WIRE_INVERT_MIN_GAIN = 0.5
WIRE_PROBE = 0.10                                 # normalized torque the wire may run into the dead band so the learner keeps seeing it
CEILING_KEY = "HondaLatCeilingParams"             # start of the dead band, written for telemetry / older versions; read as a clamp when no shape is persisted
CEILING_MIN = WIRE_CUTS[0]
CEILING_MAX = 1.0
# reporting (see the module docstring, "Reporting"): a request within this of the unit clip is the controller's own
# saturation (latcontrol's check is steer_max - |output| < 1e-3), and a wire within this of the bound in force is at it
REPORT_SATURATION_TOL = 1e-3
# the speed table may not learn from a FILTER_TAU sample the response shape bent by more than this (normalized
# torque): a shape that is too low would otherwise teach it the car's response per unit of a wire it never got,
# and the table is the feedforward and the jerk bound (route 00000129: 0.77 m/s^2 per unit at 10 mph against
# 0.4 measured, half the torque the turns needed)
GAIN_MAX_CLIP = 0.02

# learning the response shape from the steer rate (see the module docstring)
RATE_DELAY = 0.15                                 # s, wire to steer rate (route 00000139: the fit is best at 0.10-0.15 s)
RATE_DELAY_TICKS = max(int(round(RATE_DELAY / DT_CTRL)), 1)
RATE_TAU = 60.0                                   # s of samples; a per-tick regression at R^2 ~0.4 needs the memory
RATE_ALPHA = DT_CTRL / (RATE_TAU + DT_CTRL)
RATE_MIN_WIRE = 0.25                              # normalized, |delayed wire| for a tick to count: the fit needs its linear part
# below ~10 m/s the fit degrades (large angles, the centering terms least exact) and in the Integra's angle-PID
# drive 00000135 the 5-10 m/s rows alone read "linear" against knees the 10-20 m/s rows show
RATE_MIN_SPEED = 10.0                             # m/s
RATE_MIN_SAMPLES = int(round(10.0 / DT_CTRL))     # counted ticks before the first fit (MDX town route 00000127: 14 s above 10 m/s)
RATE_SCORE_INTERVAL = 5                           # ticks between fits
RATE_MIN_RATE_STD = 2.0                           # deg/s, steer-rate spread over the samples before they are fitted
# the fit has to explain a fair share of the steering before its shape is evidence: routes 00000139 and 00000127
# fit at 0.46 / 0.76, the angle-PID drive 00000135 at 0.12-0.25 with a knee within 20% of its STEER_MAX
RATE_MIN_R2 = 0.30
RATE_RIDGE = 1e-9
# regressor scales for the normal equations (band regressors in band widths, angle in ~20 deg, lat accel in ~2 m/s^2),
# so the projected-gradient solve is reasonably conditioned; the fit is scale-free in the result
RATE_SCALE = np.array([0.1] * WIRE_BANDS + [20.0, 2.0, 1.0])
WIRE_FIT_ITERS = 50                               # projected-gradient steps per fit, warm-started from the last fit
# a band moves only while the window holds samples with the wire in or above it: its scaled regressor's mean square
# (0..1, the fraction of the window fully in or above the band) must be at least this
WIRE_MIN_BAND_ACTIVITY = 0.005
WIRE_TOL = 0.03                                   # a band moves only while the fit disagrees with it by more than this
# per counted tick, toward the fit: a town drive counts ~100 s of ticks, so a drive with a clear knee gets most of the
# way there and the next one settles it
WIRE_LEARN_RATE = 0.0002


def _clip(value, lo, hi):
  return float(min(max(value, lo), hi))


def _shape_slot(lat_accel):
  # Params key slot for a shape bin: lateral accel in tenths of m/s^2 (1.5 -> 15)
  return int(round(lat_accel * 10))


def _wire_slot(cut):
  # Params key slot for a wire band: its upper edge in percent of STEER_MAX (0.6 -> 60)
  return int(round(cut * 100))


def _load(param_get, key, default):
  # a missing or not-yet-registered key must never take the car controller down
  if param_get is None:
    return default
  try:
    value = param_get(key)
  except Exception:
    return default
  if value is None:
    return default
  try:
    return float(value)
  except (TypeError, ValueError):
    return default


def _pava_nonincreasing(values):
  """Euclidean projection of a sequence onto {v_0 >= v_1 >= ... >= 0} (pool adjacent violators)."""
  blocks = []
  for v in values:
    blocks.append([float(v), 1])
    while len(blocks) > 1 and blocks[-2][0] < blocks[-1][0]:
      v2, n2 = blocks.pop()
      v1, n1 = blocks.pop()
      blocks.append([(v1 * n1 + v2 * n2) / (n1 + n2), n1 + n2])
  out = []
  for v, n in blocks:
    out.extend([max(v, 0.0)] * n)
  return out


def clamp_shape(ceiling):
  """The response shape of a hard clamp at this normalized wire torque: the learned bands' gains."""
  return tuple(_clip(round((ceiling - lo) / width, 6), WIRE_GAIN_MIN, WIRE_GAIN_MAX)
               for lo, width in zip(WIRE_BAND_LO[1:], WIRE_BAND_WIDTH[1:], strict=True))


class HondaLateralModel:
  def __init__(self, lat_accel_factor, param_get=None, wire_prior=None):
    self.lat_accel_factor = (
      float(lat_accel_factor) if lat_accel_factor and lat_accel_factor > 0.1 else DEFAULT_LAT_ACCEL_FACTOR
    )
    self.gains = [_clip(_load(param_get, GAIN_KEY_FMT.format(slot=mph), prior), GAIN_MIN, GAIN_MAX)
                  for mph, prior in zip(GAIN_BINS_MPH, GAIN_PRIOR, strict=True)]
    self.shapes = [_clip(_load(param_get, SHAPE_KEY_FMT.format(slot=_shape_slot(la)), prior), SHAPE_MIN, SHAPE_MAX)
                   for la, prior in zip(SHAPE_BINS_LAT_ACCEL, SHAPE_PRIOR, strict=True)]
    self._project_shapes()
    # the response shape: persisted bands first; a persisted clamp (the previous version's ceiling) when there are
    # none; the car's seed otherwise; linear when there is no seed
    prior = tuple(WIRE_PRIOR if wire_prior is None else wire_prior)
    persisted = [_load(param_get, WIRE_KEY_FMT.format(slot=_wire_slot(cut)), None) for cut in WIRE_CUTS[1:]]
    if any(p is not None for p in persisted):
      learned = [prior[i] if p is None else p for i, p in enumerate(persisted)]
    else:
      ceiling = _load(param_get, CEILING_KEY, None)
      learned = clamp_shape(ceiling) if ceiling is not None and ceiling < CEILING_MAX else prior
    self.wire_gains = [1.0] + [_clip(g, WIRE_GAIN_MIN, WIRE_GAIN_MAX) for g in learned]
    self._project_wire_gains()
    self.wire_hist = deque([0.0] * max(int(round(WIRE_DELAY / DT_CTRL)), 1), maxlen=max(int(round(WIRE_DELAY / DT_CTRL)), 1))
    self.wire_lag = 0.0             # delayed effective wire through the plant lag: the lateral accel the wire has "earned" so far
    self.clip_lag = 0.0             # what the response shape took off the delayed wire, through the same lag and filter
    self.wire_filt = 0.0            # FILTER_TAU pair, speed table
    self.lat_accel_filt = 0.0
    self.clip_filt = 0.0
    self.wire_fast = 0.0            # SHAPE_FILTER_TAU pair, shape table
    self.lat_accel_fast = 0.0
    # response-shape learner: EW normal equations of steer_rate on [band regressors, angle, lat accel, 1]
    n = len(RATE_SCALE)
    self.rate_hist = deque([0.0] * RATE_DELAY_TICKS, maxlen=RATE_DELAY_TICKS)
    self.rate_xtx = np.zeros((n, n))
    self.rate_xty = np.zeros(n)
    self.rate_sy = 0.0
    self.rate_syy = 0.0
    self.rate_weight = 0.0
    self.rate_samples = 0
    self.rate_beta = np.zeros(n)              # last constrained solution (scaled regressors), the next fit's warm start
    self.rate_r2 = 0.0                        # telemetry: share of the steer-rate variance the last fit explained
    self.rate_slope = 0.0                     # telemetry: fitted deg/s per unit of wire in the first band
    self.wire_fit = np.array(self.wire_gains)  # telemetry: the last fit's shape, relative to the first band
    self.wire_evidence = np.zeros(WIRE_BANDS, dtype=bool)   # bands the last fit disagrees with, on enough data
    self.rate_counted = False       # the learner took a sample this tick
    self.press_holdoff = 0.0
    # telemetry for the last update() call
    self.gain_now = float(np.interp(0.0, GAIN_BINS_MS, self.gains))   # effective gain at (v, |desired lat accel|)
    self.shape_now = 1.0
    self.ff_correction = 0.0        # model ff minus controller ff, before the unit-torque clip
    self.applied_correction = 0.0   # output minus request, i.e. what was actually added this tick
    self.learning = False           # any table updated this tick
    self.learning_gain = False
    self.learning_shape = False
    self.learning_wire = False
    self.output = 0.0
    self.jerk_limited = False       # limit() clipped the wire on the lateral jerk bound this tick
    self.accel_limited = False      # limit() clipped the wire on the lateral accel bound this tick
    self.rate_limited = False       # limit() clipped the wire on the torque-rate backstop this tick
    self.ceiling_limited = False    # limit() clipped the wire at the dead band (plus probe) this tick

  def gain(self, v_ego):
    return float(np.interp(v_ego, GAIN_BINS_MS, self.gains))

  def shape(self, lat_accel):
    """Centering multiplier on gain(v) at this |lateral accel|: 1.0 through the anchor band, then the
    learned table, held at the last bin beyond it."""
    return float(np.interp(abs(lat_accel), SHAPE_BINS_INTERP, [1.0] + self.shapes))

  def effective_gain(self, v_ego, lat_accel):
    return self.gain(v_ego) * self.shape(lat_accel)

  @property
  def ceiling(self):
    """Normalized wire torque at which the EPS stops answering: the start of the first dead band, 1.0 if none."""
    return self.wire_dead_start

  @property
  def wire_limit(self):
    """Normalized torque bound limit() holds the wire to: the dead band's start plus the probe band."""
    return self.wire_bound

  @property
  def wire_linear(self):
    return self._linear

  def _forward(self, torque, gains):
    a = abs(torque)
    if self._linear or a <= WIRE_CUTS[0]:
      return float(torque)
    eff = WIRE_CUTS[0]
    for k in range(1, WIRE_BANDS):
      lo = WIRE_BAND_LO[k]
      if a <= lo:
        break
      eff += gains[k] * min(a - lo, WIRE_BAND_WIDTH[k])
    return copysign(eff, torque)

  def effective_wire(self, torque):
    """The part of a wire torque the EPS acts on, in units of the first band's response."""
    return self._forward(torque, self.wire_gains)

  def commanded_wire(self, torque):
    """The effective torque a wire stands for through the inverse shape's own (floored) slopes: the exact inverse
    of wire_from_effective, i.e. what limit() takes the last wire to have asked for. It is at least effective_wire;
    the difference is the shortfall the inversion floor leaves to the reporting."""
    return self._forward(torque, self.wire_gains_floored)

  def wire_from_effective(self, effective):
    """The wire torque that delivers this effective torque, through the inverse shape at no more than
    1 / WIRE_INVERT_MIN_GAIN of wire per unit, bounded at the dead band plus the probe."""
    return self._invert(effective)[0]

  def reported_torque(self, request, corrected_request, wire, wire_limit):
    """What the car controller reports as actuatorsOutput.torque (see the module docstring, "Reporting").
    request: the controller's output this tick; corrected_request: update()'s output for it (the model's
    feedforward swapped in, before limit()); wire: the wire that went to the EPS (normalized, after
    limit() and any car-specific clip); wire_limit: the bound in force on it (1.0, the dead band plus the
    probe, or a car-specific clip, whichever is lowest). The request plus only the limiting that happened,
    in effective torque; the request itself when it is at the unit clip and the wire is at the bound in its
    direction, which is saturation and not limiting."""
    at_clip = abs(request) >= 1.0 - REPORT_SATURATION_TOL
    at_bound = abs(wire) >= wire_limit - REPORT_SATURATION_TOL and copysign(1.0, wire) == copysign(1.0, request)
    if at_clip and at_bound:
      return float(request)
    return float(request + (self.effective_wire(wire) - corrected_request))

  def _invert(self, effective):
    a = abs(effective)
    if self._linear or a <= WIRE_CUTS[0]:
      wire = a
    else:
      wire = WIRE_CUTS[0]
      rem = a - WIRE_CUTS[0]
      for k in range(1, WIRE_BANDS):
        g = self.wire_gains_floored[k]
        span = g * WIRE_BAND_WIDTH[k]
        if rem <= span:
          wire += rem / g
          rem = 0.0
          break
        wire += WIRE_BAND_WIDTH[k]
        rem -= span
      if rem > 0.0:
        wire = 1.0
    bounded = self.wire_bound < CEILING_MAX and wire > self.wire_bound
    return copysign(min(wire, self.wire_bound), effective), bounded

  def feedforward_correction(self, desired_curvature, v_ego):
    """Torque to add to the controller's request so the feedforward follows gain(v) * shape(|lat_accel|)
    instead of latAccelFactor. Sign: actuators.torque is positive to the right, curvature positive to the left."""
    desired_lat_accel = desired_curvature * v_ego * v_ego
    ff_model = -desired_lat_accel / self.effective_gain(v_ego, desired_lat_accel)
    ff_controller = -desired_lat_accel / self.lat_accel_factor
    return _clip(ff_model - ff_controller, -FF_CORRECTION_MAX, FF_CORRECTION_MAX)

  def update(self, request_torque, wire_torque, lat_active, steer_control_active, steering_pressed, v_ego,
             desired_curvature, current_curvature, steering_angle_deg=0.0, steering_rate_deg=0.0):
    """request_torque: controller output this tick; wire_torque: what went to the EPS last tick (after
    rate limiter and clips). steering_angle_deg / steering_rate_deg: the car's steering sensor, in the
    same sign convention as the wire (a positive wire turns the angle positive). Returns the corrected
    request (effective torque), to be bounded and mapped to the wire by limit()."""
    self._identify(wire_torque, current_curvature, v_ego, lat_active and steer_control_active, steering_pressed,
                   steering_angle_deg, steering_rate_deg)

    desired_lat_accel = desired_curvature * v_ego * v_ego
    self.shape_now = self.shape(desired_lat_accel)
    self.gain_now = self.gain(v_ego) * self.shape_now
    if not lat_active:
      self.ff_correction = 0.0
      self.applied_correction = 0.0
      self.output = float(request_torque)
      return self.output

    self.ff_correction = self.feedforward_correction(desired_curvature, v_ego)
    correction = self.ff_correction
    # The correction swaps feedforwards; it is not a license to undo feedback. A saturated request means
    # the controller's P+I+F sum is beyond the unit clip, so a correction that opposes it (model gain above
    # latAccelFactor) would take torque away while the loop is asking for more than exists: route 00000114
    # 09:23:21 held the wire at 420/433 with the request at -1.00 and the lane lines solid. Fade it out
    # over the last FF_SATURATION_FADE of request headroom rather than switching it off at 1.0.
    if np.sign(correction) == -np.sign(request_torque):
      correction *= _clip((1.0 - abs(request_torque)) / FF_SATURATION_FADE, 0.0, 1.0)
    self.output = _clip(request_torque + correction, -1.0, 1.0)
    self.applied_correction = self.output - float(request_torque)
    return self.output

  def lat_accel_from_torque(self, torque, v_ego):
    """Quasi-static lateral accel an effective torque commands at this speed, speed table only (see module docstring)."""
    return self.gain(v_ego) * float(torque)

  def torque_from_lat_accel(self, lat_accel, v_ego):
    return float(lat_accel) / self.gain(v_ego)

  def limit(self, torque, last_torque, v_ego):
    """Bound the request to ISO 11270 lateral jerk and lateral accel, computed in lateral-accel space with the
    identified speed gain, map it to the wire through the inverse response shape, then apply the torque-rate
    backstop. torque: corrected request this tick (effective torque); last_torque: the wire that went to the EPS
    last tick. Returns the wire torque to send (normalized)."""
    # the last wire stands for the effective torque the inverse mapped to it (its floored slopes, so a weak band is
    # walked through at the floor's rate rather than stalled in: what the EPS delivered short of that is the
    # reporting's business, not the limiter's)
    la_last = self.lat_accel_from_torque(self.commanded_wire(last_torque), v_ego)
    la_req = self.lat_accel_from_torque(torque, v_ego)
    up = MAX_LAT_JERK_UP * DT_CTRL
    down = MAX_LAT_JERK_DOWN * DT_CTRL
    # up-rate while |lat accel| grows, down-rate while it shrinks; a move through zero is down to zero
    # and up beyond it (same structure as apply_driver_steer_torque_limits)
    if la_last > 0.0:
      lo, hi = max(la_last - down, -up), la_last + up
    else:
      lo, hi = la_last - up, min(la_last + down, up)
    la_jerk = _clip(la_req, lo, hi)
    la_out = _clip(la_jerk, -MAX_LAT_ACCEL, MAX_LAT_ACCEL)
    self.jerk_limited = la_jerk != la_req
    self.accel_limited = la_out != la_jerk
    # untouched requests pass through bit-exact; only a bound that bit is mapped back through the gain
    eff_out = float(torque) if la_out == la_req else self.torque_from_lat_accel(la_out, v_ego)
    # the wire that delivers it, through the inverse shape; the dead band plus the probe is the bound
    out, self.ceiling_limited = self._invert(eff_out)
    backstop = _clip(out, last_torque - WIRE_RATE_MAX * DT_CTRL, last_torque + WIRE_RATE_MAX * DT_CTRL)
    self.rate_limited = backstop != out
    return backstop

  def _identify(self, wire_torque, current_curvature, v_ego, active, steering_pressed, steering_angle_deg=0.0,
                steering_rate_deg=0.0):
    raw_delayed_wire = self.wire_hist[0]
    self.wire_hist.append(float(wire_torque))
    rate_delayed_wire = self.rate_hist[0]
    self.rate_hist.append(float(wire_torque))
    # the tables regress on what the EPS acted on; the shape learner below sees the raw wire
    delayed_wire = self.effective_wire(raw_delayed_wire)
    self.wire_lag += PLANT_ALPHA * (delayed_wire - self.wire_lag)
    self.clip_lag += PLANT_ALPHA * ((raw_delayed_wire - delayed_wire) - self.clip_lag)
    # measured lat accel in torque sign convention (right positive) so that lat_accel ~= gain * wire_lag
    measured = -current_curvature * v_ego * v_ego
    # the filter's own step is its rate of change over one tick; scaled to FILTER_TAU it is the fraction
    # of the signal still in transit, which is what the quasi-static gate below looks at
    dx = FILTER_ALPHA * (self.wire_lag - self.wire_filt)
    dy = FILTER_ALPHA * (measured - self.lat_accel_filt)
    self.wire_filt += dx
    self.lat_accel_filt += dy
    self.clip_filt += FILTER_ALPHA * (self.clip_lag - self.clip_filt)
    dxf = SHAPE_FILTER_ALPHA * (self.wire_lag - self.wire_fast)
    dyf = SHAPE_FILTER_ALPHA * (measured - self.lat_accel_fast)
    self.wire_fast += dxf
    self.lat_accel_fast += dyf

    self.press_holdoff = PRESS_HOLDOFF if steering_pressed else max(self.press_holdoff - DT_CTRL, 0.0)
    self.learning_gain = False
    self.learning_shape = False
    self.learning_wire = False
    self.rate_counted = False
    self.learning = False
    if not (active and self.press_holdoff <= 0.0 and v_ego > MIN_LEARN_SPEED):
      return

    # plant: y = gain(v) * shape(|y|) * x, with shape == 1.0 through the anchor band. Gentle curves train
    # the speed table and only the speed table; harder turns train the shape, seeing the gain as known.
    # Both regress on what the EPS actually got, so a pinned wire is a valid sample for the shape; the speed
    # table skips the samples the response shape bent (see GAIN_MAX_CLIP)
    x = self.wire_filt
    y = self.lat_accel_filt
    steady = (abs(dx) * FILTER_TAU / DT_CTRL <= MAX_LEARN_CHANGE * abs(x)
              and abs(dy) * FILTER_TAU / DT_CTRL <= MAX_LEARN_CHANGE * abs(y))
    if (steady and abs(x) > MIN_LEARN_TORQUE and abs(y) > MIN_LEARN_LAT_ACCEL and np.sign(x) == np.sign(y)
        and abs(y) <= SHAPE_ANCHOR_LAT_ACCEL and abs(self.clip_filt) <= GAIN_MAX_CLIP):
      self.learning_gain = True
      pos = float(np.interp(v_ego, GAIN_BINS_MS, range(len(GAIN_BINS_MS))))
      lo = int(np.floor(pos))
      hi = min(lo + 1, len(self.gains) - 1)
      frac = pos - lo
      for idx, weight in ((lo, 1.0 - frac), (hi, frac)):
        if weight <= 0.0:
          continue
        err = y - self.gains[idx] * x
        self.gains[idx] = _clip(self.gains[idx] + LEARN_RATE * weight * err * x / (x * x + LEARN_NORM_EPS), GAIN_MIN, GAIN_MAX)

    # shape: the short-filtered pair, admitted while neither is trending (the lag model has aligned them;
    # what is left out is the S-bend transition it is least exact in)
    xf = self.wire_fast
    yf = self.lat_accel_fast
    trending = (abs(dyf) / DT_CTRL > SHAPE_MAX_LEARN_JERK or abs(dxf) / DT_CTRL > SHAPE_MAX_LEARN_TORQUE_RATE)
    if (not trending and abs(xf) > MIN_LEARN_TORQUE and abs(yf) > SHAPE_ANCHOR_LAT_ACCEL and np.sign(xf) == np.sign(yf)):
      self.learning_shape = True
      xg = self.gain(v_ego) * xf
      # a sample above the bound (more lateral accel than the speed table predicts for this torque) only
      # says "not saturated here"; it counts as a 1.0, not as its ratio, so it cannot outvote the saturated
      # samples in the same bin (route 0000011f: 1000 samples at 0.75-1.0 m/s^2 reading 1.1-1.3 against
      # 60 at 1.0-1.1 reading 0.5)
      yf = float(np.sign(yf) * min(abs(yf), SHAPE_MAX * abs(xg)))
      pos = float(np.interp(abs(yf), SHAPE_BINS_INTERP, range(len(SHAPE_BINS_INTERP))))
      lo = int(np.floor(pos))
      hi = min(lo + 1, len(SHAPE_BINS_INTERP) - 1)
      frac = pos - lo
      for idx, weight in ((lo, 1.0 - frac), (hi, frac)):
        if weight <= 0.0 or idx == 0:   # index 0 is the anchor, fixed at 1.0
          continue
        err = yf - self.shapes[idx - 1] * xg
        self.shapes[idx - 1] = _clip(self.shapes[idx - 1] + SHAPE_LEARN_RATE * weight * err * xg / (xg * xg + LEARN_NORM_EPS),
                                     SHAPE_MIN, SHAPE_MAX)
      self._project_shapes()

    self._identify_wire(rate_delayed_wire, measured, v_ego, steering_angle_deg, steering_rate_deg)
    self.learning = self.learning_gain or self.learning_shape or self.learning_wire

  @staticmethod
  def _wire_bands(wire):
    # band regressors: the part of |wire| inside each band, signed like the wire (sum over bands == wire)
    a = abs(wire)
    s = 1.0 if wire >= 0.0 else -1.0
    return [s * min(max(a - lo, 0.0), width) for lo, width in zip(WIRE_BAND_LO, WIRE_BAND_WIDTH, strict=True)]

  def _identify_wire(self, delayed_wire, lat_accel, v_ego, angle_deg, rate_deg):
    # a tick counts while the delayed wire is up where the fit has something to compare the bands against: the band
    # between RATE_MIN_WIRE and the first cut anchors the response the ones above are measured relative to
    self.rate_counted = abs(delayed_wire) > RATE_MIN_WIRE and v_ego > RATE_MIN_SPEED
    if not self.rate_counted:
      return
    x = np.array(self._wire_bands(delayed_wire) + [float(angle_deg), float(lat_accel), 1.0]) / RATE_SCALE
    y = float(rate_deg)
    alpha = RATE_ALPHA
    self.rate_xtx += alpha * (np.outer(x, x) - self.rate_xtx)
    self.rate_xty += alpha * (x * y - self.rate_xty)
    self.rate_sy += alpha * (y - self.rate_sy)
    self.rate_syy += alpha * (y * y - self.rate_syy)
    self.rate_weight += alpha * (1.0 - self.rate_weight)
    self.rate_samples += 1
    if self.rate_samples >= RATE_MIN_SAMPLES and self.rate_samples % RATE_SCORE_INTERVAL == 0:
      # a fitting tick: the verdict replaces the last one, so a fit that stops qualifying stops moving the shape
      self.wire_evidence = self._fit_wire()
    if self.wire_evidence.any():
      self.learning_wire = True
      for k in range(1, WIRE_BANDS):
        if self.wire_evidence[k]:
          self.wire_gains[k] += WIRE_LEARN_RATE * (float(self.wire_fit[k]) - self.wire_gains[k])
      self._project_wire_gains()

  def _fit_wire(self):
    none = np.zeros(WIRE_BANDS, dtype=bool)
    w = self.rate_weight
    var_y = self.rate_syy / w - (self.rate_sy / w) ** 2
    # no steering going on over the samples: nothing to explain
    if var_y < RATE_MIN_RATE_STD * RATE_MIN_RATE_STD:
      return none
    A = self.rate_xtx / w + RATE_RIDGE * np.eye(len(RATE_SCALE))
    b = self.rate_xty / w
    # least squares under beta_0 >= beta_1 >= ... >= beta_{n-1} >= 0 on the band coefficients (all in the same units:
    # deg/s per band width), by projected gradient from the last solution (the first one from the unconstrained
    # solution, projected)
    step = 1.0 / float(np.linalg.eigvalsh(A)[-1])
    beta = self.rate_beta.copy()
    if not beta.any():
      try:
        beta = np.linalg.solve(A, b)
      except np.linalg.LinAlgError:
        return none
      beta[:WIRE_BANDS] = _pava_nonincreasing(beta[:WIRE_BANDS])
    for _ in range(WIRE_FIT_ITERS):
      beta -= step * (A @ beta - b)
      beta[:WIRE_BANDS] = _pava_nonincreasing(beta[:WIRE_BANDS])
    self.rate_beta = beta
    sse = self.rate_syy / w - 2.0 * float(b @ beta) + float(beta @ A @ beta)
    self.rate_r2 = 1.0 - sse / var_y
    k0 = float(beta[0])
    self.rate_slope = k0 / RATE_SCALE[0]
    # the wire must move the wheel its own way, and the fit must explain a fair share of the steering, or the samples
    # are something else (the driver below the press threshold, a sign convention error, a drive the delay model does
    # not fit); a band is informed only if the window holds samples with the wire in or above it
    if k0 <= 0.0 or self.rate_r2 < RATE_MIN_R2:
      return none
    self.wire_fit = np.clip(beta[:WIRE_BANDS] / k0, WIRE_GAIN_MIN, WIRE_GAIN_MAX)
    activity = np.diag(A)[:WIRE_BANDS]
    evidence = (activity >= WIRE_MIN_BAND_ACTIVITY) & (np.abs(self.wire_fit - np.array(self.wire_gains)) > WIRE_TOL)
    evidence[0] = False
    return evidence

  def _project_shapes(self):
    # non-increasing in lateral accel, from the anchor's 1.0 down: a bin with no data of its own inherits
    # the saturation the last measured one showed rather than the neutral prior
    ceiling = SHAPE_MAX
    for i, s in enumerate(self.shapes):
      ceiling = min(ceiling, s)
      self.shapes[i] = ceiling

  def _project_wire_gains(self):
    # non-increasing in wire level from the first band's 1.0 down, same rule as the centering shape; then the
    # derived quantities: the dead band's start (the ceiling), the wire bound, and whether the shape is linear
    ceiling = WIRE_GAIN_MAX
    self.wire_gains[0] = WIRE_GAIN_MAX
    for k in range(1, WIRE_BANDS):
      ceiling = min(ceiling, _clip(self.wire_gains[k], WIRE_GAIN_MIN, WIRE_GAIN_MAX))
      self.wire_gains[k] = ceiling
    self.wire_dead_start = CEILING_MAX
    for k in range(1, WIRE_BANDS):
      if self.wire_gains[k] < WIRE_DEAD_GAIN:
        self.wire_dead_start = WIRE_BAND_LO[k]
        break
    self.wire_bound = _clip(self.wire_dead_start + WIRE_PROBE, CEILING_MIN, CEILING_MAX)
    self.wire_gains_floored = [max(g, WIRE_INVERT_MIN_GAIN) for g in self.wire_gains]
    self._linear = self.wire_gains[-1] >= WIRE_GAIN_MAX

  def learned_values(self):
    values = {GAIN_KEY_FMT.format(slot=mph): float(g) for mph, g in zip(GAIN_BINS_MPH, self.gains, strict=True)}
    values.update({SHAPE_KEY_FMT.format(slot=_shape_slot(la)): float(s)
                   for la, s in zip(SHAPE_BINS_LAT_ACCEL, self.shapes, strict=True)})
    values.update({WIRE_KEY_FMT.format(slot=_wire_slot(cut)): float(g)
                   for cut, g in zip(WIRE_CUTS[1:], self.wire_gains[1:], strict=True)})
    values[CEILING_KEY] = float(self.ceiling)
    return values

  @staticmethod
  def param_keys():
    return ([GAIN_KEY_FMT.format(slot=mph) for mph in GAIN_BINS_MPH] +
            [SHAPE_KEY_FMT.format(slot=_shape_slot(la)) for la in SHAPE_BINS_LAT_ACCEL] +
            [WIRE_KEY_FMT.format(slot=_wire_slot(cut)) for cut in WIRE_CUTS[1:]] +
            [CEILING_KEY])
