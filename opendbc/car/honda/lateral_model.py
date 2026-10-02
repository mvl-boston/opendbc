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

``limit()`` replaces that rate limiter. It maps the last wire torque and the new request into lateral
accel with the identified speed gain, applies the ISO jerk and accel bounds there, and maps back, all
before the torque is scaled into CAN counts. The bound is therefore a constant vehicle response at
every speed: the allowed torque rate is ``MAX_LAT_JERK / gain(v)`` per second, small on the highway
where a unit of torque buys 2.5 m/s^2 and large in town where it buys 0.3. Only the speed table is
used, not the centering shape: a shape error scales the bound directly, and whether the saturation
is a function of lateral accel (as the shape table assumes) or of torque (an EPS assist limit; the
knee in route 0000011f sits at ~0.35-0.4 of full torque at the one speed with hard turns) is not yet
settled. With the shape in the bound the unwind from a pinned wire would run the first 0.7 of torque
in ~0.1 s (it buys only ~0.4 m/s^2 there) and the rest at the anchor-band rate; that is the next step
once the shape has been learned on more than one speed. The wire torque is not the vehicle's lateral accel: it goes
through ``WIRE_DELAY`` and ``PLANT_TAU`` first, so the bound applied here is on the quasi-static
lateral accel the wire commands, which is how ``test_lateral_limits`` defines it and an upper bound
on what the car does (route 0000011f measured lateral jerk p99 0.9 m/s^3 against a wire-implied p99
of 6.9 under the old limiter).

``WIRE_RATE_MAX`` is a separate backstop on the normalized torque rate. It is not an ISO term: below
~15 mph the gain is small enough that the jerk bound alone would let the wire swing full scale in a
few ticks, and the EPS and the driver's hands see the torque step itself. It permits a full swing in
0.1 s (the old limiter took 0.33 s) and never binds above ~15 mph.

Reporting, and staying compatible with an unmodified openpilot
--------------------------------------------------------------
``gain_now`` (effective m/s^2 per unit torque at the current speed and desired lateral accel, i.e.
``gain(v) * shape``), ``applied_correction`` (torque actually added this tick), ``ceiling`` and ``learning``
(identification ran this tick) are exposed for the actuatorsOutput telemetry slots: gas is the gain,
brake the correction, speed the ceiling (0.4-1.0) plus 2.0 while learning.

controlsd freezes the torque controller's integrator whenever ``|actuators.torque - actuatorsOutput.torque|``
exceeds 0.01 (its ``steer_limited_by_safety``), and torqued fits ``latAccelFactor`` to
``-actuatorsOutput.torque``. If the car controller reported the true wire torque, the correction would
trip that check on nearly every engaged tick (93-96% in routes 0000010e/0000010f, integrator |I| stuck
near 0.03) and torqued would learn the highway plant gain, which the model then has to fight. So the
car controller reports ``request + (wire - corrected request)``: the request plus only the limiting the
rate limiter / clips actually did. The integrator then freezes only on real limiting, and torqued sees
the plant *as corrected by this model*, whose feedforward is ``latAccelFactor`` by construction, so its
live estimate settles on the same number the correction is computed against. The real wire torque is
recoverable from the log as ``actuatorsOutput.torque + actuatorsOutput.brake``.

EPS torque ceiling
------------------
The EPS does not act on the whole of the torque range the car controller can command. On the MDX 3G
the steering angle and lateral accel the car settles at stop depending on the wire above ~0.5-0.55 of
STEER_MAX (215-240 of 433 counts), and the steer rate a wire increment buys does the same: a first-order
plant fit (steer rate on wire and centering) over 10-18 m/s prefers a saturation at 190-250 counts in
both the torque-controller routes (00000127) and the angle-PID routes before them (00000048, 00000058),
and the fit degrades monotonically as the saturation is moved toward 433. The same bend at the same
speed (route 00000127 21:32:25 vs 00000058) reached the same 17.5 deg / 1.3 m/s^2 with the wire pinned
at 433 as it had with the wire at 300-400; the extra ~100 counts bought nothing. 233 is also the
number the EPS faults above while braking, so the simplest reading is an input clamp in the EPS.

Whether the clamp is in the EPS firmware or an assist limit the self-aligning torque balances against
makes no difference to the controller: above the ceiling the marginal response is zero, and every
layer that assumes 433 counts of authority accounts wrongly (the integrator only freezes at a wire of
1.0, the jerk limiter spends its budget ramping the wire through torque the EPS ignores, the wire has to
unwind through that dead band before the car feels anything on the exit, torqued and the tables here
regress on torque that was never delivered). The number is a property of the car, not of the tuning,
and it is not known for the other Nidec cars, so it is learned per car:

``ceiling`` is the normalized wire torque above which the EPS is taken to deliver nothing more. It is
identified with a bank of candidates (``CEILING_CANDIDATES``): each candidate runs the raw wire clipped
at its own value through the same delay / plant lag / short filter as the shape path, and the bank keeps
exponentially weighted moments of (|clipped wire|, |lateral accel|) over the samples on which the
candidates disagree: the lagged wire above the lowest candidate, in a real curve, while the wire is
holding a level and the lag-aligned signals have settled (``CEILING_DWELL_TOL``, ``CEILING_MAX_LEARN_*``).
Each candidate is scored by the least-squares line through its samples, as the fraction of the variance
of |lateral accel| it leaves unexplained. On a car with a clamp that score falls as the candidate rises
toward it and rises again above it, where the candidate keeps torque that moves while the car does not;
on a linear car it falls all the way to 1.0. The knee is the highest candidate within ``CEILING_TOL`` of
the best (the ones below tie with it whenever the wire rarely visits the band between), and ``ceiling``
moves toward the knee at ``CEILING_LEARN_RATE`` only while the candidate at its current value is worse
than the best by ``CEILING_EVIDENCE``: the ceiling is a property of the car, and a drive whose samples
do not disagree with where it is leaves it where it is. The bank sees the raw wire, the shape table
sees the wire clipped at the ceiling (what the EPS acted on), the speed table only the samples the clip
did not shape (``GAIN_MAX_CLIP``), and ``limit()`` clips the wire at ``ceiling + CEILING_PROBE``: the
probe band is what keeps the bank able to see that more torque *does* do something on a car whose
ceiling is higher than the persisted one (the candidates above the ceiling then beat the one at it, and
it walks back up), and on a car with a real clamp it costs nothing. A prior of 1.0 is no clamp; the car
controller seeds the MDX 3G at its measured value. On a car whose saturation really is a function of
lateral accel the knee sits where the wire had stopped buying much anyway, and the cost of taking it for
a clamp is a few percent of lateral accel at most.

The score is deliberately scale-free. The first version of the bank fitted one bounded multiplier on
``gain(v)`` per candidate by LMS and compared mean squared residuals, which made the pick depend on the
gain table's scale: in route 00000129 the table at 10-20 mph had grown to 2-3x the measured gain
(regressed on a wire clipped at a ceiling that was already too low), every candidate then fitted inside
the bound, the residuals were compared mid-convergence, and the "lowest candidate within tolerance"
rule walked the ceiling from 0.538 to 0.416 in one 7-minute drive, which put the wire bound (0.516)
below the knee the offline fits measure (0.50-0.58) and cost the car authority it had. The same drive
replayed through this bank holds 0.538. The samples the first version learned from were, for the most
part, not dwells either: the decay after a hard turn's exit also separates the candidates, but only
through the lag model's onset timing (the car's delay is longer than ``WIRE_DELAY``, which reads as a
lower ceiling), and a turn entry's rising edge makes every clipped candidate an affine copy of the same
exponential, which a free line fits equally well.

Known limits of the identification: it uses ``steeringPressed`` to exclude the driver, and on the MDX
that flag is set by the EPS's own torque-sensor oscillation in hard turns (route 00000127: 76% of the
ticks with the wire above 0.42), so a town drive yields no settled, unpressed dwell above the lowest
candidate at all (routes 00000127, 00000129: none) and the ceiling only moves on drives with long
sweepers; a car starting from the 1.0 prior needs several such drives to settle, and the MDX 3G relies
on its seed. Below ~20 mph the lateral accel a wire increment buys is too small against the noise for
the bank to decide anything (``CEILING_MIN_LAT_ACCEL_SPREAD``). The speed dependence of the ceiling
below 10 m/s is not established (the rate fit is flat there in one route and not in another); a single
number is learned.

Steer-rate evidence for the ceiling
-----------------------------------
The dwell bank above needs the wire to hold a level for a few seconds. The torque controller does not
drive that way: in a hard turn it runs the wire from 0.3 to 1.0 and back inside a second, and on the
Acura Integra (route 00000139, STEER_MAX 5120, no seed) the dwell bank counted zero informative
samples in a 14-minute drive (every tick above the lowest candidate failed the dwell gate, 95% the
trend gate too) while the wire sat pinned at 5120 for 1.7 s at a time and the car settled at 0.87
m/s^2 against a planner asking 2.0. The same drive, regressed in steer rate, shows the clamp plainly:
the steer rate a wire increment buys is linear up to ~2500-3000 counts and flat from there to 5120, in
every steering-angle band including 0-4 deg where the self-aligning torque is negligible, so it is a
function of the command level and not of the angle; a clamp model beats a linear wire with a free
centering curve (R^2 0.45 vs 0.40), and the knee is the same fraction of STEER_MAX the MDX 3G has
(0.50-0.55 of 433).

``_identify_ceiling_rate`` is a second bank on the same candidates that scores them on the steer rate
instead of the settled lateral accel. Per candidate it keeps exponentially weighted normal equations of
``steer_rate = K * clip(wire, +-c) + a * angle + b * lat_accel + d`` over the ticks with the delayed
wire above ``RATE_MIN_WIRE`` (``RATE_DELAY`` is the wire-to-rate delay, much shorter than the
lateral-accel lag the dwell bank works through), and the score is the fraction of the steer-rate
variance the line leaves unexplained. The angle and lateral-accel terms carry the centering, so a
plant that saturates with lateral accel rather than torque is explained by them and not by the clip;
the clipped wire is the only term that differs between candidates. Transients are what this bank
learns from: a turn entry that ramps the wire through the clamp is a rising rate that stops rising at
the clamp while the wire keeps going, which separates the candidates at once, where the dwell bank saw
only affine copies of one exponential through the 1 s plant lag. The knee rule and the evidence gate
are the dwell bank's, in relative terms (``RATE_TOL``, ``RATE_EVIDENCE``): the residual variances differ
by a few percent between neighboring candidates on a noisy per-tick regression, so the knee is the
highest candidate within ``RATE_TOL`` of the best residual, and the ceiling moves only while the
candidate at it is worse than the best by ``RATE_EVIDENCE``. Replays from the 1.0 prior: Integra route
00000139 -> best 0.55, knee 0.60, candidate 1.0 worse by 14%; MDX route 00000127 -> best 0.55, knee
0.60 (the offline rate fits give 0.50-0.58, the seed is 0.538), 1.0 worse by 30%; the Integra's earlier
angle-PID drive 00000135 (STEER_MAX 4096, R^2 0.12) -> no clamp, ceiling holds. Whichever bank has
evidence moves the ceiling; when both do, toward the higher knee (the cost of erring low is authority
the car had).

Persisting the tables needs the ``HondaLatGainNNParams``, ``HondaLatShapeNNParams`` and
``HondaLatCeilingParams`` keys registered in openpilot's ``common/params_keys.h`` (``param_keys()``
lists them); unregistered keys are silently dropped by the param writer and that table simply restarts
from the priors each drive.
"""
from collections import deque

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

# EPS torque ceiling (see the module docstring): normalized wire torque above which the EPS delivers nothing more
CEILING_CANDIDATES = tuple(round(0.40 + 0.05 * i, 2) for i in range(13))   # 0.40 .. 1.00
CEILING_MIN = CEILING_CANDIDATES[0]
CEILING_MAX = CEILING_CANDIDATES[-1]
CEILING_PRIOR = 1.0                               # no clamp until the car shows one; the car controller may seed a measured value
CEILING_KEY = "HondaLatCeilingParams"
CEILING_PROBE = 0.10                              # normalized torque the wire may run above the ceiling so the bank keeps seeing it
# Each candidate's score is the fraction of the variance of |lateral accel| its clipped wire leaves unexplained,
# 0..1, so the scores compare across candidates and across cars without reference to the gain table's scale.
# The knee is the highest candidate within CEILING_TOL of the best: candidates below the knee tie with it
# whenever the wire rarely visits the band between them (a shallow minimum), and the cost of erring low is
# authority the car had (route 00000129: 0.538 -> 0.416 in one drive, the wire bound below the measured knee)
CEILING_TOL = 0.02
# the ceiling starts moving once the candidate at its current value scores worse than the best by
# CEILING_EVIDENCE and stops once the margin is down to CEILING_EVIDENCE_SETTLED: the ceiling is a property
# of the car, and a drive whose few seconds of samples do not disagree with where it is leaves it where it is
CEILING_EVIDENCE = 0.04
CEILING_EVIDENCE_SETTLED = 0.01
# per informative tick, toward the knee: informative samples are settled dwells above the lowest candidate,
# a few seconds per drive at best on this car, so one drive moves the ceiling a fraction of the way and
# several settle it
CEILING_LEARN_RATE = 0.001
CEILING_RESID_TAU = 20.0                          # s of informative samples; EW moments per candidate
CEILING_RESID_ALPHA = DT_CTRL / (CEILING_RESID_TAU + DT_CTRL)
CEILING_MIN_SAMPLES = int(round(5.0 / DT_CTRL))   # informative ticks before the ceiling first moves
CEILING_MIN_LAT_ACCEL_SPREAD = 0.05               # m/s^2, standard deviation of |lat accel| over the samples before they are scored
CEILING_MIN_SPREAD = 0.02                         # normalized torque the top and bottom candidates must disagree by for a sample to count
# a sample counts only while the raw wire is holding a level: its EW standard deviation over CEILING_DWELL_TAU
# must be within CEILING_DWELL_TOL. On a rising or falling edge every clipped candidate is an affine copy of
# the same exponential and the fit cannot tell them apart; what is left to score there is the lag model's
# timing, which is not evidence about the ceiling (route 00000127's unpressed samples above 0.42 were five
# sub-second slices of turn entries, and read "linear to 1.0" against the dwells' knee)
CEILING_DWELL_TAU = 1.0                           # s
CEILING_DWELL_ALPHA = DT_CTRL / (CEILING_DWELL_TAU + DT_CTRL)
CEILING_DWELL_TOL = 0.03                          # normalized torque
# the speed table may not learn from a FILTER_TAU sample the ceiling clip shaped by more than this (normalized
# torque): a ceiling that is too low would otherwise teach it the car's response per unit of a wire it never
# got, and the table is the feedforward and the jerk bound (route 00000129: 0.77 m/s^2 per unit at 10 mph
# against 0.4 measured, half the torque the turns needed)
GAIN_MAX_CLIP = 0.02
# the bank's own trend gate, a settledness test on the lag-aligned pair rather than the shape table's
# transition filter: the candidates are static models, and a 0.1 step in the wire passes the dwell gate
# above within half a second while the lagged signals take ~3 s to arrive, so the lagged wire and the
# lateral accel must themselves have stopped moving before a sample is scored
CEILING_MAX_LEARN_JERK = 0.03                     # m/s^3
CEILING_MAX_LEARN_TORQUE_RATE = 0.03              # normalized torque per second

# EPS torque ceiling, steer-rate evidence (see the module docstring): the same candidates scored on how the
# steer rate follows the clipped wire, over transients rather than dwells
RATE_DELAY = 0.15                                 # s, wire to steer rate (route 00000139: the fit is best at 0.10-0.15 s)
RATE_DELAY_TICKS = max(int(round(RATE_DELAY / DT_CTRL)), 1)
RATE_TAU = 60.0                                   # s of samples; a per-tick regression at R^2 ~0.4 needs the memory
RATE_ALPHA = DT_CTRL / (RATE_TAU + DT_CTRL)
RATE_MIN_WIRE = 0.25                              # normalized, |delayed wire| for a tick to count: the line needs its linear part
# below ~10 m/s the fit degrades (large angles, the centering terms least exact) and in the Integra's angle-PID
# drive 00000135 the 5-10 m/s rows alone read "linear" against knees the 10-20 m/s rows show
RATE_MIN_SPEED = 10.0                             # m/s
RATE_MIN_SAMPLES = int(round(10.0 / DT_CTRL))     # counted ticks before the bank first scores (MDX town route 00000127: 14 s above 10 m/s)
RATE_SCORE_INTERVAL = 5                           # ticks between scorings (13 small solves)
RATE_MIN_RATE_STD = 2.0                           # deg/s, steer-rate spread over the samples before they are scored
# the line has to explain a fair share of the steering before where it bends is evidence: routes 00000139 and
# 00000127 fit at 0.46 / 0.76, the angle-PID drive 00000135 at 0.12-0.25 with a knee within 20% of its STEER_MAX
RATE_MIN_R2 = 0.30
RATE_TOL = 0.02                                   # relative: knee = highest candidate within 2% of the best residual variance
RATE_EVIDENCE = 0.05                              # relative: the candidate at the ceiling must be this much worse to move it
RATE_EVIDENCE_SETTLED = 0.01
# per counted tick, toward the knee: this bank counts ~100 s of ticks in a town drive (the dwell bank a few seconds),
# so a drive with a clear knee gets most of the way there and the next one settles it
RATE_LEARN_RATE = 0.0002
RATE_RIDGE = 1e-9


def _clip(value, lo, hi):
  return float(min(max(value, lo), hi))


def _shape_slot(lat_accel):
  # Params key slot for a shape bin: lateral accel in tenths of m/s^2 (1.5 -> 15)
  return int(round(lat_accel * 10))


def _load(param_get, key, default):
  # a missing or not-yet-registered key must never take the car controller down
  if param_get is None:
    return float(default)
  try:
    value = param_get(key)
  except Exception:
    return float(default)
  if value is None:
    return float(default)
  try:
    return float(value)
  except (TypeError, ValueError):
    return float(default)


class HondaLateralModel:
  def __init__(self, lat_accel_factor, param_get=None, ceiling_prior=CEILING_PRIOR):
    self.lat_accel_factor = (
      float(lat_accel_factor) if lat_accel_factor and lat_accel_factor > 0.1 else DEFAULT_LAT_ACCEL_FACTOR
    )
    self.gains = [_clip(_load(param_get, GAIN_KEY_FMT.format(slot=mph), prior), GAIN_MIN, GAIN_MAX)
                  for mph, prior in zip(GAIN_BINS_MPH, GAIN_PRIOR, strict=True)]
    self.shapes = [_clip(_load(param_get, SHAPE_KEY_FMT.format(slot=_shape_slot(la)), prior), SHAPE_MIN, SHAPE_MAX)
                   for la, prior in zip(SHAPE_BINS_LAT_ACCEL, SHAPE_PRIOR, strict=True)]
    self._project_shapes()
    self.ceiling = _clip(_load(param_get, CEILING_KEY, ceiling_prior), CEILING_MIN, CEILING_MAX)
    self.wire_hist = deque([0.0] * max(int(round(WIRE_DELAY / DT_CTRL)), 1), maxlen=max(int(round(WIRE_DELAY / DT_CTRL)), 1))
    self.wire_lag = 0.0             # delayed wire through the plant lag: the lateral accel the wire has "earned" so far
    self.clip_lag = 0.0             # what the ceiling clip took off the delayed wire, through the same lag and filter
    self.wire_filt = 0.0            # FILTER_TAU pair, speed table
    self.lat_accel_filt = 0.0
    self.clip_filt = 0.0
    self.wire_fast = 0.0            # SHAPE_FILTER_TAU pair, shape table
    self.lat_accel_fast = 0.0
    # ceiling bank: per candidate, the raw delayed wire clipped at the candidate through the plant lag and the
    # short filter, and EW moments of (gain(v) * |clipped wire|, |lat accel|) over the samples that tell the
    # candidates apart; each candidate is scored by the least-squares line through those
    n_cand = len(CEILING_CANDIDATES)
    self.ceiling_candidates = np.array(CEILING_CANDIDATES)
    # the bank's speed normalization, frozen at start-up: the live table moves while a drive's samples sit in
    # the bank's window, and a table correcting itself by 2x (route 00000129's) would rescale the same wire
    # level from one dwell to the next
    self.ceiling_gains = list(self.gains)
    self.ceiling_lag = np.zeros(n_cand)
    self.ceiling_fast = np.zeros(n_cand)
    self.wire_dwell_mean = 0.0      # EW mean and mean square of the raw delayed wire over CEILING_DWELL_TAU
    self.wire_dwell_sq = 0.0
    self.ceiling_mx = np.zeros(n_cand)
    self.ceiling_mxx = np.zeros(n_cand)
    self.ceiling_mxy = np.zeros(n_cand)
    self.ceiling_my = 0.0
    self.ceiling_myy = 0.0
    self.ceiling_slope = np.ones(n_cand)      # telemetry: fitted |lat accel| per unit of gain(v) * |clipped wire|
    self.ceiling_resid = np.ones(n_cand)      # telemetry: fraction of the |lat accel| variance each candidate leaves unexplained
    self.ceiling_weight = 0.0       # EW weight accumulated, normalizes the moments while the average is young
    self.ceiling_samples = 0
    self.ceiling_moving = False     # the bank has evidence against the current ceiling and is moving it
    self.ceiling_scored = False     # the bank scored a sample this tick
    self.ceiling_knee = self.ceiling  # the bank's current pick, telemetry
    # steer-rate bank: per candidate, EW normal equations of steer_rate on [clipped wire, angle, lat accel, 1]
    self.rate_hist = deque([0.0] * RATE_DELAY_TICKS, maxlen=RATE_DELAY_TICKS)
    self.rate_xtx = np.zeros((n_cand, 4, 4))
    self.rate_xty = np.zeros((n_cand, 4))
    self.rate_sy = 0.0
    self.rate_syy = 0.0
    self.rate_weight = 0.0
    self.rate_samples = 0
    self.rate_resid = np.ones(n_cand)         # telemetry: fraction of the steer-rate variance each candidate leaves unexplained
    self.rate_slope = np.zeros(n_cand)        # telemetry: fitted deg/s per unit of clipped wire
    self.rate_moving = False
    self.rate_counted = False       # the bank took a sample this tick
    self.rate_knee = self.ceiling
    self.press_holdoff = 0.0
    # telemetry for the last update() call
    self.gain_now = float(np.interp(0.0, GAIN_BINS_MS, self.gains))   # effective gain at (v, |desired lat accel|)
    self.shape_now = 1.0
    self.ff_correction = 0.0        # model ff minus controller ff, before the unit-torque clip
    self.applied_correction = 0.0   # output minus request, i.e. what was actually added this tick
    self.learning = False           # any table updated this tick
    self.learning_gain = False
    self.learning_shape = False
    self.learning_ceiling = False
    self.output = 0.0
    self.jerk_limited = False       # limit() clipped the wire on the lateral jerk bound this tick
    self.accel_limited = False      # limit() clipped the wire on the lateral accel bound this tick
    self.rate_limited = False       # limit() clipped the wire on the torque-rate backstop this tick
    self.ceiling_limited = False    # limit() clipped the wire at the EPS ceiling (plus probe) this tick

  def gain(self, v_ego):
    return float(np.interp(v_ego, GAIN_BINS_MS, self.gains))

  def shape(self, lat_accel):
    """Centering multiplier on gain(v) at this |lateral accel|: 1.0 through the anchor band, then the
    learned table, held at the last bin beyond it."""
    return float(np.interp(abs(lat_accel), SHAPE_BINS_INTERP, [1.0] + self.shapes))

  def effective_gain(self, v_ego, lat_accel):
    return self.gain(v_ego) * self.shape(lat_accel)

  @property
  def wire_limit(self):
    """Normalized torque bound limit() holds the wire to: the ceiling plus the probe band."""
    return _clip(self.ceiling + CEILING_PROBE, CEILING_MIN, CEILING_MAX)

  def effective_wire(self, torque):
    """The part of a wire torque the EPS acts on."""
    return _clip(torque, -self.ceiling, self.ceiling)

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
    request, to be rate limited by the caller."""
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
    """Quasi-static lateral accel the wire commands at this speed, speed table only (see module docstring)."""
    return self.gain(v_ego) * float(torque)

  def torque_from_lat_accel(self, lat_accel, v_ego):
    return float(lat_accel) / self.gain(v_ego)

  def limit(self, torque, last_torque, v_ego):
    """Bound the wire to ISO 11270 lateral jerk and lateral accel, computed in lateral-accel space with the
    identified speed gain, then to the torque-rate backstop and unit torque. torque: corrected request this
    tick; last_torque: what went to the EPS last tick. Returns the torque to send (normalized)."""
    la_last = self.lat_accel_from_torque(last_torque, v_ego)
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
    out = float(torque) if la_out == la_req else self.torque_from_lat_accel(la_out, v_ego)
    backstop = _clip(out, last_torque - WIRE_RATE_MAX * DT_CTRL, last_torque + WIRE_RATE_MAX * DT_CTRL)
    self.rate_limited = backstop != out
    # the EPS ceiling plus the probe band (see the module docstring): torque above it is never delivered
    bound = self.wire_limit
    self.ceiling_limited = bound < 1.0 and abs(backstop) > bound
    return _clip(backstop, -bound, bound)

  def _identify(self, wire_torque, current_curvature, v_ego, active, steering_pressed, steering_angle_deg=0.0,
                steering_rate_deg=0.0):
    raw_delayed_wire = self.wire_hist[0]
    self.wire_hist.append(float(wire_torque))
    rate_delayed_wire = self.rate_hist[0]
    self.rate_hist.append(float(wire_torque))
    # the tables regress on what the EPS acted on; the bank below sees the raw wire
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
    clipped = np.clip(raw_delayed_wire, -self.ceiling_candidates, self.ceiling_candidates)
    self.ceiling_lag += PLANT_ALPHA * (clipped - self.ceiling_lag)
    dxc = SHAPE_FILTER_ALPHA * (self.ceiling_lag - self.ceiling_fast)
    self.ceiling_fast += dxc
    self.wire_dwell_mean += CEILING_DWELL_ALPHA * (raw_delayed_wire - self.wire_dwell_mean)
    self.wire_dwell_sq += CEILING_DWELL_ALPHA * (raw_delayed_wire * raw_delayed_wire - self.wire_dwell_sq)

    self.press_holdoff = PRESS_HOLDOFF if steering_pressed else max(self.press_holdoff - DT_CTRL, 0.0)
    self.learning_gain = False
    self.learning_shape = False
    self.learning_ceiling = False
    self.ceiling_scored = False
    self.rate_counted = False
    self.learning = False
    if not (active and self.press_holdoff <= 0.0 and v_ego > MIN_LEARN_SPEED):
      return

    # plant: y = gain(v) * shape(|y|) * x, with shape == 1.0 through the anchor band. Gentle curves train
    # the speed table and only the speed table; harder turns train the shape, seeing the gain as known.
    # Both regress on what the EPS actually got, so a pinned wire is a valid sample for the shape; the speed
    # table skips the samples the ceiling clip shaped (see GAIN_MAX_CLIP)
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

    self._identify_ceiling(v_ego, self.lat_accel_fast, dyf, dxc)
    self._identify_ceiling_rate(rate_delayed_wire, measured, v_ego, steering_angle_deg, steering_rate_deg)
    self._move_ceiling()
    self.learning = self.learning_gain or self.learning_shape or self.learning_ceiling

  def _move_ceiling(self):
    # whichever bank has evidence against the current ceiling moves it toward its knee, by its own step on the
    # ticks it took a sample; when both do, toward the higher knee: the cost of erring low is authority the car had
    targets = []
    if self.ceiling_moving and self.ceiling_scored:
      targets.append((self.ceiling_knee, CEILING_LEARN_RATE))
    if self.rate_moving and self.rate_counted:
      targets.append((self.rate_knee, RATE_LEARN_RATE))
    if targets:
      knee, rate = max(targets)
      self.ceiling = _clip(self.ceiling + rate * (knee - self.ceiling), CEILING_MIN, CEILING_MAX)

  def _identify_ceiling(self, v_ego, yf, dyf, dxc):
    # the bank only learns from samples the candidates disagree on: the lagged raw wire (the top candidate,
    # 1.0, never clips) above the lowest candidate, in a real curve the same way, while the wire holds a
    # level (see CEILING_DWELL_TOL). The wire itself must be up there, not only the filters' memory of it:
    # the decay after a hard turn's exit separates the candidates too, but through the lag model where it is
    # least exact, and route 00000129 read 7 s of such samples against 2 s of real ones
    xc = self.ceiling_fast
    spread = abs(xc[-1]) - abs(xc[0])
    trending = (abs(dyf) / DT_CTRL > CEILING_MAX_LEARN_JERK or abs(dxc[-1]) / DT_CTRL > CEILING_MAX_LEARN_TORQUE_RATE)
    dwelling = self.wire_dwell_sq - self.wire_dwell_mean * self.wire_dwell_mean <= CEILING_DWELL_TOL * CEILING_DWELL_TOL
    if (trending or not dwelling or spread < CEILING_MIN_SPREAD or abs(xc[-1]) < CEILING_MIN + CEILING_MIN_SPREAD
        or abs(yf) < MIN_LEARN_LAT_ACCEL or np.sign(xc[-1]) != np.sign(yf)):
      return
    # plant under each candidate: |y| = slope_k * gain(v) * |x_k| + offset, both free. The score is the
    # fraction of the variance of |y| that line leaves unexplained, 0..1: the candidate at the car's ceiling
    # sees a wire that moves when the car does and stands still when it does not; one below it a wire that
    # stands still while the car still moves; one above it a wire that moves while the car does not. Only
    # co-variation counts (a candidate that merely knew left from right, or a constant, explains nothing), so
    # the score does not depend on the gain table's scale: the gain only puts samples from different speeds
    # on one line, and an inflated table cannot pull the knee down the way a bound on the slope would
    xg = float(np.interp(v_ego, GAIN_BINS_MS, self.ceiling_gains)) * np.abs(xc)
    y = abs(yf)
    alpha = CEILING_RESID_ALPHA
    self.ceiling_mx += alpha * (xg - self.ceiling_mx)
    self.ceiling_mxx += alpha * (xg * xg - self.ceiling_mxx)
    self.ceiling_mxy += alpha * (xg * y - self.ceiling_mxy)
    self.ceiling_my += alpha * (y - self.ceiling_my)
    self.ceiling_myy += alpha * (y * y - self.ceiling_myy)
    self.ceiling_weight += alpha * (1.0 - self.ceiling_weight)
    self.ceiling_samples += 1
    if self.ceiling_samples < CEILING_MIN_SAMPLES:
      return
    w = self.ceiling_weight
    mx = self.ceiling_mx / w
    my = self.ceiling_my / w
    var_x = np.maximum(self.ceiling_mxx / w - mx * mx, 1e-9)
    cov = self.ceiling_mxy / w - mx * my
    var_y = self.ceiling_myy / w - my * my
    # without a spread of lateral accel across the dwells there is nothing to explain and the scores are
    # noise (a clamp at 10 m/s leaves 0.04 m/s^2 between a wire of 0.45 and a pinned one)
    if var_y < CEILING_MIN_LAT_ACCEL_SPREAD * CEILING_MIN_LAT_ACCEL_SPREAD:
      return
    self.ceiling_slope = cov / var_x
    self.ceiling_resid = np.clip(1.0 - np.maximum(cov, 0.0) * self.ceiling_slope / var_y, 0.0, 1.0)
    self.learning_ceiling = True
    self.ceiling_scored = True
    best = float(np.min(self.ceiling_resid))
    # the score falls up to the car's ceiling and rises again above it (on a linear plant it falls all the
    # way to 1.0). The knee is the highest candidate within tolerance of the best: the ones below tie with
    # it whenever the wire rarely visits the band between
    within = np.flatnonzero(self.ceiling_resid <= best + CEILING_TOL)
    self.ceiling_knee = float(self.ceiling_candidates[within[-1]])
    # the ceiling moves only on evidence against where it is: the candidate at the current ceiling must
    # explain the car worse than the best by CEILING_EVIDENCE, and then keeps moving toward the knee until
    # that margin is down to CEILING_EVIDENCE_SETTLED. Below the car's ceiling the evidence is torque in the
    # probe band the car answered to (the way back up from a persisted value that is too low); above it,
    # torque that moved while the car did not. A wire that only ever sits at one level, or a drive with a
    # few seconds of samples that disagree with each other, is not evidence, and the ceiling stays put
    at_ceiling = float(np.interp(self.ceiling, self.ceiling_candidates, self.ceiling_resid))
    self.ceiling_moving = at_ceiling - best > (CEILING_EVIDENCE_SETTLED if self.ceiling_moving else CEILING_EVIDENCE)

  def _identify_ceiling_rate(self, delayed_wire, lat_accel, v_ego, angle_deg, rate_deg):
    # a tick counts while the delayed wire is up where the line has something to fit: the candidates agree below
    # the lowest one, and the band between RATE_MIN_WIRE and it anchors the slope the clipped band is compared to
    self.rate_counted = abs(delayed_wire) > RATE_MIN_WIRE and v_ego > RATE_MIN_SPEED
    if not self.rate_counted:
      return
    xc = np.clip(delayed_wire, -self.ceiling_candidates, self.ceiling_candidates)
    n = len(xc)
    X = np.column_stack([xc, np.full(n, float(angle_deg)), np.full(n, float(lat_accel)), np.ones(n)])
    y = float(rate_deg)
    alpha = RATE_ALPHA
    self.rate_xtx += alpha * (X[:, :, None] * X[:, None, :] - self.rate_xtx)
    self.rate_xty += alpha * (X * y - self.rate_xty)
    self.rate_sy += alpha * (y - self.rate_sy)
    self.rate_syy += alpha * (y * y - self.rate_syy)
    self.rate_weight += alpha * (1.0 - self.rate_weight)
    self.rate_samples += 1
    if self.rate_samples < RATE_MIN_SAMPLES or self.rate_samples % RATE_SCORE_INTERVAL:
      return
    # a scoring tick: the verdict below replaces the last one, so a bank that stops qualifying stops moving
    self.rate_moving = self._score_rate()

  def _score_rate(self):
    w = self.rate_weight
    var_y = self.rate_syy / w - (self.rate_sy / w) ** 2
    # no steering going on over the samples: nothing to explain
    if var_y < RATE_MIN_RATE_STD * RATE_MIN_RATE_STD:
      return False
    try:
      beta = np.linalg.solve(self.rate_xtx / w + RATE_RIDGE * np.eye(4), (self.rate_xty / w)[:, :, None])[:, :, 0]
    except np.linalg.LinAlgError:
      return False
    sse = self.rate_syy / w - np.einsum('ij,ij->i', beta, self.rate_xty / w)
    self.rate_slope = beta[:, 0]
    self.rate_resid = np.clip(sse / var_y, 0.0, 1.0)
    best_idx = int(np.argmin(self.rate_resid))
    best = float(self.rate_resid[best_idx])
    # the wire must move the wheel its own way at the best candidate, and the line must explain a fair share of
    # the steering, or the samples are something else (the driver below the press threshold, a sign convention
    # error, a drive the delay model does not fit)
    if self.rate_slope[best_idx] <= 0.0 or best <= 0.0 or best > 1.0 - RATE_MIN_R2:
      return False
    self.learning_ceiling = True
    within = np.flatnonzero(self.rate_resid <= best * (1.0 + RATE_TOL))
    self.rate_knee = float(self.ceiling_candidates[within[-1]])
    at_ceiling = float(np.interp(self.ceiling, self.ceiling_candidates, self.rate_resid))
    margin = at_ceiling / best - 1.0
    return margin > (RATE_EVIDENCE_SETTLED if self.rate_moving else RATE_EVIDENCE)

  def _project_shapes(self):
    # non-increasing in lateral accel, from the anchor's 1.0 down: a bin with no data of its own inherits
    # the saturation the last measured one showed rather than the neutral prior
    ceiling = SHAPE_MAX
    for i, s in enumerate(self.shapes):
      ceiling = min(ceiling, s)
      self.shapes[i] = ceiling

  def learned_values(self):
    values = {GAIN_KEY_FMT.format(slot=mph): float(g) for mph, g in zip(GAIN_BINS_MPH, self.gains, strict=True)}
    values.update({SHAPE_KEY_FMT.format(slot=_shape_slot(la)): float(s)
                   for la, s in zip(SHAPE_BINS_LAT_ACCEL, self.shapes, strict=True)})
    values[CEILING_KEY] = float(self.ceiling)
    return values

  @staticmethod
  def param_keys():
    return ([GAIN_KEY_FMT.format(slot=mph) for mph in GAIN_BINS_MPH] +
            [SHAPE_KEY_FMT.format(slot=_shape_slot(la)) for la in SHAPE_BINS_LAT_ACCEL] +
            [CEILING_KEY])
