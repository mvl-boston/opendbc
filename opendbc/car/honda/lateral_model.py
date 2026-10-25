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
``gain(v)`` is a per-speed table of the car's real m/s^2-per-unit-torque, the car's ``latAccelFactor``
at every speed until it has learned otherwise (no per-car table is seeded: a seed that is right for one
car is a poisoned table for another, and the learners below reach the MDX 3G's 0.1 .. 2.6 and the Civic
2022's 0.3 .. 6 from the flat prior in a drive or two). ``shape(|lat_accel|)`` multiplies it: the EPS's self-aligning / return torque grows
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
on the wire torque that actually went to the EPS (delayed by the actuator delay and passed through the
plant lag), per speed bin, the sample spread over the two neighboring bins. It identifies a bounded
physical quantity, so it cannot drift the way an integrator of delay-lag error does, and it is
indifferent to rate limiting, clipping and saturation because those all act on the wire torque it
regresses against. The only unmodelled input is the driver, so learning pauses while
``steeringPressed`` and for ``PRESS_HOLDOFF`` afterwards.

The speed table regresses *changes* over a ``GAIN_DIFF_WINDOW`` window (d lat_accel on d wire), not
levels through the origin. The levels are not proportional on this steering: the wheel holds an angle
with far less torque than it took to get there (route 00000177, 17 m/s: 9 deg and 0.63 m/s^2 held while
the wire relaxed from 0.29 to 0.11), the road's crown and bank add lateral accel no wire paid for, and
sub-threshold hand torque does the same, so a sample's lat_accel / wire ratio depends on where in a
dwell it was taken and a through-origin fit of the levels reads a slope that is mostly offset: 1.3-1.5x
the plant on every speed bin in that route offline, 2-3x on the device after a week of drives (2.2 at
20 mph against a measured 0.87, pinned at ``GAIN_MAX`` from 40 mph), with the feedforward then
delivering 40-75% of the torque a turn needed and the jerk bound, which is ``MAX_LAT_JERK / gain(v)``
in torque, holding the wire to 2-4 m/s^3 at the car instead of 5. A difference cancels whatever is held
over the window; what is left is the marginal response, which is what both the feedforward (torque for
a change in desired lateral accel) and the jerk bound (lateral accel per unit of torque rate) are
about. The same route reads 0.24 / 0.44 / 0.87 / 1.51 / 1.85 / 2.40 m/s^2 per unit from 5 to 18 m/s
that way, stable from 0.5 to 2 s windows and within 10% between wind and unwind, against priors of
0.1-0.25 / 0.45 / 0.8 / 1.35-1.9 / 1.9-2.4 / 2.1-2.4. There is no envelope around the prior: a table
far from the car's tune is a measurement on one car (the Civic 2022 reads 5.5-6 m/s^2 per unit at 60-70
mph against its 3.1) and a poisoned table on another, and only the data can tell the two apart, so what
protects against poison is that every bin is reachable from any value by the learners below, in both
directions ("Two-way reachability").

A window counts only if the wire moved by ``GAIN_MIN_DELTA_TORQUE`` and every tick in it, and for
``GAIN_SETTLE`` on either side of it, was active, unpressed, in the linear part of the wire
(``GAIN_MAX_CLIP``), inside the first response band (``GAIN_MAX_WIRE``) and under ``GAIN_MAX_LAT_ACCEL``
of lateral accel. The settle is about the lag model: it is a first-order lag at ``PLANT_TAU``, the
car's is not exactly that (the routes fit 0.7-1.3 s), so a window at the start of a change reads the
ratio high and one in its tail reads it low, by 2x either way on a plant 20% faster than the model;
over a whole change the two cancel, over the part of one the band admits they do not. Windows leading
into a hard turn are its head and the ones coming out of it its tail (where the lagged wire also still
carries torque the centering shape bent), so neither side may be within ``GAIN_SETTLE`` of a tick
outside the band. The windows are fitted by least squares over exponentially weighted sums per bin
(``GAIN_FIT_TAU`` of learning ticks), and the table tracks the fit at ``GAIN_TRACK_RATE``: the fit
weights a window by the size of its change, so the whole changes carry it and the short windows at the
edges of the band do not, and it does not depend on the order the windows came in, which an LMS step
per window does (the test plant, 0.8 s against the model's 1.0 s, reads 2.20 of 2.2 by least squares
and 1.7-2.07 by LMS depending on the step size).

Where the speed table may learn
-------------------------------
The gate is on the wire, not on the lateral accel the table predicts for it. A table that is already
low would otherwise admit the hard turns and one that is already high only the gentlest, each reading
its own error back and never leaving: that is how a poisoned table stays poisoned. The wire's first
response band (0.4 of STEER_MAX, where every Honda measured so far is linear) is where the table is
defined, and the EPS response shape above it is the wire learner's business; a window with its ends on
opposite sides of zero is skipped (the EPS has its own nonlinearity about center: with those windows
the Civic's highway class read 7.7-7.9 for a steady secant of 6.5-7.3).

Three things in the windows are not the plant and are taken out before the fit. Friction: a change in
the wire smaller than the steering's friction band moves the wheel against the friction and not the
road, and most 1 s windows are small (mean |dx| 0.07), so a fit of dy on dx reads the small changes low
(the Civic at 13-18 m/s: 0.7 m/s^2 per unit of change at |dx| 0.06-0.09, 1.5-1.9 at 0.15-0.2; its table
at 30-40 mph came out at 1.0 that way against 2 for the changes its curves are made of). The band is a
torque, so its width in STEER_MAX differs by car; it is learned per speed class from ``GAIN_FRICTIONS``
as the candidate under which the small and the large windows, and the head and the tail of a change,
read the same gain (see the constants). Closed-loop bias: the controller's P term answers the lateral
accel's noise and the road's disturbances with wire of the opposite sign, so a regression of the
lateral-accel change on the wire change reads the plant low wherever the reference is not what moved
the wire (0.5-0.6 of the others' in the lowest third of reference movement on all three cars); the
windows are fitted by instrumental variables with the planner's desired lateral accel, through the same
delay, lag and filter, as the instrument (``GAIN_IV_MIN_CORR``). And the lag-friction valley: in a 1 s
window a plant faster than ``PLANT_TAU`` makes the head of a change read more than its tail exactly as a
friction band does, so the friction candidate is taken only where it makes both splits agree, and a
plant 20-40% faster than the model still reads 1.2-1.6x (the known cost of the window length, bounded
in the tests).

Two-way reachability
--------------------
A table has to be able to come back from a poisoned value in either direction, from the evidence of an
ordinary drive. Down is the windows above. Up is harder: a table that is low starves the feedforward,
the controller's integrator makes up the torque, and the drive is mostly pinned wire and driver help,
which the windows skip. Two things read it anyway. The over-delivery floor: a steady sample with the
effective wire at or above ``GAIN_OVER_MIN_WIRE`` whose lateral accel is more than ``GAIN_OVER_RATIO``
times what the table predicts says the table is low here, whatever the windows have been reading; the
bins are raised toward secant / ``GAIN_OVER_RATIO``. The margin keeps the holding branch (the steady
secant runs 1.1-1.3x the marginal response on every car measured) out of it, so the floor's equilibrium
sits below the marginal on a healthy table and 1.5-3x above a poisoned one (route 0000005a, the
driver's "lag getting worse on curves": the Civic's table at 0.9 from 30 mph up on a car delivering
2.2-2.6 there and 5.5-6 at 60-70 mph; replayed from that table the 60-70 mph bins read 5.5-5.8 again
within the drive).
And the delivery fit (below, "Where the wire is bounded"): the lateral accel's own reading of the wire
response shape, which keeps a persisted shape from holding the wire below a level the car answers to,
the other way a drive can be starved of torque.

The two tables are trained on disjoint data so they cannot trade scale: the speed table learns only
inside the anchor band (gentle curves, where shape == 1 by definition) and the shape table only above
it, treating the gain as known. The shape is bounded at 1.0 and non-increasing in lateral accel: a
saturating EPS never delivers more per unit torque in a hard turn than in a gentle one. Route 0000011f
measured it at 29-36 mph: 2.7 m/s^2 per unit of lagged wire torque below 0.3, 1.9 at 0.4-0.5, 1.4 at
0.7-0.8, 1.27 at 0.8-0.9 (shape 1.0 -> 0.5 between 0.85 and 1.05 m/s^2), and 1.1-1.3 m/s^2 with the
wire pinned at 433 counts. The tables persisted before this bound held 1.5-1.6 there, which put the
feedforward *below* the linear one exactly where the car was undershooting with the wire pinned.

The shape path uses a short filter (``SHAPE_FILTER_TAU``) on the lag-aligned pair rather than the 2 s one
the speed table once used: hard-turn dwells between the driver's corrections last 0.5-1.5 s (route
0000011f: 136 pinned, unpressed dwells, median 0.06 s, none over 1.5 s), and a 2 s filter with a
quasi-static gate never admitted one (0 pinned samples in that route). The lag model does the work
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
the wire band the EPS no longer answers to at the current speed, as this drive has judged it, 1.0 if
there is none) and ``learning`` (identification ran this tick) are exposed for the actuatorsOutput
telemetry slots: gas is the gain, brake the correction, speed the ceiling (0.4-1.0) plus 2.0 while
learning.

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
the wire pinned at 433 as with the wire at 300-400. Route 00000177 (657 s, the wire at 433 on 10% of the
engaged ticks and in 233-433 on another 4%) settles the form of it: with the steer-rate plant fitted on
the rows below a clamp and the residual rate looked at above it, the response is linear to ~195-216
counts, a soft knee carries 10-30% of the linear slope from there to ~260-300, and from ~0.7 of STEER_MAX
(303 counts) to 433 the marginal response is 0.00 +- 0.1 of the linear band with the hands off (the
constrained per-band fit reads 1.0 / 0.3 / 0.24 / 0.24 / 0.02 / 0.02 / 0.02; at |angle| < 6 deg, where
the self-aligning torque is small, 1.0 / 0.4 / 0.4 / 0.4 / 0 / 0 / 0, so it is the command level and
not the angle). Everything above the knee together is worth ~0.02-0.05 of effective torque, 10-20
counts of the linear band: the EPS acts on the equivalent of ~215-230 counts no matter what is
commanded above that. It is a cutoff with a soft knee, not the Integra's long reduced slope. On the
Acura Integra (route 00000139, STEER_MAX
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
is (1, 1, 0.5, 0, 0, 0, 0); a linear car is all ones, which is the prior for every car (no seeds: the
Civic 2022's shape is nothing like the MDX's, and a seed that is right for one is a bound on the
other). Everything downstream is in effective torque: the gain and centering
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
``WIRE_DEAD_GAIN`` is dead in the persisted shape; the wire is bounded at a dead band's start plus
``WIRE_PROBE`` only on this drive's verdict for the speed ("Where the wire is bounded"), the probe band
keeping the learner able to see whether torque there does something after all. With no bound the wire
runs to 1.0 through the inverse at up to twice the wire per unit the linear part needs, the shortfall
reported as limiting.

``_identify_wire`` learns the shape from the steer rate, which answers the wire within ``RATE_DELAY``
(far shorter than the lateral-accel lag, so transients are usable and a wire that never holds a level,
which is how the torque controller drives, is enough). It keeps exponentially weighted normal equations
(``RATE_TAU``) of ``steer_rate = sum_k beta_k * band_k(wire) + a * angle + b * lat_accel + d`` over the
ticks with the delayed wire above ``RATE_MIN_WIRE`` at road speed (``RATE_MIN_SPEED``), unpressed, per
speed class (``SPEED_CLASSES``: the EPS's authority is not the same at every speed, see below). The
angle and lateral-accel terms carry the centering, so a plant that saturates with lateral accel rather
than torque is explained by them and not by the bands. Every ``FIT_INTERVAL`` counted ticks the class's
system is solved under ``beta_0 >= beta_1 >= ... >= 0`` (projected gradient with a pool-adjacent-violators
projection, warm-started from the last solution). Unconstrained, the band coefficients are
ill-conditioned: a wire that is pinned at 1.0 for part of a turn makes the top band an indicator of
"pinned", which a free fit uses to absorb whatever the centering terms left over; the constraint is what
makes them an estimate of the response. The fit has to explain a fair share of the steering
(``RATE_MIN_R2``) and the first band must turn the wheel its own way; a band moves only while the window
holds samples with the wire in or above it (``WIRE_MIN_BAND_ACTIVITY``) and the fit disagrees with it
by more than ``WIRE_TOL``, toward the fit at ``WIRE_LEARN_RATE`` per counted tick, and the bands above
an informed one inherit its saturation (non-increasing). The persisted shape follows the classes
jointly: it is the response where the car answers best over the speeds it has been driven at, so a
band's target is the highest any class with evidence reads for it, and it is demoted only when every
class with evidence has it dead. Replays from the linear prior (see the tests and the PR notes): the
Integra's route 00000139 reads 1.0 to 0.6 of STEER_MAX and ~0.05-0.2 above, the MDX's 00000127 1.0 to
0.5 and 0.3-0.4 to 0.7, the Integra's angle-PID drive 00000135 (R^2 0.12-0.25) is rejected by the fit
gate and holds.

A dead band comes alive only on evidence. A pinned wire fills every band at once, so the rows that make
up most of the data above the knee say nothing about which band the response belongs to, and the
non-increasing projection pools their noise upward: in route 00000177 the online fits (R^2 0.74-0.84)
read 0.08-0.13 for the bands above 0.7 of STEER_MAX in a third to a half of the windows while forcing
those bands dead cost the fit under 0.0007 of the steer-rate variance, i.e. nothing. That is how the
MDX's table had reached "alive everywhere" by that drive: every band above the seed's dead start had
been raised past ``WIRE_DEAD_GAIN`` one noisy window at a time, the bound was 1.0 and the wire ran to
433 for 10% of the engaged ticks, through torque the EPS ignores.

Learning down. The same pooling keeps a table like that from coming back: with every band persisted at
0.11-0.15 the pooled fit reads 0.10-0.27 for every band from 0.4 to 0.8 (routes 00000177 / 00000181 /
00000182), the noise floor of a non-negative estimate on collinear rows, so tracking the fit never takes
a band below ``WIRE_DEAD_GAIN`` and the bound stays at 1.0 drive after drive. Where the response stops is
decided instead by the response above each cut: the constrained fit is re-solved with the bands from
the cut up forced to zero (``_solve_bands`` with ``dead_from``), and the steer-rate variance that costs,
over what those bands' joint regressor would explain at the first band's slope, square-rooted, is the
mean response per unit of wire above the cut relative to the linear band -- as the data see it jointly,
free of the pooling. The cuts are scanned from the lowest active band up; a dead band comes alive only
above ``WIRE_ALIVE_SLOPE``, an alive band goes dead only below ``WIRE_DEAD_SLOPE`` (and only once the
window has filled well past the first fit, ``WIRE_DEMOTE_MIN_SAMPLES``: the fits in the 10 s after
``RATE_MIN_SAMPLES`` read the response above the knee at a third of its settled value), and between the
two it holds. Forcing from a higher cut is a weaker restriction, so the first cut that settles settles
every band above it: those are demoted toward zero at ``WIRE_DEMOTE_RATE`` (10x the learning rate, since
a town drive counts only 25-40 s of ticks in all) where the lateral accel agrees the band delivers
nothing, at the learning rate where it has nothing to say yet, and not at all where it shows the band
alive ("Where the wire is bounded"), and the fit with them forced to zero, no longer pooled with them,
is what the bands below track. The statistic is a mean over the range above the cut
weighted by where the wire was, so when the wire runs far above a knee it is diluted by the dead range
(a simulated hard clamp at 0.55 reads 0.26-0.33 above 0.4, where the band is fully alive, and 0.13-0.20
above 0.5, where half of it is; a linear EPS reads 0.60-0.87 above every cut, a 0.3 slope above its knee
0.14-0.18 and a clamp 0.00 above it). The probe bound is what takes the dilution out: once the bound is
down, the band above the ceiling is the top one the wire reaches and the test for it is its own
coefficient. The MDX 3G's routes read, above 0.4 / 0.5 / 0.6 / 0.7 of STEER_MAX: 00000177 0.06-0.13 /
0.04-0.07 / 0.03-0.07 / 0.00-0.06, 00000181 (the driver's hand below the press threshold on the
windups) 0.09-0.19 / 0.11-0.18 / 0.10-0.16 / 0.07-0.14, 00000182 0.02-0.10 / 0.03-0.12 / 0.00-0.14 /
0.00-0.14; the pinned residuals put the knee at 0.51 of STEER_MAX in 177 and 181 and 0.44 in 182, with
the bands above delivering 0.06-0.11 per unit, i.e. a soft knee inside the 0.4-0.5 band. Replayed from
the linear prior with the learners below, 177 reads (1, 0.83, 0.26, 0.26, ...) and 182 after it (1,
0.52, 0.02, 0.02, ...), the 30-40 mph class bounding the wire at 0.6 from the third drive on (0.5 dead,
the 0.5-0.6 band the probe).

Where the wire is bounded
-------------------------
The steer rate reads where the response stops; by itself it is one sensor's word, and a bound on its
word alone costs authority the car may have (an EPS whose rate saturates reads dead where its lateral
accel does not). The lateral accel is the second witness: the delivery fit (``_identify_delivery``)
regresses the change of the lateral accel over a ``GAIN_DIFF_WINDOW`` window (the centering shape taken
out, in the table's unit, and only against speed bins whose gain is settled, ``gain_settled``) on the
change of each band's lag-modelled share of the raw wire, under 1 >= g_1 >= ... >= 0. Changes, as for
the speed table: a pinned wire fills its bands by constant amounts, which a regression of levels cannot
tell from its intercept (the per-cut floors it read on the MDX's pinned routes that way were noise,
0.1-0.6 rising with the cut). The regressor is the lag of each band's share and not the share of the
lagged wire: the response shape acts on the wire before the plant's lag (a Hammerstein plant), and the
other way a clamp at 0.5 read the band above it delivering 0.4 while the wire was still climbing
through it. The window has no settle and is cleared only on a press or a gap in control: the MDX's EPS
sets ``steeringPressed`` through most of a hard turn, and with the speed table's 5 s of clean ticks
around a window the fit saw 300 windows in a 12-minute drive, 700-850 without them. The part of a
window's change inside the steering's friction band moved nothing, so the bands' shares are scaled to
the part beyond it (the Civic's 0.04 is most of a 1 s window's change; unscaled, its 13-18 m/s bands
read 0.1 for a measured 0.2-0.25). What the data force each band to deliver is the mean delivery above
its cut (the fit with the bands from there up held at zero against the full one, as in the steer-rate
scan), so it is a floor: biased low on a band the knee falls inside and on bands the wire rarely
reaches, never high. The floor is solved per speed class and the persisted shape is held at or above
the best class's, band by band; a band the floor shows alive (``WIRE_FLOOR_ALIVE``) is never called
dead by the steer rate, and a band it shows dead is demoted at the full rate.

The bound itself is per speed class and for this drive only. A class's verdict is its steer-rate dead
start with a witness on the band: the class's own delivery floor known and dead there, or the persisted
shape already dead there (which took the lateral accel's consent on an earlier drive). The verdict has
to hold over ``WIRE_BOUND_HOLD_FITS`` fits before the bound moves down (the fits in the first 10 s read
the response above the knee at a third of its settled value; a knee inside a band has them alternate
between it and the band above, and the pending bound is the least restrictive verdict since the last
lift, so both support it), and the bound lifts when the steer rate stops saying so or the floor shows
the band alive, as far as the persisted shape has the bands alive: the alive reading raises the band
first (at ``WIRE_LEARN_RATE``, ~5 s of counted ticks, or at once toward a floor that shows it) and the
bound follows it, since a lift on the steer rate's word alone runs the wire into the band above, where
the band it was read alive on as the top one reached is read diluted and dead again. A class without a
verdict borrows the nearest higher class's: a response that
stops at some command level on the highway stops there in town too, the assist being no larger; the
reverse does not hold, so a town verdict never bounds the highway. The persisted shape never bounds by
itself: it is the response where the car answers best over the speeds it has been driven at, and a
town-only history says nothing about the highway, where the complaint in route 0000005a came from (a
bound at 0.8 learned in town, carried to 24-32 m/s, with a speed table poisoned to 0.9 under it). The
Civic 2022 measures 0.2-0.25 per unit above 0.4 of STEER_MAX at 13-18 m/s (route 00000052: |lat accel|
0.74 at 0.4-0.5 of STEER_MAX to 1.03 at 0.9-1.0) and about the same relative to its first band at
24-32 m/s (route 0000005a segment 17: 2.5 m/s^2 delivered at 0.8 of wire, what the first band alone
buys at its 6.5-7.3 secant), so its verdicts are "dead above 0.4-0.5" at every speed and the bound
there costs it nothing; the architecture is for the car whose authority does differ by speed, and for
the one that has not yet been driven at the speed it is at.

The first version of this learned one clamp level, from settled dwells of lateral accel and from a bank
of clip candidates on the steer rate. The dwell bank counted zero samples on the Integra's drive (the
torque controller never holds the wire), and a clamp cannot say what a reduced slope does.
``HondaLatCeilingParams`` is still written (the persisted shape's dead-band start, ``shape_dead_start``)
and is read as a clamp when no shape has been persisted yet, so the MDX's 0.538 carries over.

Known limits: ``steeringPressed`` excludes the driver, and on the MDX that flag is set by the EPS's own
torque-sensor oscillation in hard turns (route 00000127: 76% of the ticks with the wire above 0.42), so
the shape there learns from the gaps between (and the delivery fit from its own, unsettled window);
below ~10 m/s the fit degrades (large angles, the centering terms least exact) and nothing is learned
there, the town class taking its bound from the class above; one shape is persisted for all speeds, the
best over them, and the bound is what carries the per-speed knowledge within a drive.

Compute budget
--------------
This runs inside the car controller's 100 Hz loop on the device, and ``selfdrived`` paces itself on the
``carState`` that loop publishes: when the loop averages more than ~11 ms over a second, ``selfdrivedLagging``
soft-disables with "TAKE CONTROL IMMEDIATELY / System Lagging". The constrained fits are what costs here:
each is a handful of projected-gradient solves (``WIRE_FIT_ITERS`` steps of a 10x10 normal system plus the
pool-adjacent-violators projection in Python), one for the full system and one per band forced dead, and
the delivery fit does that per speed class. Solved whole, every 5 counted ticks, a pinned wire ran the
model at ~30x its idle cost for as long as the turn lasted, and the loop dropped to 88-93 Hz (route
000001cc, 2:23:59 PM ET, wire at the 0.6 bound at 18.6 m/s for 1.5 s). The fits are therefore scheduled:
a fit is due every ``FIT_INTERVAL`` counted ticks (the normal equations have ``RATE_TAU`` of memory, so
anything faster re-solves the same system), and a due fit is a generator that does exactly one solve per
tick (``_step_fits``), the queue serving one solve per tick in all. Peak and mean cost are both bounded
that way; the verdict arrives a few ticks after it is due, from a snapshot of the system taken when the
fit started, and the learners between fits track the last complete one as before. The bound's hold is
in fits (``WIRE_BOUND_HOLD_FITS``), derived from the interval so it stays ~10 s of counted ticks.

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

# speed bins (mph, for readable Params keys). The prior is the car's latAccelFactor in every bin, i.e. the torque
# controller's own linear plant: with it the correction is zero and the car drives exactly as an unmodified openpilot
# would until the table has learned otherwise. No per-car table is seeded (see the module docstring, "How it learns"):
# the MDX 3G's measured 0.10 .. 2.60 from 5 to 70 mph is what its learner reaches from this prior in a drive or two,
# and a seed that is right for one car is a poisoned table for another.
GAIN_BINS_MPH = (5, 10, 15, 20, 30, 40, 50, 60, 70)
GAIN_BINS_MS = tuple(mph * CV.MPH_TO_MS for mph in GAIN_BINS_MPH)
GAIN_MIN = 0.10
GAIN_MAX = 8.00                                   # above every Honda torque tune (ACURA_MDX_4G_MMR: 6.0)
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
SHAPE_FILTER_ALPHA = DT_CTRL / (SHAPE_FILTER_TAU + DT_CTRL)
WIRE_DELAY = 0.30                                 # s, actuator delay applied to the wire before the plant lag
# The car's lateral accel follows the wire as a first-order lag, not instantly: a lag of 1.0-1.3 s (after the
# 0.3 s delay) maximizes the wire / lat-accel correlation at 10-16 m/s in routes 0000010e, 0000010f and
# 00000114, and 1.0 s does so above 16 m/s. Regressing on the lagged wire keeps a sample taken mid-turn
# honest; without it every turn entry read the gain low and every exit read it high.
PLANT_TAU = 1.0
PLANT_ALPHA = DT_CTRL / (PLANT_TAU + DT_CTRL)
LEARN_NORM_EPS = 0.01                             # torque^2, keeps the shape table's normalized step finite near zero
MIN_LEARN_TORQUE = 0.15                           # |filtered wire| needed for a shape sample
# speed table: windowed differences of the SHAPE_FILTER_TAU pair (see the module docstring). The window is on the
# lag-modelled wire, so a 1 s window straddles the plant's response to a change; 0.5-2 s read the same gain in route
# 00000177 (a 1 s window's dx there is mostly 0.05-0.1, so the threshold below is as high as it can go: 0.1 leaves a
# quarter of the windows, 0.15 a tenth)
GAIN_DIFF_WINDOW = 1.0                            # s
GAIN_DIFF_TICKS = max(int(round(GAIN_DIFF_WINDOW / DT_CTRL)), 1)
GAIN_MIN_DELTA_TORQUE = 0.05                      # normalized effective torque
# A window counts with both ends on the same side of zero and inside the first response band (GAIN_MAX_WIRE, linear
# in the wire by definition), with the lateral accel within GAIN_MAX_LAT_ACCEL over the window and its settle: a
# loose cap that keeps hard turns out without selecting on the output (see the module docstring, "Where the speed
# table may learn")
GAIN_MAX_WIRE = 0.40                              # normalized wire, == WIRE_CUTS[0]
GAIN_MAX_LAT_ACCEL = 1.50                         # m/s^2, == 2 * SHAPE_ANCHOR_LAT_ACCEL
# Friction (see the module docstring, "Where the speed table may learn"): a change in the wire smaller than the
# steering's friction band moves the wheel against the friction and not the road, a larger one moves the car by the
# plant's response to the part beyond it. It is a torque, so its width in STEER_MAX differs by car, and a through-origin
# fit of the windows reads it as a low gain: the Civic 2022's windows at 13-18 m/s read 0.7 m/s^2 per unit of change at
# |dx| 0.06-0.09 and 1.5-1.9 at 0.15-0.2 (routes 00000052 and 00000042; a line through the two is 1.5-2.6 per unit
# beyond a band of 0.035-0.053), 2.4 against 3.8 at 20-30 m/s (band 0.013-0.021); the Integra 1.2 against 1.4 (0.012);
# the MDX 3G 1.8-1.9 at every size (none). Most windows are small (mean |dx| 0.07), so the Civic's table at 30-40 mph
# came out at 1.0 that way against 2 for the changes its curves are made of. The band is learned from the candidates
# below, per speed class (SPEED_CLASSES; the Civic's is 0.04 in town and 0.02 at 60-70 mph): the window sums are kept
# for each candidate with the change mapped through that band (sign(dx) * max(|dx| - f, 0)), split two ways -- the
# small windows (|dx| below GAIN_FRICTION_SPLIT) against the large ones, and the head of a change (a window larger
# than the one before it) against its tail -- and the candidate under which both splits read the same gain is the
# band. Friction is what makes a small change read low and a large one not, and a candidate that is too wide makes
# the small ones read high instead; the head and the tail disagree under a lag model that is not the car's (the
# routes fit 0.7-1.1 s against PLANT_TAU's 1.0), and in a 1 s window a lag error can pass for friction (the Civic's
# windows read 1.4 to 4.5 along a valley of (lag, band) pairs that each make one split agree), so the band is only
# taken where it makes both. A car without friction keeps f = 0 (a plant whose response falls with the size of the
# change, the MDX's hard turns, reads a negative band and holds 0). The band itself is telemetry: the controller's
# friction term is what pays it, and the table is the response beyond it, which is what the feedforward (torque for a
# change in desired lateral accel) and the jerk bound are about
GAIN_FRICTIONS = (0.0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06)
GAIN_FRICTION_SPLIT = 0.10                        # normalized torque: a window's |dx| below this is a small change, above a large one
GAIN_FRICTION_MARGIN = 0.05                       # a candidate replaces the current one only by this much less mismatch (0..2)
GAIN_FRICTION_MIN_WEIGHT = 0.02                   # EW weight (~1 s of windows) each side of a split needs in a bin before it is compared
GAIN_WINDOW_GROUPS = 4                            # (small, large) x (tail, head): group = (|dx| >= split) * 2 + head
# speed classes, by GAIN_BINS_MPH index: to 20 mph, 30-40, 50 and up. The friction band is learned per class and the
# delivery fit below is solved per class
SPEED_CLASSES = ((0, 1, 2, 3), (4, 5), (6, 7, 8))
# The windows are fitted by instrumental variables, the desired lateral accel (through the same delay, lag and filter as
# the wire) standing in for the wire: in closed loop the P term answers the measurement's noise and the road's
# disturbances with wire of the opposite sign, so a regression of the lateral-accel change on the wire change reads
# the plant low wherever the reference is not what moved the wire (the windows in the lowest third of reference
# movement read 0.6 of the others' on the Civic, 0.5 on the MDX 3G, 0.55 on the Integra; against the reference the
# MDX's read 2.25 for 1.81 by least squares, the Integra's 1.65 for 1.2). The planner's reference does not answer the
# noise, and a window in which it did not move carries no weight. The fit counts once the instrument explains a fair
# share of the wire changes (GAIN_IV_MIN_CORR, the correlation between the two over the windows: 0.75-0.85 in every
# route; below it a bin's windows are noise answering noise)
GAIN_IV_MIN_CORR = 0.50
# Over-delivery floor (see the module docstring, "Two-way reachability"): a steady sample with the effective wire at
# or above GAIN_OVER_MIN_WIRE whose lateral accel is more than GAIN_OVER_RATIO times what the table predicts says the
# table is low here, whatever the windows have been reading; the bins are raised toward secant / GAIN_OVER_RATIO at
# GAIN_OVER_RATE per such tick. The margin is what keeps the holding branch out: the steady secant runs 1.1-1.3x the
# marginal response at 0.3-0.5 of STEER_MAX on every car measured (MDX 3G 1.35 / 1.2 at 13-18 m/s, Civic 2022 1.89 /
# 1.6, Integra 1.68 / 1.5), so the floor's equilibrium, max secant / 1.5, sits below the marginal on a healthy table
# and 1.5-3x above a poisoned one (Civic 2022 route 0000005a: 3.0 delivered against 0.9 in the table)
GAIN_OVER_RATIO = 1.5
GAIN_OVER_MIN_WIRE = 0.30                         # normalized effective torque
GAIN_OVER_RATE = 0.002                            # per such tick, toward secant / GAIN_OVER_RATIO
# qualifying ticks needed on both sides of a window (see the module docstring): 2 PLANT_TAU, the lagged wire's memory
# of a hard turn down to 13%, and past the head and tail of a change where the lag error is largest. Route 00000177
# counts 36 s of windows at 2 s (50 at 1 s, 75 with none); the simulated drives with a hard turn every 90 s read the
# gain within 1% with it and 15-40% low without
GAIN_SETTLE = 2.0                                 # s
GAIN_SETTLE_TICKS = int(round(GAIN_SETTLE / DT_CTRL))
# least squares over the windows (see the module docstring): exponentially weighted sums with GAIN_FIT_TAU of learning
# ticks of memory (about a town drive's worth), the table tracking the fit at GAIN_TRACK_RATE per learning tick (a
# drive's 40-50 s of windows takes a bin most of the way, 3 s of them moves it 15%)
GAIN_FIT_TAU = 30.0                               # s of learning ticks
GAIN_FIT_ALPHA = DT_CTRL / (GAIN_FIT_TAU + DT_CTRL)
GAIN_FIT_MIN_SXX = GAIN_MIN_DELTA_TORQUE ** 2     # the weighted mean dx^2 (of the raw change) the fit needs before the table follows it
# per learning tick, toward the fit, scaled by the fit's weight up to GAIN_SETTLED_WEIGHT: a fit on a few seconds of
# windows (they overlap tick by tick, so 5 s of them is ~5 independent readings) moves the table slowly, a well-fed
# one quickly (the Civic 2022's town bins see 5-10 s of qualifying windows in a half-hour drive, the fit on them
# wandering 0.8 -> 2.3 -> 1.5 as they came in; the MDX 3G's 30-50 s)
GAIN_TRACK_RATE = 0.003
GAIN_TRACK_FULL_WEIGHT = 0.30                     # the fit's EW weight (1.0 after many GAIN_FIT_TAU of windows) for the full rate
# a bin is settled, i.e. its gain is a measurement and not the prior on its way to one, once a fit on at least
# GAIN_SETTLED_WEIGHT of windows agrees with the table to within GAIN_SETTLED_TOL (a table that was learned agrees
# with the first few seconds of a drive's windows; the prior has to be tracked there first). The delivery fit below
# counts a sample only against a settled bin, since it reads the lateral accel in the table's unit. The weight is not
# persisted, so this is re-earned every drive
GAIN_SETTLED_WEIGHT = 0.10
GAIN_SETTLED_TOL = 0.25
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
# 1 s response lag provides that margin. No roll allowance: the planner adds it to the desired curvature.
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
WIRE_PRIOR = (1.0,) * (WIRE_BANDS - 1)            # learned bands (all but the first): linear until the car shows otherwise, every car
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
# the speed table may not learn from a window in which the response shape bent the lag-modelled wire by more than
# this (normalized torque): a shape that is too low would otherwise teach it the car's response per unit of a wire
# it never got, and the table is the feedforward and the jerk bound (route 00000129: 0.77 m/s^2 per unit at 10 mph
# against 0.4 measured, half the torque the turns needed)
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
# counted ticks between fits (steer-rate per class, and the delivery fit), see the module docstring, "Compute budget":
# the systems have RATE_TAU of memory, and a fit is a queue of solves done one per tick, so the interval must exceed
# the longest fit (the delivery fit: up to WIRE_BANDS solves per speed class)
FIT_INTERVAL = int(round(1.0 / DT_CTRL))
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
# Where the response stops is decided by the mean response of the wire above a cut (see the module docstring,
# "Learning down"): the steer-rate variance the fit loses with the bands from that cut up forced to zero, over what
# their joint regressor would explain at the first band's slope, square-rooted -- the response per unit of wire above
# the cut relative to the linear band, as the data sees it jointly, free of the pooling. A dead band comes alive only
# above WIRE_ALIVE_SLOPE, an alive band goes dead only below WIRE_DEAD_SLOPE, and between the two it holds. The
# simulated plants read 0.60-0.87 above every cut of a linear EPS (the top band on 3% of the samples included, with
# 3 deg/s of noise), 0.14-0.18 above the knee of a 0.3 slope, 0.00 above a clamp and 0.13-0.20 across the band a
# clamp halves; the MDX 3G's routes read 0.02-0.19 above 0.4 of STEER_MAX, 0.03-0.18 above 0.5 and 0.00-0.16 above
# 0.6 and 0.7, a soft knee at 0.44-0.51 (the mean over a pinned range is diluted by the dead part of it; the probe
# bound takes the dilution out, see the module docstring)
WIRE_ALIVE_SLOPE = 0.20
WIRE_DEAD_SLOPE = 0.10
# per counted tick toward zero while the verdict holds: 10x the learning rate, so a band persisted at 0.15 (the noise
# floor the pooled fit reads for a dead band) is dead after ~6 s of verdicts. A town drive counts only 25-40 s of
# ticks in all (route 00000181: 26 s), so the verdicts of one drive have to be enough
WIRE_DEMOTE_RATE = 0.002
# an alive band is only demoted on a window that has filled well past the first fit: the fits in the 10 s after
# RATE_MIN_SAMPLES read the response above the knee at a third of its settled value in route 00000182
WIRE_DEMOTE_MIN_SAMPLES = 2 * RATE_MIN_SAMPLES
# The wire bound (see the module docstring, "Where the wire is bounded"): per speed class and for this drive only, where
# the class's steer-rate fit reads the response dead and the class's own lateral accel agrees (its delivery floor
# known and below WIRE_FLOOR_ALIVE), once that verdict has held over this many fits (10 s of counted ticks); lifted
# when either stops saying so, as far as the persisted shape has the bands alive (see _judge_bound). The persisted
# shape never bounds by itself: it is the response where the car
# answers best over the speeds it has been driven at, and a town-only history says nothing about the highway: a bound
# needs the speed class's own witnesses, and a class without a verdict borrows the nearest higher class's only (the
# Civic 2022 delivers 0.2-0.25 per unit above 0.4 of STEER_MAX at 13-18 m/s, route 00000052, and about the same at
# 24-32 m/s, route 0000005a, so for it the bound costs nothing at any speed; a car whose assist is bounded in torque
# at one speed and not another is what this is for)
WIRE_BOUND_HOLD_FITS = RATE_MIN_SAMPLES // FIT_INTERVAL

# Delivery fit (see the module docstring, "Two-way reachability"): the second, independent estimate of the response
# shape, from the lateral accel rather than the steer rate. Over GAIN_DIFF_WINDOW windows at road speed with the
# lag-modelled raw wire above the first cut at either end, the change of the lateral accel (the centering shape taken
# out, in the table's unit) less the first band's share of the change is regressed on each band's share of it above
# the first, plus an intercept, under 1 >= beta_1 >= ... >= beta_n >= 0: a band's coefficient is the effective torque
# a unit of raw wire in it bought. Changes, as for the speed table: a pinned wire fills its bands by constant amounts,
# which a regression of levels cannot tell from its intercept (the per-cut floors it read on the MDX 3G's pinned
# routes were noise, 0.1-0.6 rising with the cut), and a level carries whatever the wheel holds without wire. It is a
# floor on the band gains: the steer-rate fit may say less (an EPS whose rate saturates reads dead there and the
# bound would then cost authority the car has), and a cut above which the car is still delivering at
# WIRE_FLOOR_ALIVE per unit cannot be where the response stops; and it is the lateral accel's consent a bound needs
DELIVERY_SCALE = np.array([0.1] * (WIRE_BANDS - 1) + [1.0])   # band regressors in band widths, then the intercept
DELIVERY_RIDGE = 1e-9
# a speed class is fitted once its settled bins hold this much EW weight (~5 s of windows at RATE_TAU; the window
# is on the lag-modelled wire, so the windows overlap and this is a few independent readings). The MDX 3G's EPS sets
# steeringPressed through most of a hard turn (route 00000127: 76% of the ticks with the wire above 0.42), so its
# routes count 700-850 windows in a 10-12 minute drive, 0.10 of weight in the 30-40 mph class
DELIVERY_MIN_WEIGHT = 0.08
# The fit is solved per speed class and the floor is the best of them, band by band: one shape is persisted for all
# speeds and the wire must never be held below a level the car answers to at some speed. Where the EPS is weaker
# than the shape says, the shortfall is reported as limiting, which is what an unmodified openpilot does everywhere
WIRE_FLOOR_ALIVE = 0.20                           # a band delivering at least this per unit of raw wire is not dead
WIRE_FLOOR_RATE = WIRE_DEMOTE_RATE                # per steady tick, toward the floor where the shape is below it


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


def _beyond_friction(dx, friction):
  # the part of a change in the wire beyond a friction band of this width, signed like the change
  return copysign(max(abs(dx) - friction, 0.0), dx)


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
  def __init__(self, lat_accel_factor, param_get=None):
    self.lat_accel_factor = (
      float(lat_accel_factor) if lat_accel_factor and lat_accel_factor > 0.1 else DEFAULT_LAT_ACCEL_FACTOR
    )
    self.gain_prior = _clip(self.lat_accel_factor, GAIN_MIN, GAIN_MAX)
    self.gains = [_clip(_load(param_get, GAIN_KEY_FMT.format(slot=mph), self.gain_prior), GAIN_MIN, GAIN_MAX)
                  for mph in GAIN_BINS_MPH]
    self.shapes = [_clip(_load(param_get, SHAPE_KEY_FMT.format(slot=_shape_slot(la)), prior), SHAPE_MIN, SHAPE_MAX)
                   for la, prior in zip(SHAPE_BINS_LAT_ACCEL, SHAPE_PRIOR, strict=True)]
    self._project_shapes()
    # the response shape: persisted bands first; a persisted clamp (the previous version's ceiling) when there are
    # none; linear otherwise, whatever the car
    persisted = [_load(param_get, WIRE_KEY_FMT.format(slot=_wire_slot(cut)), None) for cut in WIRE_CUTS[1:]]
    if any(p is not None for p in persisted):
      learned = [WIRE_PRIOR[i] if p is None else p for i, p in enumerate(persisted)]
    else:
      ceiling = _load(param_get, CEILING_KEY, None)
      learned = clamp_shape(ceiling) if ceiling is not None and ceiling < CEILING_MAX else WIRE_PRIOR
    self.wire_gains = [1.0] + [_clip(g, WIRE_GAIN_MIN, WIRE_GAIN_MAX) for g in learned]
    self._project_wire_gains()
    # the bound in force at the speed limit() was last called at: none until a class's verdict is in (see below)
    self.wire_dead_start = CEILING_MAX
    self.wire_bound = CEILING_MAX
    self.wire_hist = deque([0.0] * max(int(round(WIRE_DELAY / DT_CTRL)), 1), maxlen=max(int(round(WIRE_DELAY / DT_CTRL)), 1))
    self.wire_lag = 0.0             # delayed effective wire through the plant lag: the lateral accel the wire has "earned" so far
    self.clip_lag = 0.0             # what the response shape took off the delayed wire, through the same lag
    self.wire_fast = 0.0            # SHAPE_FILTER_TAU pair: the shape table's sample, and the speed table's differences
    self.lat_accel_fast = 0.0
    # each band's share of the raw (not effective) wire through the plant lag and the same filter: the delivery fit's
    # regressors. The EPS response shape acts on the wire before the plant's lag, so what the lateral accel answers
    # is the lag of each band's share, not the share of the lagged wire (a clamp at 0.5 read the band above it
    # delivering 0.4 of the latter while the wire was still climbing through it)
    self.band_lag = np.zeros(WIRE_BANDS)
    self.band_fast = np.zeros(WIRE_BANDS)
    # the reference (desired lateral accel) through the same delay, lag and filter: the speed table's instrument
    self.ref_hist = deque([0.0] * self.wire_hist.maxlen, maxlen=self.wire_hist.maxlen)
    self.ref_lag = 0.0
    self.ref_fast = 0.0
    # speed table: the triples (wire, lateral accel, reference) back to a window before the window GAIN_SETTLE ago, how
    # many consecutive ticks have qualified, and the EW sums of the windows per bin and window group (GAIN_WINDOW_GROUPS):
    # per friction candidate (the wire change mapped through that band) dx^2 and dr dx; shared, since every candidate
    # sees every window, dr^2, dr dy and the weight. Every window decays all of its bin's sums, so a mean over any set
    # of groups is sum(sums) / sum(weights) and a fresh bin's means read right from its first windows
    nb = len(GAIN_BINS_MPH)
    nf = len(GAIN_FRICTIONS)
    self.gain_hist = deque(maxlen=2 * GAIN_DIFF_TICKS + GAIN_SETTLE_TICKS + 1)
    self.gain_clean_ticks = 0
    self.gain_sxx = np.zeros((nf, nb, GAIN_WINDOW_GROUPS))
    self.gain_srx = np.zeros((nf, nb, GAIN_WINDOW_GROUPS))
    self.gain_srr = np.zeros((nb, GAIN_WINDOW_GROUPS))
    self.gain_sry = np.zeros((nb, GAIN_WINDOW_GROUPS))
    self.gain_sw = np.zeros((nb, GAIN_WINDOW_GROUPS))
    self.friction_idx = [0] * len(SPEED_CLASSES)   # the candidate each speed class tracks: GAIN_FRICTIONS[idx] is its band
    self.friction_judged = [False] * len(SPEED_CLASSES)   # ... on its own windows; an unjudged class borrows the nearest judged one's
    self.bin_class = [next(c for c, bins in enumerate(SPEED_CLASSES) if b in bins) for b in range(nb)]
    # response-shape learner: EW normal equations of steer_rate on [band regressors, angle, lat accel, 1], per speed
    # class (the EPS's authority is not the same at every speed, see the module docstring, "Where the wire is bounded")
    n = len(RATE_SCALE)
    nc = len(SPEED_CLASSES)
    self.rate_hist = deque([0.0] * RATE_DELAY_TICKS, maxlen=RATE_DELAY_TICKS)
    self.rate_xtx = np.zeros((nc, n, n))
    self.rate_xty = np.zeros((nc, n))
    self.rate_sy = np.zeros(nc)
    self.rate_syy = np.zeros(nc)
    self.rate_weight = np.zeros(nc)
    self.class_rate_samples = np.zeros(nc, dtype=int)
    self.rate_beta = np.zeros((nc, n))        # last constrained solution per class (scaled regressors), the next fit's warm start
    self.class_rate_r2 = np.zeros(nc)         # telemetry: share of the steer-rate variance each class's last fit explained
    self.rate_class = 0                       # the class the last counted tick went to
    self.rate_slope = 0.0                     # telemetry: fitted deg/s per unit of wire in the first band, last fit
    self.class_fit = np.tile(np.array(self.wire_gains), (nc, 1))   # each class's last fit, relative to the first band
    self.wire_evidence = np.zeros((nc, WIRE_BANDS), dtype=bool)   # bands each class's last fit moves, on enough data
    self.wire_demote = np.zeros((nc, WIRE_BANDS), dtype=bool)     # ... of which these go toward zero (the dead start and up)
    self.class_judged = np.zeros(nc, dtype=bool)   # a fit of the class has passed the gates this drive
    self.class_dead_from = np.full(nc, WIRE_BANDS, dtype=int)   # telemetry: the dead start each class's last fit named
    # the bound, per class: the band the dead start the class's verdict names, with the lateral accel's consent
    # (WIRE_BANDS: none); the verdict pending that hold, and the fits it has held for
    self.class_bound = np.full(nc, WIRE_BANDS, dtype=int)
    self.class_bound_pending = np.full(nc, WIRE_BANDS, dtype=int)
    self.class_bound_fits = np.zeros(nc, dtype=int)
    self.rate_counted = False       # the learner took a sample this tick
    # the fits are scheduled (see the module docstring, "Compute budget"): one is due every FIT_INTERVAL counted ticks
    # and runs as a generator doing one solve per tick, the queue serving one solve per tick in all
    self.fit_jobs = deque()                   # (kind, generator): the fits in progress, the first one running
    self.rate_fit_sample = np.zeros(nc, dtype=int)   # class_rate_samples at each class's last scheduled fit
    self.deliv_fit_sample = 0                 # deliv_samples at the last scheduled delivery fit
    # delivery fit: EW normal equations of (|lat accel| / gain(v) - first band's raw wire) on [bands above the first, 1],
    # kept per speed bin with the lateral accel undivided, so the fit is always read in the table's current unit and
    # only against the bins that have settled (see the module docstring, "Two-way reachability")
    m = len(DELIVERY_SCALE)
    self.deliv_xtx = np.zeros((nb, m, m))
    self.deliv_xty = np.zeros((nb, m))         # sum of x * |lat accel|
    self.deliv_xtb = np.zeros((nb, m))         # sum of x * (first band's raw wire)
    self.deliv_w = np.zeros(nb)
    # its own window: (lateral accel, band shares) over the last GAIN_DIFF_WINDOW, with no settle on either side (the
    # MDX 3G's EPS sets steeringPressed through most of a hard turn, and the speed table's 5 s of clean ticks around a
    # window would leave it ~300 samples in a drive, see the module docstring)
    self.deliv_hist = deque(maxlen=GAIN_DIFF_TICKS + 1)
    self.deliv_samples = 0
    self.deliv_beta = np.zeros((len(SPEED_CLASSES), m))   # last constrained solution per class, the next fit's warm start
    self.wire_floor = np.ones(WIRE_BANDS)         # telemetry: effective torque per unit of raw wire by band, from the lateral accel
    self.wire_floor_known = np.zeros(WIRE_BANDS, dtype=bool)   # bands the fit has data in
    self.deliv_floor_class = np.full((len(SPEED_CLASSES), WIRE_BANDS), np.nan)   # telemetry: the floor each class read
    self.deliv_counted = False      # the delivery fit took a sample this tick
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
    """Normalized wire torque at which the EPS stops answering at the speed limit() was last called at: the start of
    the dead band this drive's verdict for that speed names, 1.0 if none."""
    return self.wire_dead_start

  @property
  def wire_limit(self):
    """Normalized torque bound limit() held the wire to on its last call: the dead band's start plus the probe band."""
    return self.wire_bound

  @property
  def shape_dead_start(self):
    """Where the persisted shape says the EPS stops answering (the first band below WIRE_DEAD_GAIN), 1.0 if none:
    telemetry, and what an older version reads as a clamp."""
    for k in range(1, WIRE_BANDS):
      if self.wire_gains[k] < WIRE_DEAD_GAIN:
        return WIRE_BAND_LO[k]
    return CEILING_MAX

  @property
  def wire_fit(self):
    """The steer-rate fits' shape, relative to the first band: band by band the best any judged speed class read
    (the persisted shape's target), the persisted shape where none has."""
    fit = np.array(self.wire_gains)
    if self.class_judged.any():
      fit = self.class_fit[self.class_judged].max(axis=0)
    return fit

  @property
  def rate_samples(self):
    return int(self.class_rate_samples.sum())

  @property
  def rate_r2(self):
    return float(self.class_rate_r2[self.rate_class])

  def _speed_class(self, v_ego):
    pos = float(np.interp(v_ego, GAIN_BINS_MS, range(len(GAIN_BINS_MS))))
    return self.bin_class[int(round(pos))]

  def bound_band(self, v_ego):
    """The band the wire is bounded at (its start is the ceiling) at this speed, WIRE_BANDS for none: the speed
    class's own verdict this drive, or the nearest higher class's when it has none (a response that stops at some
    command level at highway speed stops there in town too, the assist being no larger; the reverse does not hold,
    the Civic 2022's authority above 0.4 of STEER_MAX grows with speed, so a town verdict never bounds the highway)."""
    c = self._speed_class(v_ego)
    for d in range(c, len(SPEED_CLASSES)):
      # a class with a dead verdict still in its hold defers to the class above too
      if self.class_judged[d] and (self.class_bound[d] < WIRE_BANDS or self.class_bound_pending[d] >= WIRE_BANDS):
        return int(self.class_bound[d])
    return WIRE_BANDS

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
                   steering_angle_deg, steering_rate_deg, desired_curvature)

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
    # the accel bound is on what the car delivers: the speed table reads the response inside the anchor band, and
    # the car centers above it (shape < 1), so the effective torque that reaches MAX_LAT_ACCEL is the bound divided
    # by the shape there. Without this a highway gain of 6-8 (Civic 2022 at 60-70 mph) would cap the wire at
    # 0.4-0.5 while the car was still well short of 3 m/s^2.
    la_cap = MAX_LAT_ACCEL / self.shape(MAX_LAT_ACCEL)
    la_out = _clip(la_jerk, -la_cap, la_cap)
    self.jerk_limited = la_jerk != la_req
    self.accel_limited = la_out != la_jerk
    # untouched requests pass through bit-exact; only a bound that bit is mapped back through the gain
    eff_out = float(torque) if la_out == la_req else self.torque_from_lat_accel(la_out, v_ego)
    # the wire that delivers it, through the inverse shape; the dead band this speed's verdict names plus the probe is
    # the bound
    band = self.bound_band(v_ego)
    self.wire_dead_start = WIRE_BAND_LO[band] if band < WIRE_BANDS else CEILING_MAX
    self.wire_bound = _clip(self.wire_dead_start + WIRE_PROBE, CEILING_MIN, CEILING_MAX)
    out, self.ceiling_limited = self._invert(eff_out)
    backstop = _clip(out, last_torque - WIRE_RATE_MAX * DT_CTRL, last_torque + WIRE_RATE_MAX * DT_CTRL)
    self.rate_limited = backstop != out
    return backstop

  def _identify(self, wire_torque, current_curvature, v_ego, active, steering_pressed, steering_angle_deg=0.0,
                steering_rate_deg=0.0, desired_curvature=0.0):
    raw_delayed_wire = self.wire_hist[0]
    self.wire_hist.append(float(wire_torque))
    delayed_ref = self.ref_hist[0]
    self.ref_hist.append(-float(desired_curvature) * v_ego * v_ego)
    self.ref_lag += PLANT_ALPHA * (delayed_ref - self.ref_lag)
    self.ref_fast += SHAPE_FILTER_ALPHA * (self.ref_lag - self.ref_fast)
    rate_delayed_wire = self.rate_hist[0]
    self.rate_hist.append(float(wire_torque))
    # the tables regress on what the EPS acted on; the shape learner below sees the raw wire
    delayed_wire = self.effective_wire(raw_delayed_wire)
    self.wire_lag += PLANT_ALPHA * (delayed_wire - self.wire_lag)
    self.clip_lag += PLANT_ALPHA * ((raw_delayed_wire - delayed_wire) - self.clip_lag)
    # measured lat accel in torque sign convention (right positive) so that lat_accel ~= gain * wire_lag
    measured = -current_curvature * v_ego * v_ego
    dxf = SHAPE_FILTER_ALPHA * (self.wire_lag - self.wire_fast)
    dyf = SHAPE_FILTER_ALPHA * (measured - self.lat_accel_fast)
    self.wire_fast += dxf
    self.lat_accel_fast += dyf
    self.band_lag += PLANT_ALPHA * (np.array(self._wire_bands(raw_delayed_wire)) - self.band_lag)
    self.band_fast += SHAPE_FILTER_ALPHA * (self.band_lag - self.band_fast)
    xf = self.wire_fast
    yf = self.lat_accel_fast

    self.press_holdoff = PRESS_HOLDOFF if steering_pressed else max(self.press_holdoff - DT_CTRL, 0.0)
    self.learning_gain = False
    self.learning_shape = False
    self.learning_wire = False
    self.rate_counted = False
    self.deliv_counted = False
    self.learning = False
    # one solve of whatever fit is in progress, on every tick: a fit works on a snapshot of its system, so a press or
    # a gap in control does not invalidate it, and finishing it keeps the queue short
    self._step_fits()
    if not (active and self.press_holdoff <= 0.0 and v_ego > MIN_LEARN_SPEED):
      # a window never spans a press or a gap in control
      self.gain_hist.clear()
      self.deliv_hist.clear()
      self.gain_clean_ticks = 0
      return

    # plant: y = gain(v) * shape(|y|) * x, with shape == 1.0 through the anchor band. Gentle curves train
    # the speed table and only the speed table; harder turns train the shape, seeing the gain as known.
    # Both regress on what the EPS actually got, so a pinned wire is a valid sample for the shape; the speed
    # table skips the windows the response shape bent (see GAIN_MAX_CLIP).
    # speed table: the change over the window, so whatever lateral accel is held without wire (the wheel
    # parked by friction, crown, bank, a hand below the threshold) drops out and the marginal response is
    # what is fitted (see the module docstring)
    self.gain_hist.append((xf, yf, self.ref_fast))
    self.deliv_hist.append((yf, self.band_fast.copy()))
    # a tick qualifies while the lag-modelled wire is inside the first response band and the lateral accel is
    # moderate: after a hard turn the lagged wire remembers torque the shape bent for about PLANT_TAU, and a window
    # that includes that memory reads the car's response to a wire it never fully got (low). The window is taken
    # GAIN_SETTLE back from the newest tick and needs GAIN_SETTLE of qualifying ticks on both sides of it: the ones
    # before so that memory has decayed, the ones after so the window is not the head of a change that then left the
    # band (see the module docstring: a part of a change reads a lag error the whole of it cancels). The gate is on
    # the wire, not on the lateral accel the table predicts for it: a table that is already low would otherwise
    # admit the hard turns and the one that is already high only the gentlest, each reading its own error back
    if abs(self.clip_lag) <= GAIN_MAX_CLIP and abs(xf) <= GAIN_MAX_WIRE and abs(yf) <= GAIN_MAX_LAT_ACCEL:
      self.gain_clean_ticks += 1
    else:
      self.gain_clean_ticks = 0
    if (self.gain_clean_ticks > GAIN_DIFF_TICKS + 2 * GAIN_SETTLE_TICKS
        and len(self.gain_hist) > 2 * GAIN_DIFF_TICKS + GAIN_SETTLE_TICKS):
      xp = self.gain_hist[0][0]
      x0, y0, r0 = self.gain_hist[GAIN_DIFF_TICKS]
      x1, y1, r1 = self.gain_hist[2 * GAIN_DIFF_TICKS]
      dx = x1 - x0
      dy = y1 - y0
      dr = r1 - r0
      # both ends on the same side of zero: the EPS has its own nonlinearity about center (the Civic 2022's windows
      # across zero read lower than the friction-in-change mapping accounts for, and with them the highway class
      # went to f = 0.04 and 7.7-7.9, over the 6.5-7.3 the steady secant measures; without them, 0.03 and 6.6-6.9).
      # On a plant with the model's own timing the gate costs nothing (1.209 for 1.2 against 1.202 with every
      # window); the change is mapped through each friction candidate (see the module docstring, "Where the speed
      # table may learn")
      if abs(dx) > GAIN_MIN_DELTA_TORQUE and (x0 > 0.0) == (x1 > 0.0):
        self.learning_gain = True
        dxs = np.array([_beyond_friction(dx, f) for f in GAIN_FRICTIONS])
        pos = float(np.interp(v_ego, GAIN_BINS_MS, range(len(GAIN_BINS_MS))))
        lo = int(np.floor(pos))
        hi = min(lo + 1, len(self.gains) - 1)
        frac = pos - lo
        c = (2 if abs(dx) >= GAIN_FRICTION_SPLIT else 0) + (1 if abs(dx) > abs(x0 - xp) else 0)
        for idx, weight in ((lo, 1.0 - frac), (hi, frac)):
          if weight <= 0.0:
            continue
          a = GAIN_FIT_ALPHA * weight
          self.gain_sxx[:, idx] *= 1.0 - a
          self.gain_srx[:, idx] *= 1.0 - a
          self.gain_srr[idx] *= 1.0 - a
          self.gain_sry[idx] *= 1.0 - a
          self.gain_sw[idx] *= 1.0 - a
          self.gain_sxx[:, idx, c] += a * dxs * dxs
          self.gain_srx[:, idx, c] += a * dr * dxs
          self.gain_srr[idx, c] += a * dr * dr
          self.gain_sry[idx, c] += a * dr * dy
          self.gain_sw[idx, c] += a
        self._select_friction(lo, hi)
        for idx, weight in ((lo, 1.0 - frac), (hi, frac)):
          fit = self.gain_fit(idx)
          if weight > 0.0 and fit is not None:
            rate = GAIN_TRACK_RATE * min(float(self.gain_sw[idx].sum()) / GAIN_TRACK_FULL_WEIGHT, 1.0)
            self.gains[idx] = _clip(self.gains[idx] + rate * weight * (fit - self.gains[idx]), GAIN_MIN, GAIN_MAX)

    # shape: the short-filtered pair, admitted while neither is trending (the lag model has aligned them;
    # what is left out is the S-bend transition it is least exact in)
    trending = (abs(dyf) / DT_CTRL > SHAPE_MAX_LEARN_JERK or abs(dxf) / DT_CTRL > SHAPE_MAX_LEARN_TORQUE_RATE)
    if (not trending and abs(xf) > MIN_LEARN_TORQUE and abs(yf) > SHAPE_ANCHOR_LAT_ACCEL and np.sign(xf) == np.sign(yf)):
      self.learning_shape = True
      xg = self.gain(v_ego) * xf
      # far more lateral accel than the table says this torque buys, with the torque well up: the table is low at
      # this speed, whatever the windows read (see the module docstring, "Two-way reachability"). The shape cannot
      # carry it (it is bounded at 1.0), so the speed table does, toward the secant less the margin
      if abs(xf) >= GAIN_OVER_MIN_WIRE and abs(yf) > GAIN_OVER_RATIO * abs(xg):
        self.learning_gain = True
        target = abs(yf) / abs(xf) / GAIN_OVER_RATIO
        pos = float(np.interp(v_ego, GAIN_BINS_MS, range(len(GAIN_BINS_MS))))
        lo = int(np.floor(pos))
        hi = min(lo + 1, len(self.gains) - 1)
        frac = pos - lo
        for idx, weight in ((lo, 1.0 - frac), (hi, frac)):
          if weight > 0.0:
            self.gains[idx] = _clip(self.gains[idx] + GAIN_OVER_RATE * weight * (target - self.gains[idx]), GAIN_MIN, GAIN_MAX)
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

    self._identify_delivery(v_ego)
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
    c = self._speed_class(v_ego)
    self.rate_class = c
    x = np.array(self._wire_bands(delayed_wire) + [float(angle_deg), float(lat_accel), 1.0]) / RATE_SCALE
    y = float(rate_deg)
    alpha = RATE_ALPHA
    self.rate_xtx[c] += alpha * (np.outer(x, x) - self.rate_xtx[c])
    self.rate_xty[c] += alpha * (x * y - self.rate_xty[c])
    self.rate_sy[c] += alpha * (y - self.rate_sy[c])
    self.rate_syy[c] += alpha * (y * y - self.rate_syy[c])
    self.rate_weight[c] += alpha * (1.0 - self.rate_weight[c])
    self.class_rate_samples[c] += 1
    if (self.class_rate_samples[c] >= RATE_MIN_SAMPLES and self.class_rate_samples[c] - self.rate_fit_sample[c] >= FIT_INTERVAL
        and not self._fit_queued(('wire', c))):
      # a fit is due: the class's verdict replaces its last one when it completes, so a fit that stops qualifying
      # stops moving the shape
      self.rate_fit_sample[c] = self.class_rate_samples[c]
      self.fit_jobs.append((('wire', c), self._fit_wire(c)))
    self._track_wire_fits()

  def _fit_queued(self, kind):
    return any(k == kind for k, _ in self.fit_jobs)

  def _step_fits(self):
    # one solve of the fit at the head of the queue (see the module docstring, "Compute budget"); a fit that has
    # finished its last solve completes on its next step and leaves the queue
    if not self.fit_jobs:
      return
    try:
      next(self.fit_jobs[0][1])
    except StopIteration:
      self.fit_jobs.popleft()

  def _track_wire_fits(self):
    # The persisted shape follows the classes jointly: it is the response where the car answers best, so a band's
    # target is the highest any class with evidence reads for it, and a band is demoted only when every class with
    # evidence has it from its dead start up (see the module docstring, "Where the wire is bounded")
    if not self.wire_evidence.any():
      return
    self.learning_wire = True
    for k in range(1, WIRE_BANDS):
      classes = np.flatnonzero(self.wire_evidence[:, k])
      if len(classes) == 0:
        continue
      if all(self.wire_demote[c, k] for c in classes):
        # demoted at the full rate while the lateral accel agrees, at the learning rate while it has nothing to say
        # (a short drive on a car with a real knee; the next drive with the floor known settles it either way), and
        # not at all where it shows the band delivering at some speed (the floor is re-fitted between the scans that
        # decide the dead start, so it is checked here too)
        if self._floor_dead(k):
          self.wire_gains[k] -= WIRE_DEMOTE_RATE * self.wire_gains[k]
        elif not self.wire_floor_known[k]:
          self.wire_gains[k] -= WIRE_LEARN_RATE * self.wire_gains[k]
      else:
        # toward the best fit, but not below what the lateral accel shows delivered, and without that evidence not
        # below the inverse's floor: the steer rate alone can take a band to where the inverse still makes it up, not
        # to where the reporting takes over
        target = max(float(self.class_fit[c, k]) for c in classes if not self.wire_demote[c, k])
        if self.wire_floor_known[k]:
          target = max(target, float(self.wire_floor[k]))
        elif self.wire_gains[k] >= WIRE_INVERT_MIN_GAIN:
          target = max(target, WIRE_INVERT_MIN_GAIN)
        self.wire_gains[k] += WIRE_LEARN_RATE * (target - self.wire_gains[k])
    self._project_wire_gains()

  def _floor_alive(self, k):
    # the lateral accel shows the band delivering at some speed
    return bool(self.wire_floor_known[k]) and float(self.wire_floor[k]) >= WIRE_FLOOR_ALIVE

  def _floor_dead(self, k):
    # ... shows it delivering nothing at every speed it has been measured at
    return bool(self.wire_floor_known[k]) and float(self.wire_floor[k]) < WIRE_FLOOR_ALIVE

  def _class_floor_dead(self, c, k):
    # ... at this speed class in particular (the floor is the best class, so this is the combined one's verdict with
    # the class among the measured)
    return self._floor_dead(k) and not np.isnan(self.deliv_floor_class[c, k])

  def _identify_delivery(self, v_ego):
    # A GAIN_DIFF_WINDOW window on the raw wire: the change of the lateral accel over it against the change of each
    # band's lag-modelled share of the wire (the EPS response shape acts before the plant lag, so the regressor is
    # the lag of each band's share and not the share of the lagged wire: a clamp at 0.5 read the band above it
    # delivering 0.4 the other way while the wire was still climbing through it). The wire is wanted above the
    # first cut at one end, both ends on the same side of zero (as for the speed table), and the centering shape is
    # taken out of the lateral accel so a hard turn the EPS delivers counts as what it is
    self.deliv_counted = False
    if v_ego <= RATE_MIN_SPEED or len(self.deliv_hist) <= GAIN_DIFF_TICKS:
      return
    y0, b0 = self.deliv_hist[0]
    y1, b1 = self.deliv_hist[-1]
    w0 = float(b0.sum())
    w1 = float(b1.sum())
    dw = w1 - w0
    pos = float(np.interp(v_ego, GAIN_BINS_MS, range(len(GAIN_BINS_MS))))
    lo = int(np.floor(pos))
    hi = min(lo + 1, len(GAIN_BINS_MS) - 1)
    frac = pos - lo
    # the steering's friction band at this speed (the speed table's, see GAIN_FRICTIONS): the part of the change
    # inside it moved nothing, and the table is the response to the part beyond it, so the bands' shares are scaled to
    # that part. A change in a dead band then reads negative (clipped to zero) rather than the friction's worth alive:
    # the Civic 2022's 0.04 is most of a 1 s window's change, and unmapped its 13-18 m/s bands read 0.1 for a
    # measured 0.2-0.25 (route 00000052, |lat accel| 0.74 at 0.4-0.5 of STEER_MAX to 1.03 at 0.9-1.0)
    friction = GAIN_FRICTIONS[self._class_friction_idx(self.bin_class[int(round(pos))])]
    if abs(dw) <= max(GAIN_MIN_DELTA_TORQUE, friction) or (w0 > 0.0) != (w1 > 0.0) or max(abs(w0), abs(w1)) <= WIRE_CUTS[0]:
      return
    self.deliv_counted = True
    bands = (b1 - b0) * ((abs(dw) - friction) / abs(dw))
    dy = (y1 - y0) / self.shape(0.5 * (abs(y0) + abs(y1)))
    x = np.array(list(bands[1:]) + [1.0]) / DELIVERY_SCALE
    # the sample goes to the two speed bins around v, the lateral accel kept undivided: the fit divides by the bin's
    # gain as it stands when it is solved, so the shape it reads moves with the table instead of lagging it
    alpha = RATE_ALPHA
    self.deliv_xtx *= 1.0 - alpha
    self.deliv_xty *= 1.0 - alpha
    self.deliv_xtb *= 1.0 - alpha
    self.deliv_w *= 1.0 - alpha
    xx = np.outer(x, x)
    for idx, weight in ((lo, 1.0 - frac), (hi, frac)):
      if weight > 0.0:
        self.deliv_xtx[idx] += alpha * weight * xx
        self.deliv_xty[idx] += alpha * weight * x * dy
        self.deliv_xtb[idx] += alpha * weight * x * bands[0]
        self.deliv_w[idx] += alpha * weight
    self.deliv_samples += 1
    if self.deliv_samples - self.deliv_fit_sample >= FIT_INTERVAL and not self._fit_queued('delivery'):
      self.deliv_fit_sample = self.deliv_samples
      self.fit_jobs.append(('delivery', self._fit_delivery()))
    # the floor: a band the lateral accel shows delivering more than the shape says is raised toward it, where it
    # shows the band alive (a band delivering less than WIRE_FLOOR_ALIVE is demoted on the steer rate's word, see
    # _identify_wire, and is not raised off the dead gain by the same number: the MDX 3G's 0.13-0.19 above its knee
    # would otherwise hold every band off dead and leave no bound)
    raised = False
    for k in range(1, WIRE_BANDS):
      if self._floor_alive(k) and self.wire_floor[k] > self.wire_gains[k] + WIRE_TOL:
        self.wire_gains[k] += WIRE_FLOOR_RATE * (float(self.wire_floor[k]) - self.wire_gains[k])
        raised = True
    if raised:
      self.learning_wire = True
      self._project_wire_gains()

  @staticmethod
  def _iv_fit(sxx, srx, srr, sry, sw, min_weight):
    # the instrumental-variable fit (nan where it is not a measurement): windows carrying min_weight, the mapped change
    # not vanished under the candidate, and the reference having moved the wire (GAIN_IV_MIN_CORR)
    ok = (sw >= min_weight) & (sxx > 0.0) & (srx > 0.0) & (srx * srx >= GAIN_IV_MIN_CORR * GAIN_IV_MIN_CORR * srr * sxx)
    return np.where(ok, sry / np.where(ok, srx, 1.0), np.nan)

  def _fits(self):
    # every candidate's fit in every bin over both window sizes, and only where the raw changes carry some size
    sw = self.gain_sw.sum(axis=1)
    srr = self.gain_srr.sum(axis=1)
    sry = self.gain_sry.sum(axis=1)
    fits = self._iv_fit(self.gain_sxx.sum(axis=2), self.gain_srx.sum(axis=2), srr[None, :], sry[None, :], sw[None, :], 1e-12)
    size_ok = self.gain_sxx[0].sum(axis=1) >= GAIN_FIT_MIN_SXX * sw
    return np.where(size_ok[None, :], fits, np.nan)

  def _group_fit(self, groups):
    # the fit per candidate and bin over a set of window groups
    g = list(groups)
    return self._iv_fit(self.gain_sxx[:, :, g].sum(axis=2), self.gain_srx[:, :, g].sum(axis=2),
                        self.gain_srr[None, :, g].sum(axis=2), self.gain_sry[None, :, g].sum(axis=2),
                        self.gain_sw[None, :, g].sum(axis=2), GAIN_FRICTION_MIN_WEIGHT)

  def _friction_mismatch(self):
    # per candidate and bin, how far apart the two sides of each split read the gain, summed over the splits (0..2, nan
    # where a side is not a measurement); the candidate under which both agree is the friction band
    with np.errstate(invalid='ignore', divide='ignore'):
      total = 0.0
      for a, b in (((0, 1), (2, 3)), ((0, 2), (1, 3))):       # small vs large, tail vs head
        fa, fb = self._group_fit(a), self._group_fit(b)
        total = total + np.abs(fa - fb) / np.maximum(np.maximum(fa, fb), 1e-9)
      return total

  def _friction_scores(self, c):
    # a speed class's mismatch per candidate: the weighted mean over its bins that can judge, each by the weight of the
    # scarcest side of its splits (small, large, tail, head pooled; a highway bin rarely sees a large change in a
    # tail window, and must still count) (nan for a candidate none of the bins can judge)
    bins = list(SPEED_CLASSES[c])
    mismatch = self._friction_mismatch()[:, bins]
    sw = self.gain_sw[bins]
    sides = np.stack([sw[:, 0] + sw[:, 1], sw[:, 2] + sw[:, 3], sw[:, 0] + sw[:, 2], sw[:, 1] + sw[:, 3]], axis=1)
    w = np.where(np.isnan(mismatch), 0.0, sides.min(axis=1)[None, :])
    with np.errstate(invalid='ignore', divide='ignore'):
      return (np.nan_to_num(mismatch) * w).sum(axis=1) / w.sum(axis=1)

  def _select_friction(self, *bins):
    # the candidate under which the small and the large windows agree, with hysteresis, for the classes these bins are in
    for c in {self.bin_class[b] for b in bins}:
      scores = self._friction_scores(c)
      if np.isnan(scores).all():
        continue
      cur = self.friction_idx[c]
      best = int(np.nanargmin(scores))
      if not self.friction_judged[c] or scores[best] <= scores[cur] - GAIN_FRICTION_MARGIN:
        self.friction_idx[c] = best
      self.friction_judged[c] = True

  def _class_friction_idx(self, c):
    # a class that has not had both window sizes yet uses the nearest class that has (the band is a torque, and the
    # Civic 2022's reads 0.03-0.05 of STEER_MAX at every speed it was measured at), the lower one first
    if self.friction_judged[c]:
      return self.friction_idx[c]
    for d in sorted(range(len(SPEED_CLASSES)), key=lambda d: (abs(d - c), d)):
      if self.friction_judged[d]:
        return self.friction_idx[d]
    return 0

  @property
  def friction(self):
    """The steering's friction band as learned, per speed class: the normalized wire change below which the car does
    not answer (telemetry; the speed table is the response to the part of a change beyond it)."""
    return tuple(GAIN_FRICTIONS[self._class_friction_idx(c)] for c in range(len(SPEED_CLASSES)))

  def gain_fit(self, idx):
    """The window fit for a speed bin under its class's friction band, or None while it is not a measurement."""
    fit = self._fits()[self._class_friction_idx(self.bin_class[idx]), idx]
    return None if np.isnan(fit) else float(fit)

  def gain_settled(self, idx):
    """Whether a speed bin's gain is a measurement: the window fit carries GAIN_SETTLED_WEIGHT and the table has
    followed it to within GAIN_SETTLED_TOL."""
    if float(self.gain_sw[idx].sum()) < GAIN_SETTLED_WEIGHT:
      return False
    fit = self.gain_fit(idx)
    return fit is not None and abs(self.gains[idx] - fit) <= GAIN_SETTLED_TOL * abs(fit)

  def _fit_delivery(self):
    # a generator: one projected-gradient solve per step, the floor published when it completes (see _step_fits)
    n = WIRE_BANDS - 1
    floor = np.zeros(WIRE_BANDS)
    known = np.zeros(WIRE_BANDS, dtype=bool)
    floor_class = np.full((len(SPEED_CLASSES), WIRE_BANDS), np.nan)
    for c, bins in enumerate(SPEED_CLASSES):
      # the effective torque the lateral accel stands for, in the table's unit (the first band's), less the first
      # band's share of the raw wire, which delivers at 1.0 by definition; only the bins whose unit is a measurement
      A = np.zeros_like(self.deliv_xtx[0])
      b = np.zeros_like(self.deliv_xty[0])
      w = 0.0
      for idx in bins:
        if self.deliv_w[idx] > 0.0 and self.gain_settled(idx):
          A += self.deliv_xtx[idx]
          b += self.deliv_xty[idx] / self.gains[idx] - self.deliv_xtb[idx]
          w += self.deliv_w[idx]
      if w < DELIVERY_MIN_WEIGHT:
        continue
      A = A / w + DELIVERY_RIDGE * np.eye(len(DELIVERY_SCALE))
      b = b / w
      step = 1.0 / float(np.linalg.eigvalsh(A)[-1])
      beta = self._solve_delivery(A, b, step, self.deliv_beta[c].copy())
      yield
      self.deliv_beta[c] = beta
      sse = float(beta @ A @ beta) - 2.0 * float(b @ beta)
      # The pooled fit splits a pinned wire's delivery among the bands it fills together (they are collinear there,
      # and the non-increasing projection shares the knee's delivery out to the dead bands above it: a clamp at 0.5
      # with the wire bounded at 0.6 read the probe band delivering half, which raised it, moved the bound up, and
      # so on to a 1.0 bound with every band at 0.18). What the data forces each band to deliver is the mean
      # delivery above its cut, the fit with the bands from there up held at zero against the full one: nothing
      # below a cut can absorb it, so it is a floor, and a band the knee falls inside reads low (conservative), as
      # in _fit_wire. A band is known once the window holds samples with the wire in or above it (its scaled
      # regressor's mean square, the fraction of the window fully in or above the band)
      active = np.diag(A)[:n] >= WIRE_MIN_BAND_ACTIVITY
      g = np.zeros(n)
      for j in range(n):
        if not active[j]:
          break
        forced = beta.copy()
        forced[j:n] = 0.0
        forced = self._solve_delivery(A, b, step, forced, dead_from=j)
        yield
        cost = max(float(forced @ A @ forced) - 2.0 * float(b @ forced) - sse, 0.0)
        g[j] = min(float(np.sqrt(cost / A[j:n, j:n].sum())) / DELIVERY_SCALE[j], 1.0)
      g = _pava_nonincreasing(g)
      floor_class[c, 1:] = np.where(active, g, np.nan)
      # the floor is the best class, band by band
      floor[1:] = np.where(active, np.maximum(floor[1:], g), floor[1:])
      known[1:] |= active
    floor[0] = 1.0
    floor[1:] = np.where(known[1:], floor[1:], 1.0)
    self.deliv_floor_class = floor_class
    self.wire_floor = floor
    self.wire_floor_known = known

  @staticmethod
  def _solve_delivery(A, b, step, beta, dead_from=None):
    # least squares under 1 >= g_1 >= ... >= g_n >= 0 on the per-unit band gains (the coefficients are per band
    # width, so the cap is the width), the intercept free, optionally with the bands from dead_from upward held at
    # zero; projected gradient, warm-started
    n = WIRE_BANDS - 1
    for _ in range(WIRE_FIT_ITERS):
      beta = beta - step * (A @ beta - b)
      beta[:n] = np.minimum(_pava_nonincreasing(beta[:n]), DELIVERY_SCALE[:n])
      if dead_from is not None:
        beta[dead_from:n] = 0.0
    return beta

  def _fit_wire(self, c):
    # a generator: one projected-gradient solve per step, the class's evidence published when it completes (see
    # _step_fits); the system is a snapshot taken at the first step
    none = np.zeros(WIRE_BANDS, dtype=bool)
    w = self.rate_weight[c]
    yy = self.rate_syy[c] / w
    var_y = yy - (self.rate_sy[c] / w) ** 2
    # no steering going on over the samples: nothing to explain
    if var_y < RATE_MIN_RATE_STD * RATE_MIN_RATE_STD:
      self.wire_evidence[c] = none
      return
    A = self.rate_xtx[c] / w + RATE_RIDGE * np.eye(len(RATE_SCALE))
    b = self.rate_xty[c] / w
    # least squares under beta_0 >= beta_1 >= ... >= beta_{n-1} >= 0 on the band coefficients (all in the same units:
    # deg/s per band width), by projected gradient from the last solution (the first one from the unconstrained
    # solution, projected)
    step = 1.0 / float(np.linalg.eigvalsh(A)[-1])
    beta = self.rate_beta[c].copy()
    if not beta.any():
      try:
        beta = np.linalg.solve(A, b)
      except np.linalg.LinAlgError:
        self.wire_evidence[c] = none
        return
      beta[:WIRE_BANDS] = _pava_nonincreasing(beta[:WIRE_BANDS])
    beta = self._solve_bands(A, b, step, beta)
    yield
    self.rate_beta[c] = beta
    sse = self._sse(yy, A, b, beta)
    self.class_rate_r2[c] = 1.0 - sse / var_y
    k0 = float(beta[0])
    self.rate_slope = k0 / RATE_SCALE[0]
    # the wire must move the wheel its own way, and the fit must explain a fair share of the steering, or the samples
    # are something else (the driver below the press threshold, a sign convention error, a drive the delay model does
    # not fit); a band is informed only if the window holds samples with the wire in or above it
    if k0 <= 0.0 or self.class_rate_r2[c] < RATE_MIN_R2:
      self.wire_evidence[c] = none
      return
    activity = np.diag(A)[:WIRE_BANDS]
    # Where does the response stop? The pooled fit cannot say: a pinned wire fills every band at once, so the bands
    # above the knee are collinear there and the non-increasing projection pools them with the knee into one value
    # (0.10-0.27 of the first band in routes 00000177 / 00000181 / 00000182, the same number for every band from 0.4
    # to 0.8), which raises a dead band and lowers an alive one toward it. The mean response above each cut decides
    # instead (see the module docstring, "Learning down"): the dead start is the lowest active band that is dead and
    # not shown alive, or alive and shown dead. Forcing from a higher band is a weaker restriction, so the first band
    # that settles settles every band above it, and the fit with those bands forced to zero, no longer pooled with
    # them, is the estimate for the bands below. A band the knee falls inside reads by the wire above it: swept to
    # full scale, the little it answers is averaged with the dead range above and it is called dead (the ceiling one
    # band under the knee, the probe band still driving the part that answers); that is the conservative call, and
    # the per-band alternative (what forcing a band costs beyond forcing the ones above it) read the MDX 3G's pinned
    # wire alive to 0.7-0.8 against a measured 0.50-0.58
    dead_from = WIRE_BANDS
    forced = beta
    for k in range(1, WIRE_BANDS):
      if activity[k] < WIRE_MIN_BAND_ACTIVITY:
        break                     # the wire never got here (nor above: the activity is non-increasing in the band)
      if self._floor_alive(k):
        continue                  # the lateral accel shows the wire above this cut still delivering: not where it stops
      dead = beta.copy()
      dead[k:WIRE_BANDS] = 0.0
      dead = self._solve_bands(A, b, step, dead, dead_from=k)
      yield
      cost = self._sse(yy, A, b, dead) - sse
      slope = float(np.sqrt(max(cost, 0.0) / (k0 * k0 * A[k:WIRE_BANDS, k:WIRE_BANDS].sum())))
      if self.wire_gains[k] < WIRE_DEAD_GAIN:
        if slope > WIRE_ALIVE_SLOPE:
          continue                # a dead band the data cannot do without: it is raised below
      elif not (self.class_rate_samples[c] >= WIRE_DEMOTE_MIN_SAMPLES and slope < WIRE_DEAD_SLOPE):
        continue                  # an alive band goes dead on the steer rate, and only if the lateral accel does not object
      dead_from = k
      forced = dead
      break
    k0 = float(forced[0])
    if k0 <= 0.0:
      self.wire_evidence[c] = none
      return
    self.class_judged[c] = True
    self.class_dead_from[c] = dead_from
    fit = np.clip(forced[:WIRE_BANDS] / k0, WIRE_GAIN_MIN, WIRE_GAIN_MAX)
    fit[dead_from:] = 0.0
    self.class_fit[c] = fit
    self.wire_demote[c] = False
    self.wire_demote[c, dead_from:] = True
    self._judge_bound(c, dead_from)
    gains = np.array(self.wire_gains)
    # the bands below the dead start track the fit (a dead band among them was shown alive above); the ones from it
    # up are demoted toward zero at their own rate while they carry anything
    evidence = (activity >= WIRE_MIN_BAND_ACTIVITY) & (np.abs(fit - gains) > WIRE_TOL)
    evidence[dead_from:] = (activity[dead_from:] >= WIRE_MIN_BAND_ACTIVITY) & (gains[dead_from:] > 0.0)
    evidence[0] = False
    self.wire_evidence[c] = evidence

  def _judge_bound(self, c, dead_from):
    # The class's bound: the dead start, with a second witness: the lateral accel at this speed (the class's own
    # floor known there and showing nothing delivered) or the persisted shape (the band dead there already, which
    # took the lateral accel's consent on an earlier drive, or many drives without it). None otherwise, and none
    # the moment the steer rate stops saying so or the lateral accel shows the band alive (the scan skips an alive
    # band). Moving the bound down has to hold over WIRE_BOUND_HOLD_FITS fits first (the fits in the 10 s after the
    # first read the response above the knee at a third of its settled value)
    # The steer rate has every band from the dead start up dead; the bound starts at the first of them the witness
    # agrees on (the MDX 3G's knee sits inside the 0.4-0.5 band: the steer rate reads that band dead when the wire
    # has run above it, the witnesses have it half alive, and the band above is what both call dead)
    verdict = WIRE_BANDS
    for k in range(dead_from, WIRE_BANDS):
      if self._class_floor_dead(c, k) or self.wire_gains[k] < WIRE_DEAD_GAIN:
        verdict = k
        break
    # a lift does not pass a band the persisted shape still has dead: the alive reading raises that band first (toward
    # the fit at WIRE_LEARN_RATE, ~5 s of counted ticks to WIRE_DEAD_GAIN, or toward the floor at once where the lateral
    # accel shows it) and the bound follows it. Lifted on the steer rate's word alone the wire runs into the band above
    # at once, the band it was read alive on as the top one reached is read diluted by that one and dead again, and
    # the bound walks up and down with the reading while the band is raised and demoted in turn (a persisted clamp at
    # 0.416 on a car whose knee is at 0.55 never got its 0.5-0.6 band off zero that way)
    for k in range(self.class_bound[c], verdict):
      if self.wire_gains[k] < WIRE_DEAD_GAIN:
        verdict = k
        break
    if verdict >= self.class_bound[c]:
      self.class_bound[c] = verdict
      self.class_bound_pending[c] = verdict
      self.class_bound_fits[c] = 0
      return
    # the pending bound is the least restrictive of the verdicts since the last lift (a knee inside a band has the
    # fits alternate between it and the band above; both support the higher one), held for as long as they stay at
    # or below it
    if self.class_bound_pending[c] >= self.class_bound[c] or verdict > self.class_bound_pending[c]:
      self.class_bound_pending[c] = verdict
      self.class_bound_fits[c] = 1
    else:
      self.class_bound_fits[c] += 1
    if self.class_bound_fits[c] >= WIRE_BOUND_HOLD_FITS:
      self.class_bound[c] = self.class_bound_pending[c]

  @staticmethod
  def _solve_bands(A, b, step, beta, dead_from=None):
    # projected gradient on the EW normal equations: non-increasing, non-negative band coefficients, optionally with the
    # bands from dead_from upward held at zero
    for _ in range(WIRE_FIT_ITERS):
      beta = beta - step * (A @ beta - b)
      beta[:WIRE_BANDS] = _pava_nonincreasing(beta[:WIRE_BANDS])
      if dead_from is not None:
        beta[dead_from:WIRE_BANDS] = 0.0
    return beta

  @staticmethod
  def _sse(yy, A, b, beta):
    # yy: the mean square of the steer rate over the fit's samples (a snapshot, like A and b)
    return yy - 2.0 * float(b @ beta) + float(beta @ A @ beta)

  def _project_shapes(self):
    # non-increasing in lateral accel, from the anchor's 1.0 down: a bin with no data of its own inherits
    # the saturation the last measured one showed rather than the neutral prior
    ceiling = SHAPE_MAX
    for i, s in enumerate(self.shapes):
      ceiling = min(ceiling, s)
      self.shapes[i] = ceiling

  def _project_wire_gains(self):
    # non-increasing in wire level from the first band's 1.0 down, same rule as the centering shape; then the
    # derived quantities: the inverse's floored slopes and whether the shape is linear
    ceiling = WIRE_GAIN_MAX
    self.wire_gains[0] = WIRE_GAIN_MAX
    for k in range(1, WIRE_BANDS):
      ceiling = min(ceiling, _clip(self.wire_gains[k], WIRE_GAIN_MIN, WIRE_GAIN_MAX))
      self.wire_gains[k] = ceiling
    self.wire_gains_floored = [max(g, WIRE_INVERT_MIN_GAIN) for g in self.wire_gains]
    self._linear = self.wire_gains[-1] >= WIRE_GAIN_MAX

  def learned_values(self):
    values = {GAIN_KEY_FMT.format(slot=mph): float(g) for mph, g in zip(GAIN_BINS_MPH, self.gains, strict=True)}
    values.update({SHAPE_KEY_FMT.format(slot=_shape_slot(la)): float(s)
                   for la, s in zip(SHAPE_BINS_LAT_ACCEL, self.shapes, strict=True)})
    values.update({WIRE_KEY_FMT.format(slot=_wire_slot(cut)): float(g)
                   for cut, g in zip(WIRE_CUTS[1:], self.wire_gains[1:], strict=True)})
    values[CEILING_KEY] = float(self.shape_dead_start)
    return values

  @staticmethod
  def param_keys():
    return ([GAIN_KEY_FMT.format(slot=mph) for mph in GAIN_BINS_MPH] +
            [SHAPE_KEY_FMT.format(slot=_shape_slot(la)) for la in SHAPE_BINS_LAT_ACCEL] +
            [WIRE_KEY_FMT.format(slot=_wire_slot(cut)) for cut in WIRE_CUTS[1:]] +
            [CEILING_KEY])
