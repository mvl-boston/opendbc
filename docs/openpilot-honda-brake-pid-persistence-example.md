# Example openpilot-side PR for Honda brake PID persistence

This branch is a review artifact that shows how the openpilot-side half of the
`brake_pid_factor` persistence change could look when paired with the opendbc
controller hooks in PR #545.

## Goal

Persist the learned Honda Nidec brake PID integrator state across device power
cycles in the same way that openpilot already persists values like:

- `CalibrationParams`
- `LiveTorqueParameters`

The important detail is that we should persist the learned integrator state
instead of the raw per-frame `brake_pid_factor` output. On the opendbc side,
the value exported for this is `brakePIDFactorNonLowSpeed`.

## Proposed split

### opendbc

The opendbc PR adds two controller hook methods:

- `get_persistent_state()`
- `set_persistent_state(...)`

Honda Nidec uses those hooks to export/import:

- a version number
- the car fingerprint
- the learned non-lowspeed brake PID factor

That keeps opendbc free of any direct dependency on openpilot's `Params`
implementation.

### openpilot

The openpilot side would:

1. define a new persistent key, for example `HondaBrakePIDParams`
2. load that key after `CarInterface` has constructed the controller
3. pass the decoded payload into `CI.CC.set_persistent_state(...)`
4. periodically call `CI.CC.get_persistent_state()` while onroad
5. write the returned payload back to `Params` with `put_nonblocking`

## Files you would touch in openpilot

- `common/params_keys.h`
- the Python module that creates `CarInterface` and owns the `CI.CC` lifetime
  for onroad control

The exact openpilot control file varies by checkout, but the integration point
should be the code that already has access to:

- `Params()`
- the constructed `CarInterface` instance
- the periodic onroad control loop

## Patch sketch

See `docs/openpilot-honda-brake-pid-persistence-example.patch` in this branch
for a concrete example patch you can adapt in the real openpilot repo.

## Safety/robustness notes

- Only restore for Honda Nidec.
- Include a cache version.
- Include the `carFingerprint`.
- Clamp the restored value to the same sane bounds used by the controller.
- Save infrequently, not every control frame.
- Remove or ignore stale values if the schema changes.
