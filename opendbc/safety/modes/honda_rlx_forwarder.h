#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/defaults.h"

// Standalone steer-bus bridge for the 2017 Acura RLX Sport Hybrid.
//
// On this car the EPS (and the stock LKAS camera) live on a CAN bus that is separate from the
// powertrain bus the comma device is connected to. This used to be handled with a second panda
// that openpilot addressed as buses 4-6, but openpilot no longer supports multiple pandas.
//
// This mode instead turns that second panda into an unattended forwarder so the regular Honda
// Nidec port can talk to the EPS through the powertrain bus:
//
//   bus 0: powertrain bus (the same bus the comma harness bus 0 is on)
//   bus 2: steer bus, EPS side
//
//   steer bus -> powertrain bus: STEER_STATUS (0x18F), CAR_SPEED (0x309)
//   powertrain bus -> steer bus: STEERING_CONTROL (0x194), LKAS_HUD (0x33D)
//
// Everything else is blocked in both directions and nothing may be transmitted over USB, so this
// panda cannot actuate anything by itself. It is meant to be the boot default safety mode in the
// panda firmware (board/main.c) for the bridge panda; openpilot never selects it.
//
// Set param bit 0 (HONDA_RLX_FORWARDER_PARAM_SWAP_BUSES) if the wiring is the other way around,
// i.e. steer bus on bus 0 and powertrain bus on bus 2.

static int honda_rlx_forwarder_pt_bus = 0;
static int honda_rlx_forwarder_steer_bus = 2;

static safety_config honda_rlx_forwarder_init(uint16_t param) {
  const uint16_t HONDA_RLX_FORWARDER_PARAM_SWAP_BUSES = 1U;

  const bool swap_buses = GET_FLAG(param, HONDA_RLX_FORWARDER_PARAM_SWAP_BUSES);
  honda_rlx_forwarder_pt_bus = swap_buses ? 2 : 0;
  honda_rlx_forwarder_steer_bus = swap_buses ? 0 : 2;

  // no RX checks and no TX allowlist: this panda only bridges the two buses
  return (safety_config){NULL, 0, NULL, 0, false}; // NOLINT(readability/braces)
}

static bool honda_rlx_forwarder_fwd_hook(int bus_num, int addr) {
  bool allowed = false;

  if (bus_num == honda_rlx_forwarder_steer_bus) {
    // EPS feedback the Honda port reads from the powertrain bus
    allowed = (addr == 0x18F) || (addr == 0x309);
  } else if (bus_num == honda_rlx_forwarder_pt_bus) {
    // openpilot steering command and LKAS HUD, sent on the powertrain bus
    allowed = (addr == 0x194) || (addr == 0x33D);
  } else {
    // nothing is forwarded from any other bus
  }

  return !allowed;
}

const safety_hooks honda_rlx_forwarder_hooks = {
  .init = honda_rlx_forwarder_init,
  .rx = default_rx_hook,
  .tx = nooutput_tx_hook,
  .fwd = honda_rlx_forwarder_fwd_hook,
};
