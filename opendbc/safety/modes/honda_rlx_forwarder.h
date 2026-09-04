#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/defaults.h"

// Standalone steer-bus bridge for the 2017 Acura RLX Sport Hybrid.
//
// On this car the EPS and the stock LKAS camera live on a CAN bus ("steer bus") that is separate
// from the powertrain bus the comma device is connected to. This used to be handled with a second
// panda that openpilot addressed as buses 4-6, but openpilot no longer supports multiple pandas.
//
// This mode instead turns that second panda into an unattended bridge. It keeps intercepting the
// LKAS camera like a regular Honda harness and additionally taps the powertrain bus, so the regular
// Honda Nidec port can drive the EPS through the powertrain bus:
//
//   bus 0: steer bus, car side (EPS)
//   bus 1: powertrain bus (the same bus the comma harness bus 0 is on)
//   bus 2: steer bus, camera side (stock LKAS camera)
//
//   camera (2) -> EPS (0):     everything except the camera's STEERING_CONTROL (0x194) and LKAS_HUD (0x33D)
//   EPS (0) -> camera (2):     everything except STEER_STATUS (0x18F) and CAR_SPEED (0x309),
//                              which go to the powertrain bus (1) instead so openpilot can read them
//   powertrain (1) -> EPS (0): openpilot's STEERING_CONTROL (0x194) and LKAS_HUD (0x33D), nothing else
//
// The firmware forwards each received frame to at most one bus and never rewrites its address, so
// openpilot has to send the stock 0x194/0x33D addresses on the powertrain bus, and the camera no
// longer sees STEER_STATUS/CAR_SPEED. The camera's own LKAS_HUD is deliberately not relayed to the
// powertrain bus: openpilot transmits 0x33D there itself and the comma panda treats a received 0x33D
// on its bus 0 as a relay malfunction.
//
// Logs from the dual-panda setup (a5cd616a92467aed|0000013b--370250c82a) show STEER_STATUS/CAR_SPEED
// only on the steer bus and never on the powertrain bus, and that the car's gateway relays openpilot's
// BRAKE_COMMAND/ACC_HUD from the powertrain bus onto the steer bus. If it turns out to relay 0x194/0x33D
// as well, set param bit 0 (HONDA_RLX_FORWARDER_PARAM_GATEWAY_RELAYS_STEER_CMDS) so this panda stops
// forwarding them itself and the EPS doesn't see every frame twice.
//
// Nothing may be transmitted over USB, so this panda cannot actuate anything by itself. It is meant
// to be the boot default safety mode in the panda firmware (board/main.c) for the bridge panda;
// openpilot never selects it.

#define HONDA_RLX_FWD_BUS_EPS 0
#define HONDA_RLX_FWD_BUS_PT 1
#define HONDA_RLX_FWD_BUS_CAMERA 2

static bool honda_rlx_forwarder_fwd_steer_cmds = true;

static safety_config honda_rlx_forwarder_init(uint16_t param) {
  const uint16_t HONDA_RLX_FORWARDER_PARAM_GATEWAY_RELAYS_STEER_CMDS = 1U;

  honda_rlx_forwarder_fwd_steer_cmds = !GET_FLAG(param, HONDA_RLX_FORWARDER_PARAM_GATEWAY_RELAYS_STEER_CMDS);

  // no RX checks and no TX allowlist: this panda only bridges buses
  return (safety_config){NULL, 0, NULL, 0, false}; // NOLINT(readability/braces)
}

static int honda_rlx_forwarder_fwd_bus_hook(int bus_num, int addr) {
  int destination_bus = -1;

  // steering command and LKAS HUD, sent by openpilot and by the stock camera
  const bool is_steer_cmd = (addr == 0x194) || (addr == 0x33D);
  // EPS feedback the Honda port reads from the powertrain bus
  const bool is_eps_status = (addr == 0x18F) || (addr == 0x309);

  if (bus_num == HONDA_RLX_FWD_BUS_CAMERA) {
    destination_bus = is_steer_cmd ? -1 : HONDA_RLX_FWD_BUS_EPS;
  } else if (bus_num == HONDA_RLX_FWD_BUS_EPS) {
    destination_bus = is_eps_status ? HONDA_RLX_FWD_BUS_PT : HONDA_RLX_FWD_BUS_CAMERA;
  } else if (bus_num == HONDA_RLX_FWD_BUS_PT) {
    destination_bus = (is_steer_cmd && honda_rlx_forwarder_fwd_steer_cmds) ? HONDA_RLX_FWD_BUS_EPS : -1;
  } else {
    // nothing is forwarded from any other bus
  }

  return destination_bus;
}

const safety_hooks honda_rlx_forwarder_hooks = {
  .init = honda_rlx_forwarder_init,
  .rx = default_rx_hook,
  .tx = nooutput_tx_hook,
  .fwd_bus = honda_rlx_forwarder_fwd_bus_hook,
};
