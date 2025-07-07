#pragma once

#include "opendbc/safety/safety_declarations.h"

// *** honda rlx red panda safety mode ***
static bool honda_rlx_red_panda_tx_hook(const CANPacket_t *to_send) {
  UNUSED(to_send);
  return true; // will update once this mode gets enabled
}

const safety_hooks alloutput_hooks = {
  .init = nooutput_init,
  .rx = default_rx_hook,
  .tx = honda_rlx_red_panda_tx_hook,
};
