"""Alpha-longitudinal lifecycle: call CarInterface.deinit when appropriate.

openpilot's card only calls CarInterface.init/apply; brand-specific deinit() hooks live in
opendbc (e.g. Honda Bosch radar re-enable). This proxy wraps CarInterface so those hooks run
without openpilot changes.
"""
import atexit
from typing import Any

from openpilot.common.params import Params

from opendbc.car.carlog import carlog
from opendbc.car.can_definitions import CanRecvCallable, CanSendCallable
from opendbc.car.interfaces import CarInterfaceBase

_active_proxy: "CarInterfaceLifecycleProxy | None" = None


def wrap_car_interface(ci: CarInterfaceBase) -> CarInterfaceBase:
  global _active_proxy
  proxy = CarInterfaceLifecycleProxy(ci)
  _active_proxy = proxy
  return proxy


def _atexit_shutdown() -> None:
  if _active_proxy is not None:
    _active_proxy.shutdown("process exit")


atexit.register(_atexit_shutdown)


class CarInterfaceLifecycleProxy:
  """Delegates to a CarInterface while managing init/deinit for alpha longitudinal."""

  def __init__(self, ci: CarInterfaceBase):
    self._ci = ci
    self._can_recv: CanRecvCallable | None = None
    self._can_send: CanSendCallable | None = None
    self._params = Params()
    self._alpha_long_prev = self._params.get_bool("AlphaLongitudinalEnabled")

  def __getattr__(self, name: str) -> Any:
    return getattr(self._ci, name)

  def init(self, CP, can_recv: CanRecvCallable, can_send: CanSendCallable) -> None:
    self._can_recv = can_recv
    self._can_send = can_send
    if CP.alphaLongitudinalAvailable and not self._params.get_bool("AlphaLongitudinalEnabled"):
      self._deinit("alpha long off at controls init")
    type(self._ci).init(CP, can_recv, can_send)

  def apply(self, *args, **kwargs):
    self._check_alpha_long_toggle()
    return self._ci.apply(*args, **kwargs)

  def shutdown(self, reason: str) -> None:
    if not self._ci.CP.alphaLongitudinalAvailable:
      return
    if self._can_recv is None or self._can_send is None:
      return
    self._deinit(reason)

  def _check_alpha_long_toggle(self) -> None:
    if not self._ci.CP.alphaLongitudinalAvailable:
      return
    alpha_long = self._params.get_bool("AlphaLongitudinalEnabled")
    if self._alpha_long_prev and not alpha_long:
      self._deinit("alpha long toggled off")
    self._alpha_long_prev = alpha_long

  def _deinit(self, reason: str) -> None:
    if self._can_recv is None or self._can_send is None:
      return
    carlog.warning(f"CarInterface.deinit ({reason})")
    type(self._ci).deinit(self._ci.CP, self._can_recv, self._can_send)
