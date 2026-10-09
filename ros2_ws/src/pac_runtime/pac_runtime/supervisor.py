"""Stage 3: Supervisor. Process state, conveyor and MISSING confirmation.

Modes (flowchart): NORMAL, PALLET_CHANGE, REPACKING, HOLD. The pallet change
stops the conveyor (HOLD) for ``T_change``; a manual change or an L4
escalation forces HOLD until the operator releases it. With the order list
known (SKU counts), boxes still expected when the conveyor has been idle for
``missing_timeout_s`` are confirmed MISSING and removed from the remaining
inventory (the planners must not keep space for them).
"""

from collections import Counter
from enum import Enum


class Mode(str, Enum):
    NORMAL = "NORMAL"
    PALLET_CHANGE = "PALLET_CHANGE"
    REPACKING = "REPACKING"
    HOLD = "HOLD"


class Supervisor:
    def __init__(self, config, expected_by_sku):
        self.cfg = config
        self.mode = Mode.NORMAL
        self.conveyor_running = True
        self.expected = Counter(expected_by_sku)
        self.arrived = Counter()
        self.missing = Counter()
        self.log = []
        self.time_in = Counter()

    def _set(self, mode, t, reason):
        self.log.append({"t": round(t, 3), "from": self.mode.value, "to": mode.value, "reason": reason})
        self.mode = mode

    # -- events ---------------------------------------------------------
    def on_arrival(self, sku):
        self.arrived[sku] += 1

    def pallet_change(self, t, manual=None):
        """Returns the time the cell is blocked."""
        manual = self.cfg.pallet_change_manual if manual is None else manual
        self._set(Mode.HOLD if manual else Mode.PALLET_CHANGE, t, "manual pallet change" if manual else "PALLET_CLOSE")
        self.conveyor_running = False
        blocked = self.cfg.pallet_change_time_s
        self.time_in[self.mode.value] += blocked
        self._set(Mode.NORMAL, t + blocked, "new pallet in place")
        self.conveyor_running = True
        return blocked

    def repacking(self, t, duration):
        self._set(Mode.REPACKING, t, "PARTIAL_REPACK")
        self.time_in[Mode.REPACKING.value] += duration
        self._set(Mode.NORMAL, t + duration, "repack done")

    def hold(self, t, reason):
        """L4 escalation / safety: HOLD until the operator releases."""
        self._set(Mode.HOLD, t, reason)
        self.conveyor_running = False
        blocked = self.cfg.operator_time_s
        self.time_in[Mode.HOLD.value] += blocked
        self._set(Mode.NORMAL, t + blocked, "operator released")
        self.conveyor_running = True
        return blocked

    def can_pick(self):
        return self.mode == Mode.NORMAL

    def outstanding(self):
        return Counter({k: v - self.arrived[k] - self.missing[k] for k, v in self.expected.items()
                        if v - self.arrived[k] - self.missing[k] > 0})

    def confirm_missing(self, t, idle_s):
        """Conveyor idle long enough: everything still expected is MISSING."""
        if idle_s < self.cfg.missing_timeout_s:
            return Counter()
        out = self.outstanding()
        if out:
            self.missing.update(out)
            self.log.append({"t": round(t, 3), "event": "MISSING", "by_sku": dict(out)})
        return out
