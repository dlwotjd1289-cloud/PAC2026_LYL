"""Stage 2: State Validator. Cross-checks label, weight and size against the
SKU catalog plus the recognition confidence and classifies anomalies:

- ``RECOGNITION_FAIL``: label not read / low confidence -> base view
  (Recover); still unknown -> Inspection / NG
- ``DAMAGED``: visible dent -> Reject + operator alert (flowchart), or
  placed with 0 N allowed on top (``damage_policy: place_no_load``)
- ``SPEC_MISMATCH``: size or weight disagrees with the SKU -> re-measure
  (base view) and continue with the measured values (uncertain, delta)
- ``OK``

Nominal sizes are per SKU (the generator uses one size per SKU); weights
vary inside the SKU weight range, so only weights outside that range count.
"""

from dataclasses import dataclass, field
from enum import Enum

from pac_common import RejectCode

from .perception import to_box_state


class Anomaly(str, Enum):
    OK = "OK"
    RECOGNITION_FAIL = "RECOGNITION_FAIL"
    DAMAGED = "DAMAGED"
    SPEC_MISMATCH = "SPEC_MISMATCH"
    UNKNOWN_SKU = "UNKNOWN_SKU"


@dataclass
class Verdict:
    kind: Anomaly
    route: str                       # "PLAN" | "INSPECTION"
    box: object = None               # measured BoxState for PLAN
    uncertain: bool = False          # -> PlanningContext.uncertain_box_ids (delta)
    no_load_on_top: bool = False     # -> capacity_overrides_n = 0
    codes: tuple = ()
    notes: list = field(default_factory=list)
    used_base_view: bool = False
    sku: str | None = None           # resolved label (after the base view), also for INSPECTION


class StateValidator:
    def __init__(self, catalog, config, weight_ranges=None):
        self.catalog = dict(catalog)
        self.cfg = config
        self.weight_ranges = dict(weight_ranges or {})

    def _size_mismatch(self, measured, nominal):
        tol = lambda v: max(self.cfg.size_tolerance_m, self.cfg.size_tolerance_ratio * v)  # noqa: E731
        m = (measured.x, measured.y, measured.z)
        for n in ((nominal.x, nominal.y, nominal.z), (nominal.y, nominal.x, nominal.z)):
            if all(abs(a - b) <= tol(b) for a, b in zip(m, n)):
                return False
        return True

    def _weight_mismatch(self, weight, sku):
        lo, hi = self.weight_ranges.get(sku, (None, self.catalog[sku].weight_kg))
        r = self.cfg.weight_tolerance_ratio
        if hi is not None and weight > hi * (1 + r):
            return True
        return lo is not None and weight < lo * (1 - r)

    def validate(self, obs, perception=None, field_box=None):
        """``perception`` / ``field_box`` allow the base-view second look."""
        verdict = self._validate(obs, perception, field_box)
        if verdict.sku is None:
            verdict.sku = verdict.box.sku_id if verdict.box is not None else self._last_label
        return verdict

    def _validate(self, obs, perception, field_box):
        self._last_label = obs.label_sku
        notes = []
        used_base = False
        if obs.label_sku is None or obs.confidence < self.cfg.min_label_confidence:
            if perception is not None:
                obs = perception.base_view(field_box, obs)
                used_base = True
                notes.append("base view: label " + ("recovered" if obs.label_sku else "still unread"))
                self._last_label = obs.label_sku
            if obs.label_sku is None:
                return Verdict(Anomaly.RECOGNITION_FAIL, "INSPECTION", codes=(RejectCode.TRACKING_LOST,),
                               notes=notes, used_base_view=used_base)
        sku = obs.label_sku
        self._last_label = sku
        if sku not in self.catalog:
            return Verdict(Anomaly.UNKNOWN_SKU, "INSPECTION", codes=(RejectCode.INVALID_STATE,), notes=notes,
                           used_base_view=used_base)
        spec = self.catalog[sku]
        if obs.visual_damage:
            if self.cfg.damage_policy == "reject":
                return Verdict(Anomaly.DAMAGED, "INSPECTION", codes=(RejectCode.INVALID_STATE,),
                               notes=notes + ["dented: rejected, operator alerted"], used_base_view=used_base)
            box = to_box_state(obs, spec)
            return Verdict(Anomaly.DAMAGED, "PLAN", box, uncertain=obs.confidence < 1.0, no_load_on_top=True,
                           notes=notes + ["dented: placed, nothing on top"], used_base_view=used_base)
        size_bad = self._size_mismatch(obs.size, spec.size)
        weight_bad = self._weight_mismatch(obs.weight_kg, sku)
        if size_bad or weight_bad:
            codes = tuple(c for c, bad in ((RejectCode.SIZE_MISMATCH, size_bad),
                                           (RejectCode.WEIGHT_MISMATCH, weight_bad)) if bad)
            if perception is not None and not used_base:
                obs = perception.base_view(field_box, obs)  # re-measure
                used_base = True
            if not self._size_mismatch(obs.size, spec.size) and not weight_bad:
                notes.append("re-measured: matches the SKU (first reading was noise)")
                return Verdict(Anomaly.OK, "PLAN", to_box_state(obs, spec, size=spec.size),
                               uncertain=False, notes=notes, used_base_view=used_base)
            notes.append("re-measured: continue with the measured size / weight")
            return Verdict(Anomaly.SPEC_MISMATCH, "PLAN", to_box_state(obs, spec), uncertain=True, codes=codes,
                           notes=notes, used_base_view=used_base)
        # nominal size per SKU; the measured weight is used (it varies per box)
        uncertain = obs.confidence < 1.0
        box = to_box_state(obs, spec, size=obs.size if uncertain else spec.size)
        return Verdict(Anomaly.OK, "PLAN", box, uncertain=uncertain, notes=notes, used_base_view=used_base)
