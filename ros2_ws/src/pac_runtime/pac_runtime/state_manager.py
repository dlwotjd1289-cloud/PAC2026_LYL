"""Stage 8: State Manager. Single owner of the plant state; every change
bumps ``state_version`` so stale plans are detected (STALE_PLAN).

The pallet is stored as measured after execution (stage 7), not as planned.
``remaining_by_sku`` holds boxes not yet seen (order list minus arrivals and
confirmed MISSING); the current and buffered boxes live in
``tracked_boxes`` (statuses READY_FOR_PICK / BUFFERED).
"""

from collections import Counter
from dataclasses import replace

from pac_common import BoxStatus, InventoryState, PalletState, PlacedBox, PlanningContext, SystemState


class StateManager:
    def __init__(self, pallet_size, catalog, expected_by_sku, *, pallet_max_weight_kg, pallet_prefix="PALLET",
                 buffer_slots=4):
        self.pallet_size = pallet_size
        self.catalog = dict(catalog)
        self.max_load = float(pallet_max_weight_kg)
        self.prefix = pallet_prefix
        self.pallet_index = 0
        self.version = 0
        self.t = 0.0
        self.placed = []
        self.closed = []
        self.remaining = Counter(expected_by_sku)
        self.tracked = {}
        self.slots = [None] * buffer_slots
        self.buffered_at = {}
        self.decisions = 0
        self.uncertain = set()
        self.no_load = set()

    # -- reads ----------------------------------------------------------
    @property
    def pallet_id(self):
        return f"{self.prefix}-{self.pallet_index + 1:02d}"

    def snapshot(self):
        return SystemState(
            self.version, self.t,
            PalletState(self.pallet_id, self.pallet_size, tuple(self.placed)),
            InventoryState(dict(self.tracked), dict(+self.remaining)),
        )

    def context(self):
        return PlanningContext(
            catalog=self.catalog, pallet_max_weight_kg=self.max_load,
            capacity_overrides_n={b: 0.0 for b in self.no_load},
            buffer_capacity=len(self.slots),
            uncertain_box_ids=tuple(sorted(self.uncertain & set(self.tracked))),
        )

    def buffer_slots(self):
        return {i: b for i, b in enumerate(self.slots) if b is not None}

    def buffer_age(self):
        return {b: self.decisions - self.buffered_at[b] for b in self.slots if b is not None}

    def current_id(self):
        pending = [k for k, b in self.tracked.items() if b.status == BoxStatus.READY_FOR_PICK]
        return pending[0] if pending else None

    def cog_and_weight(self):
        total = sum(p.weight_kg for p in self.placed)
        if total <= 0:
            return None, 0.0
        from pac_candidates.geometry import rotated_dims

        cx = cy = 0.0
        for p in self.placed:
            dx, dy, _ = rotated_dims(p.size, p.pose.yaw)
            cx += p.weight_kg * (p.pose.x + dx / 2)
            cy += p.weight_kg * (p.pose.y + dy / 2)
        return (cx / total, cy / total), total

    # -- writes ---------------------------------------------------------
    def _bump(self):
        self.version += 1

    def arrive(self, box, uncertain=False, no_load=False):
        self.remaining[box.sku_id] -= 1
        if self.remaining[box.sku_id] <= 0:
            del self.remaining[box.sku_id]
        self.tracked[box.box_id] = replace(box, status=BoxStatus.READY_FOR_PICK)
        if uncertain:
            self.uncertain.add(box.box_id)
        if no_load:
            self.no_load.add(box.box_id)
        self._bump()

    def discard_expected(self, box_sku):
        """An arrival routed to inspection still consumes its order-list entry."""
        self.remaining[box_sku] -= 1
        if self.remaining[box_sku] <= 0:
            del self.remaining[box_sku]
        self._bump()

    def to_buffer(self, box_id, slot):
        if self.slots[slot] is not None:
            raise ValueError(f"buffer slot {slot} occupied")
        self.slots[slot] = box_id
        self.buffered_at[box_id] = self.decisions
        self.tracked[box_id] = replace(self.tracked[box_id], status=BoxStatus.BUFFERED)
        self._bump()

    def reconcile(self, size, pose, tol=0.002, ignore=None, z_tol=0.015):
        """Make a measured pose consistent with the stored state.

        Measurement noise can leave the measured box a millimetre inside a
        neighbour (e.g. the lower box was measured slightly taller than it
        is) or outside the pallet edge. Downstream planners require a
        consistent state (no overlaps, inside the pallet), so overlaps and
        protrusions are resolved: z is lifted onto the stored top of the box
        below (up to ``z_tol``, the height error of a low-confidence
        measurement), x / y are pushed out along the smaller penetration (up
        to ``tol``). Larger conflicts are left for the post check (L4).
        Returns (pose, shift in m).
        """
        from pac_candidates.geometry import rotated_dims

        dx, dy, dz = rotated_dims(size, pose.yaw)
        x, y, z = pose.x, pose.y, pose.z
        for _ in range(4):
            moved = False
            for p in self.placed:
                if p.box_id == ignore:
                    continue
                px, py, pz = rotated_dims(p.size, p.pose.yaw)
                ox = min(x + dx, p.pose.x + px) - max(x, p.pose.x)
                oy = min(y + dy, p.pose.y + py) - max(y, p.pose.y)
                oz = min(z + dz, p.pose.z + pz) - max(z, p.pose.z)
                if ox <= 1e-9 or oy <= 1e-9 or oz <= 1e-9:
                    continue
                if oz <= z_tol and p.pose.z < z and min(ox, oy) > tol:  # resting on it: lift onto its stored top
                    z = p.pose.z + pz
                elif min(ox, oy) <= tol:                 # side contact: push out
                    if ox <= oy:
                        x += -ox if x < p.pose.x else ox
                    else:
                        y += -oy if y < p.pose.y else oy
                else:
                    continue
                moved = True
            X, Y = self.pallet_size.x, self.pallet_size.y
            nx, ny = min(max(x, 0.0), X - dx) if dx <= X else x, min(max(y, 0.0), Y - dy) if dy <= Y else y
            if abs(nx - x) <= tol and abs(ny - y) <= tol and (nx, ny) != (x, y):
                x, y, moved = nx, ny, True
            if not moved:
                break
        shift = ((x - pose.x) ** 2 + (y - pose.y) ** 2 + (z - pose.z) ** 2) ** 0.5
        return replace(pose, x=x, y=y, z=z), shift

    def place(self, box_id, measured_pose):
        """Record the box at its measured pose (stage 7 result)."""
        box = self.tracked.pop(box_id)
        for i, b in enumerate(self.slots):
            if b == box_id:
                self.slots[i] = None
        self.placed.append(PlacedBox(box.box_id, box.sku_id, box.size, box.weight_kg, measured_pose))
        self._bump()
        return box

    def move_placed(self, box_id, measured_pose):
        self.placed = [replace(p, pose=measured_pose) if p.box_id == box_id else p for p in self.placed]
        self._bump()

    def reject(self, box_id):
        box = self.tracked.pop(box_id)
        for i, b in enumerate(self.slots):
            if b == box_id:
                self.slots[i] = None
        self._bump()
        return box

    def close_pallet(self):
        self.closed.append((self.pallet_id, tuple(self.placed)))
        self.placed = []
        self.pallet_index += 1
        self._bump()

    def confirm_missing(self, by_sku):
        for sku, n in by_sku.items():
            self.remaining[sku] -= n
            if self.remaining[sku] <= 0:
                del self.remaining[sku]
        if by_sku:
            self._bump()
