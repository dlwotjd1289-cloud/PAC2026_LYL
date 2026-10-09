"""Look-ahead search over the next N boxes visible on the conveyor (no learning).

At every stage-4 decision the policy tries each feasible action
(PLACE_CURRENT / BUFFER_CURRENT / RETRIEVE_BUFFER(i)) on a copy of the world
that only holds the current box, the buffer and the next ``horizon`` arrivals,
plays the copy to the end of that window and scores the final state with a
hand-written score (lost pallet volume). The best first action is executed;
the next box restarts the search (receding horizon).

Two search modes:
  ``pilot``  after the first action every decision is taken by ``RulePolicy``
             (cheap: one rollout per first action).
  ``beam``   every decision in the window branches, keeping the ``beam`` best
             states per first action (more search, more time).

Safety is unchanged: every simulated step goes through the same 5-1/5-2
masks as the real world, so the search only chooses among safe options.
Boxes beyond the window are never read.

Speed-ups (see docs/taehyeon/lookahead.md):
  * light clone: lists/dicts copied, immutable data and the candidate backend
    shared (deepcopy took ~0.7 s per state mid-episode),
  * shared backend cache: 5-1/5-2 results are cached per pallet snapshot, so
    siblings with the same pallet (e.g. after BUFFER_CURRENT) reuse them,
  * duplicate states merged in beam mode,
  * time budget per decision, a hard limit: the window copies check the
    clock on every 5-1/5-2 call (candidate generation, each mask check,
    repack search) and abort the branch at the limit, which keeps a reserve
    (min(0.4 s, 15 %) of the budget) for the last call that cannot be cut.
    Expansion stops a grace period earlier (min(0.5 s, 20 %) of the budget)
    so the running branch can still be scored as it is; a branch whose score
    is not ready at the limit is dropped. The rule's branch is scored first; without it the rule's action
    is returned (only scored branches can be chosen).
"""

from collections import Counter
from dataclasses import dataclass, field, replace
import math
import time

from pac_common import BoxStatus

import numpy as np

from .actions import from_index, to_index
from .rules import RulePolicy
from .world import PalletizingWorld, box_volume


@dataclass
class LookaheadConfig:
    horizon: int = 3            # boxes after the current one visible on the conveyor
    mode: str = "pilot"         # pilot | beam
    beam: int = 3               # beam mode: states kept per first action and level
    time_budget_s: float = 0.0  # per decision; 0 = no limit
    time_budget_ratio: float = 0.0  # if > 0: budget = ratio x pick-and-place cycle (overrides time_budget_s)
    margin: float = 0.005       # leave the rule action only if the score is better by this
    # leaf score weights (unit: pallet-volume fractions, lower is better)
    w_void: float = 1.0         # empty volume trapped below the open pallet's surface
    w_rough: float = 0.3        # height std of the open pallet / pallet height
    w_buffer: float = 0.01      # per occupied buffer slot at the end of the window
    w_ng: float = 0.5           # per box sent to NG in the window
    w_time: float = 0.0005      # per second of robot time in the window
    # "dead pallet": share of the still-expected volume (buffer + visible boxes
    # + the order list) that has no safe spot on the open pallet; such a pallet
    # is closed soon, so it costs about dead_share * (1 - fill)
    w_dead: float = 1.0
    dead_top_skus: int = 6      # order-list SKUs checked (largest remaining volume first)
    dead_order_weight: float = 1.0  # weight of the order-list volume vs the boxes actually seen
    placer: str = "layer"       # 5-3 rule used by the world and inside the search: layer | dblf
    objective: str = "legacy"   # legacy | j_proxy (cost-scaled terminal estimate, not exact final J)
    pallet_cost_s: float = 2368.0  # fixed user assumption; not measured field cost
    measured_preview_count: int | None = None  # None: every visible weight is measured; integrated profile: 5

    def __post_init__(self):
        if type(self.horizon) is not int or self.horizon < 0 or type(self.beam) is not int or self.beam < 1:
            raise ValueError("horizon must be a nonnegative integer and beam a positive integer")
        if type(self.dead_top_skus) is not int or self.dead_top_skus < 0:
            raise ValueError("dead_top_skus must be a nonnegative integer")
        for name in ("margin", "w_void", "w_rough", "w_buffer", "w_ng", "w_time", "w_dead",
                     "dead_order_weight", "time_budget_s", "time_budget_ratio"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        if self.mode not in ("pilot", "beam"):
            raise ValueError("mode must be pilot or beam")
        if self.placer not in ("layer", "dblf", "merged"):
            raise ValueError("placer must be layer, dblf or merged")
        if self.objective not in ("legacy", "j_proxy"):
            raise ValueError("objective must be legacy or j_proxy")
        if isinstance(self.pallet_cost_s, bool) or not isinstance(self.pallet_cost_s, (int, float)) or not math.isfinite(self.pallet_cost_s) or self.pallet_cost_s <= 0:
            raise ValueError("pallet_cost_s must be finite and positive")
        if self.measured_preview_count is not None and (
                type(self.measured_preview_count) is not int or self.measured_preview_count < 0):
            raise ValueError("measured_preview_count must be a nonnegative integer or None")


@dataclass
class ActionBudget:
    """Search time per decision from the robot action it is planned during
    (same rule as the ROS runtime, ``runtime.yaml`` -> ``planning``):
    ``ahead_ratio`` x the action's duration, clamped to [min, max];
    ``wait_budget_s`` when nothing runs meanwhile (the robot waits).

    As ``LookaheadPolicy.budget`` on a simulated world it reads the action
    duration from the world clock: the time that passed since the previous
    decision (the previous action and the rule actions after it)."""

    ahead_ratio: float = 0.8
    min_budget_s: float = 0.3
    max_budget_s: float = 10.0
    wait_budget_s: float = 1.5

    def ahead(self, seconds):
        if seconds <= 0.0:
            return self.wait_budget_s
        return min(self.max_budget_s, max(self.min_budget_s, self.ahead_ratio * seconds))

    def __call__(self, world):
        last = getattr(self, "_last", None)
        self._last = (id(world), world.time_s)
        if last is None or last[0] != id(world):
            return self.wait_budget_s  # first decision of this world: nothing ran before it
        return self.ahead(world.time_s - last[1])

    @classmethod
    def from_planning(cls, planning):
        """From ``pac_runtime`` ``PlanningConfig`` (or any object with the fields)."""
        return cls(planning.ahead_ratio, planning.min_budget_s, planning.max_budget_s, planning.wait_budget_s)


def lookahead_config_from_dict(data):
    data = dict(data or {})
    known = LookaheadConfig.__dataclass_fields__
    unknown = set(data) - set(known)
    if unknown:
        raise ValueError(f"unknown lookahead keys: {sorted(unknown)}")
    return LookaheadConfig(**data)


def load_lookahead_config(path=None):
    """``config/taehyeon/lookahead.yaml`` (top key ``lookahead``)."""
    if path is None:
        return LookaheadConfig()
    from pathlib import Path

    import yaml

    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    return lookahead_config_from_dict(data.get("lookahead", data))


class SearchTimeout(Exception):
    """The decision's time budget ran out inside a simulated step."""


class _DeadlineBackend:
    """Candidate backend of a window copy: raises ``SearchTimeout`` past the
    deadline on the calls that do the work (5-1 generation, 5-2 checks)."""

    def __init__(self, backend, deadline):
        self._backend = backend
        self._deadline = deadline

    def _check(self):
        if time.perf_counter() > self._deadline:
            raise SearchTimeout

    def candidate_set(self, box, state):
        # the backend's own loop, run with this object as ``self`` so that
        # every candidate's mask check goes through the clock check below
        self._check()
        return type(self._backend).candidate_set(self, box, state)

    def generate_with_report(self, *args, **kwargs):
        self._check()
        return self._backend.generate_with_report(*args, **kwargs)

    def validate_constraints(self, *args, **kwargs):
        self._check()
        return self._backend.validate_constraints(*args, **kwargs)

    def __getattr__(self, name):
        return getattr(self._backend, name)


class WindowWorld(PalletizingWorld):
    """A world copy that stops when the visible window is used up (the real
    world would pull box N+1 next, which the search must not know)."""

    window_end = False
    deadline = None             # hard limit (perf_counter); None = no limit
    _guard = None

    def backend(self):
        backend = super().backend()
        if self.deadline is None:
            return backend
        if self._guard is None or self._guard._backend is not backend:
            self._guard = _DeadlineBackend(backend, self.deadline)
        return self._guard

    def _advance(self):
        if self.current is None and self.next_arrival >= len(self.arrivals):
            self.window_end = True
            self._options = None
            return
        super()._advance()

    @property
    def leaf(self):
        return self.done or self.window_end or not self.action_mask().any()


def window_clone(world, end):
    """Copy of ``world`` holding arrivals ``[:end]`` only (light, see module doc)."""
    c = object.__new__(WindowWorld)
    c.__dict__.update(world.__dict__)
    c.arrivals = world.arrivals[:end]
    measured = getattr(world, "measured_preview_count", None)
    if measured is not None:
        cutoff = world.next_arrival + measured
        known = getattr(world, "known_weight_ids", None)
        if known is None:
            known = {a.box.box_id for a in c.arrivals[:cutoff]}
            if world.current is not None:
                known.add(world.current.box.box_id)
            known.update(e.arrival.box.box_id for e in world.buffer if e is not None)
        c.known_weight_ids = frozenset(known)
        assumed = set(getattr(world, "assumed_weight_ids", ()))
        guarded = []
        for index, arrival in enumerate(c.arrivals):
            box = arrival.box
            if index >= world.next_arrival and box.box_id not in c.known_weight_ids:
                spec = world.catalog[box.sku_id]
                bounds = getattr(world, "weight_ranges", {}).get(box.sku_id)
                upper = float(bounds[1]) if bounds is not None else float(spec.weight_kg)
                box = replace(box, weight_kg=upper, source="CATALOG_WEIGHT_UPPER_BOUND")
                assumed.add(box.box_id)
            # Hidden strength/true weight is never an input to a search copy.
            guarded.append(replace(arrival, box=box, truth=None))
        c.arrivals = guarded
        c.assumed_weight_ids = frozenset(assumed)
    c.window_end = False
    for name in ("buffer", "placed", "closed", "ng", "uncertain"):
        setattr(c, name, list(getattr(world, name)))
    for name in ("capacity_overrides", "placed_truth", "_boxes"):
        setattr(c, name, dict(getattr(world, name)))
    if measured is not None:
        by_id = {arrival.box.box_id: arrival for arrival in c.arrivals}
        if c.current is not None:
            c.current = by_id.get(c.current.box.box_id, replace(c.current, truth=None))
        c.buffer = [replace(entry, arrival=by_id.get(entry.arrival.box.box_id,
                                                   replace(entry.arrival, truth=None)))
                    if entry is not None else None for entry in c.buffer]
        # Search diagnostics may use stored sizes, but cannot retain hidden truth.
        c.placed_truth = {placed.box_id: placed for placed in c.placed}
    c.counts = Counter(world.counts)
    c.remaining = Counter(world.remaining)
    return c


def _child(node):
    c = object.__new__(WindowWorld)
    c.__dict__.update(node.__dict__)
    for name in ("buffer", "placed", "closed", "ng", "uncertain"):
        setattr(c, name, list(getattr(node, name)))
    for name in ("capacity_overrides", "placed_truth", "_boxes"):
        setattr(c, name, dict(getattr(node, name)))
    c.counts = Counter(node.counts)
    c.remaining = Counter(node.remaining)
    return c


def state_key(node):
    placed = tuple(sorted(
        (p.box_id, round(p.pose.x, 4), round(p.pose.y, 4), round(p.pose.z, 4), round(p.pose.yaw, 3))
        for p in node.placed
    ))
    buffer = tuple(None if e is None else e.arrival.box.box_id for e in node.buffer)
    current = None if node.current is None else node.current.box.box_id
    return (node.pallet_index, placed, buffer, current, node.next_arrival)


def leaf_score(node, root, cfg):
    """Lost volume (pallet fractions) of the window + penalties; lower is better.

    Pallets used at the end = total box volume / pallet volume + waste of the
    closed pallets + what the open pallet will waste, so the waste of pallets
    closed in the window counts fully and the open pallet's future waste is
    estimated by its trapped void and its unevenness.
    """
    waste = sum(1.0 - p.fill for p in node.closed[len(root.closed):])
    score = waste
    if node.placed:
        grid = node.heightmap()
        cell = node.config.features.heightmap_cell_m
        under = float(grid.sum()) * cell * cell
        boxes = sum(box_volume(p.size) for p in node.placed)
        score += cfg.w_void * max(0.0, under - boxes) / node.pallet_volume
        score += cfg.w_rough * float(grid.std()) / node.pallet_size.z
    if cfg.w_dead > 0 and node.placed:
        score += cfg.w_dead * dead_share(node, cfg) * (1.0 - node.fill())
    score += cfg.w_buffer * sum(e is not None for e in node.buffer)
    score += cfg.w_ng * (len(node.ng) - len(root.ng))
    if cfg.objective == "j_proxy":
        # Shape/waste/buffer/NG terms are terminal estimates, not actual
        # expenditure. Robot/buffer/repack/change time is counted once here.
        return cfg.pallet_cost_s * score + (node.time_s - root.time_s)
    score += cfg.w_time * (node.time_s - root.time_s)
    return score


def _probe_boxes(node, cfg):
    """(box, volume weight) to test: buffer and window boxes as they are,
    plus the order list's largest-volume SKUs (types and counts only).

    The window boxes have not been picked yet, so they are still in the order
    list's remaining counts; they are taken out there so that no box is
    counted twice (the current and buffered boxes are already out)."""
    out, template = [], None
    for e in node.buffer:
        if e is not None:
            out.append((e.arrival.box, box_volume(e.arrival.box.size)))
            template = template or e.arrival.box
    if node.current is not None:
        out.append((node.current.box, box_volume(node.current.box.size)))
        template = template or node.current.box
    in_window = Counter()
    for a in node.arrivals[node.next_arrival:]:
        out.append((a.box, box_volume(a.box.size)))
        in_window[a.box.sku_id] += 1
        template = template or a.box
    if template is None and node.placed:
        template = node._box_of(node.placed[0].box_id)
    unseen = []
    for sku, n in node.remaining_by_sku().items():
        n -= in_window[sku]
        spec = node.catalog.get(sku)
        if spec is not None and n > 0:
            unseen.append((n * box_volume(spec.size), spec))
    unseen.sort(key=lambda t: -t[0])
    for vol, spec in unseen[: cfg.dead_top_skus]:
        if template is None:
            break
        probe = replace(template, box_id=f"PROBE-{spec.sku_id}", sku_id=spec.sku_id, size=spec.size,
                        weight_kg=spec.weight_kg, allowed_yaws_rad=spec.allowed_yaws_rad,
                        status=BoxStatus.READY_FOR_PICK)
        out.append((probe, vol * cfg.dead_order_weight))
    return out


def has_safe_spot(backend, box, state):
    """True if 5-1 yields a 5-2-valid candidate (stops at the first one;
    lowest candidates first since they pass most often)."""
    gen = backend.generate_with_report(box, state)
    for c in sorted(gen.candidates, key=lambda c: c.target_pose.z):
        if backend.validate_constraints(box, c, state).success:
            return True
    return False


def dead_share(node, cfg):
    """Volume share of the expected boxes with no safe spot on the open pallet."""
    probes = _probe_boxes(node, cfg)
    total = sum(v for _, v in probes)
    if total <= 0:
        return 0.0
    state = node.state()
    backend = node.backend()
    seen, dead = {}, 0.0
    for box, vol in probes:  # has_safe_spot goes through the (deadline-checked) backend
        key = (box.size, round(box.weight_kg, 6), tuple(box.allowed_yaws_rad), box.box_id in node.uncertain)
        if key not in seen:
            seen[key] = not has_safe_spot(backend, box, state)
        dead += vol * seen[key]
    return dead / total


@dataclass
class SearchStats:
    decisions: int = 0
    searched: int = 0           # decisions with more than one feasible action
    changed: int = 0            # decisions where the search overrode the rule
    expansions: int = 0         # simulated steps
    timeouts: int = 0
    aborted: int = 0            # branches cut by the hard limit (not scored)
    seconds: list = field(default_factory=list)

    def summary(self):
        s = np.array(self.seconds) if self.seconds else np.zeros(1)
        return {
            "decisions": self.decisions,
            "searched": self.searched,
            "changed": self.changed,
            "expansions": self.expansions,
            "timeouts": self.timeouts,
            "aborted": self.aborted,
            "decision_s_mean": round(float(s.mean()), 3),
            "decision_s_p95": round(float(np.percentile(s, 95)), 3),
            "decision_s_max": round(float(s.max()), 3),
        }


class LookaheadPolicy:
    """``policy(world) -> HighLevelAction`` like ``RulePolicy``."""

    name = "lookahead"

    def __init__(self, hl_config, config=None, budget=None):
        """``budget``: ``ActionBudget`` (or ``world -> seconds``) for decisions
        that get no explicit ``budget_s``; ``None`` = the configured budget."""
        self.cfg = config or LookaheadConfig()
        self.budget = budget
        self.rule = RulePolicy(hl_config)
        self.hl = hl_config
        self.stats = SearchStats()

    # -- search ---------------------------------------------------------
    def _deadline_passed(self, deadline):
        return deadline is not None and time.perf_counter() > deadline

    def _step(self, node, index):
        c = _child(node)
        c.step(index)
        self.stats.expansions += 1
        return c

    def _pilot(self, node, root, deadline):
        """``deadline``: stop expanding (soft); the hard limit sits in the copies."""
        while not node.leaf:
            if self._deadline_passed(deadline):
                self.stats.timeouts += 1
                break
            node = self._step(node, to_index(self.rule(node)))
        return leaf_score(node, root, self.cfg)

    def _beam(self, node, root, deadline):
        frontier, best = [node], float("inf")
        while frontier:
            children = {}
            for n in frontier:
                if n.leaf:
                    best = min(best, leaf_score(n, root, self.cfg))
                    continue
                for k in np.flatnonzero(n.action_mask()):
                    if self._deadline_passed(deadline):
                        self.stats.timeouts += 1
                        return min([best] + [leaf_score(x, root, self.cfg) for x in frontier])
                    c = self._step(n, int(k))
                    children.setdefault(state_key(c), c)
            ranked = sorted(children.values(), key=lambda c: leaf_score(c, root, self.cfg))
            frontier = ranked[: self.cfg.beam]
        return best

    def branches(self, world, first=None):
        """Feasible first actions, the rule's action ``first`` leading."""
        actions = [int(i) for i in np.flatnonzero(world.action_mask())]
        return sorted(actions, key=lambda k: k != first)

    def scores(self, world, first=None, window_end=None, budget_s=None):
        """Score per feasible first action (index -> score). ``budget_s``
        overrides the configured budget for this decision (the runtime sets
        it from the duration of the robot action it plans during)."""
        cfg = self.cfg
        if cfg.measured_preview_count is not None:
            world.measured_preview_count = cfg.measured_preview_count
        start = time.perf_counter()
        budget = cfg.time_budget_s
        if cfg.time_budget_ratio > 0:
            budget = cfg.time_budget_ratio * self.hl.timing.place_time_s
        if budget_s is not None:
            budget = max(1e-6, float(budget_s))
        deadline = hard = None
        if budget > 0:
            hard = start + budget - min(0.4, 0.15 * budget)  # reserve: the last uncut 5-1 call
            deadline = hard - min(0.5, 0.2 * budget)          # leave time to score the running branch
        end = world.next_arrival + cfg.horizon
        if window_end is not None:  # real time: only the boxes that have reached the camera
            end = min(end, window_end)
        root = window_clone(world, end)
        root.deadline = hard
        run = self._pilot if cfg.mode == "pilot" else self._beam
        out = {}
        for k in self.branches(world, first):
            if out and self._deadline_passed(deadline):
                self.stats.timeouts += 1
                break  # the rule's branch is always tried first; the rest only while time remains
            try:
                out[k] = run(self._step(root, k), root, deadline)
            except SearchTimeout:  # hard limit inside a step or the score: branch not comparable
                self.stats.timeouts += 1
                self.stats.aborted += 1
                break
        return out

    def __call__(self, world, window_end=None, budget_s=None):
        start = time.perf_counter()
        if budget_s is None and self.budget is not None:
            budget_s = self.budget(world)
        rule_action = self.rule(world)
        self.stats.decisions += 1
        mask = world.action_mask()
        end = world.next_arrival + self.cfg.horizon
        if window_end is not None:
            end = min(end, window_end)
        # nothing visible after the current box: the window ends right after
        # the first action, so its score cannot see what a buffered box costs
        # (it would prefer parking a placeable box); there is nothing to search
        if mask.sum() <= 1 or min(end, len(world.arrivals)) <= world.next_arrival:
            self.stats.seconds.append(time.perf_counter() - start)
            return rule_action
        self.stats.searched += 1
        scores = self.scores(world, first=to_index(rule_action), window_end=window_end, budget_s=budget_s)
        rule_index = to_index(rule_action)
        if rule_index not in scores:  # not even the rule's branch finished in time
            self.stats.seconds.append(time.perf_counter() - start)
            return rule_action
        best = min(scores, key=lambda k: (scores[k], k != rule_index))
        self.stats.seconds.append(time.perf_counter() - start)
        margin = self.cfg.margin * (self.cfg.pallet_cost_s if self.cfg.objective == "j_proxy" else 1.0)
        if best != rule_index and scores[best] < scores[rule_index] - margin:
            self.stats.changed += 1
            return from_index(best, world.slots)
        return rule_action


__all__ = [
    "ActionBudget",
    "LookaheadConfig",
    "LookaheadPolicy",
    "SearchTimeout",
    "WindowWorld",
    "dead_share",
    "has_safe_spot",
    "leaf_score",
    "load_lookahead_config",
    "lookahead_config_from_dict",
    "window_clone",
]
