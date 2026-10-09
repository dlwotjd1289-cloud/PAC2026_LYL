"""Stages 5 + 6 together: the low-level choice only returns candidates the
robot can execute ("실패 시 다음 후보").

Ranking: the flat-layer / side-contact score (``layer_ranker``, the
runtime default, same rule as the look-ahead search), DBLF, or donghan's
planner (``pac_planning``). The first
``max_checks`` ranked candidates go through stage 6; when none passes, the
option counts as infeasible for stage 4 (its action is masked).
"""

from collections import Counter
from copy import deepcopy
import math
from threading import RLock
import time

from pac_highlevel.world import dblf_choice  # noqa: F401  (same DBLF key below)


# Initial soft scoring assumptions, not physical constraints or validated optima.
OFFICIAL_TIME_WEIGHT = 0.10
OFFICIAL_EXECUTION_WEIGHT = 0.05
OFFICIAL_CYCLE_REFERENCE_S = 8.0
OFFICIAL_CLEARANCE_REFERENCE_M = 0.05
OFFICIAL_DECISION_REFERENCE_S = 7.0
OFFICIAL_CHECK_BUDGET_S = 0.5  # soft: cannot interrupt one native checker call
OFFICIAL_MAX_CHECKS = 12


def official_robot_assumptions():
    return {"status": "initial_soft_assumptions_not_physical_constraints",
            "time_weight": OFFICIAL_TIME_WEIGHT,
            "execution_weight": OFFICIAL_EXECUTION_WEIGHT,
            "cycle_reference_s": OFFICIAL_CYCLE_REFERENCE_S,
            "clearance_reference_m": OFFICIAL_CLEARANCE_REFERENCE_M,
            "decision_reference_s": OFFICIAL_DECISION_REFERENCE_S,
            "check_budget_s": OFFICIAL_CHECK_BUDGET_S,
            "max_checks": OFFICIAL_MAX_CHECKS,
            "generation_timing_scope": "world_supplies_candidate_generation_otherwise_not_measured"}


def _finite_metric(value):
    if isinstance(value, bool):
        raise ValueError("Boolean is not a numeric evaluation metric")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("Nonfinite evaluation metric")
    return number


def dblf_order(valid):
    def key(c):
        p = c.target_pose
        return (round(p.z, 6), round(p.y, 6), round(p.x, 6), c.candidate_id)

    return sorted(valid, key=key)


class RobotAwarePlacer:
    wants_context = True

    def __init__(self, robot, max_checks=12, ranker=None):
        self.robot = robot
        self.max_checks = max_checks
        self.ranker = ranker            # (valid, box, state, backend) -> ordered list; None = DBLF
        self.stats = Counter()
        self.last_rejected = {}
        self.last_evaluation = {}
        self._lock = RLock()

    def __deepcopy__(self, memo):
        copied = type(self).__new__(type(self))
        memo[id(self)] = copied
        with self._lock:
            for key, value in self.__dict__.items():
                if key != "_lock":
                    setattr(copied, key, deepcopy(value, memo))
        copied._lock = RLock()
        return copied

    def __call__(self, valid, box, state, backend):
        with self._lock:
            self.last_evaluation = {}
            started = time.perf_counter()
            ordered = self.ranker(valid, box, state, backend) if self.ranker else dblf_order(valid)
            return self._select(ordered, box, state, started, time.perf_counter()-started)

    def choose_in_world(self, valid, box, state, backend, world):
        with self._lock:
            self.last_evaluation = {}
            world._placement_evaluation = {}
            rank_in_world = getattr(self.ranker, "rank_in_world", None)
            started = time.perf_counter()
            if rank_in_world is None:
                ordered = self.ranker(valid, box, state, backend) if self.ranker else dblf_order(valid)
            else:
                ordered = rank_in_world(valid, box, state, backend, world)
            selected = self._select(ordered, box, state, started, time.perf_counter()-started,
                                   getattr(world, "_candidate_generation_elapsed_s", None))
            # The actual world and forecast copies own separate immutable-by-use
            # snapshots; never read shared last_evaluation after releasing lock.
            world._placement_evaluation = deepcopy(self.last_evaluation)
            return selected

    def _select(self, ordered, box, state, started=None, ranking_elapsed_s=0., generation_elapsed_s=None):
        self.stats["calls"] += 1
        config = getattr(getattr(self.ranker, "placer", None), "cfg", None)
        if getattr(config, "official_evaluation", False):
            return self._select_official(ordered, box, state, started, ranking_elapsed_s, generation_elapsed_s)
        self.last_evaluation = {}
        chosen, verdict, rejected = self.robot.first_executable(box, ordered[: self.max_checks], state)
        self.last_rejected = rejected
        self.stats["robot_rejected"] += len(rejected)
        for v in rejected.values():
            for code in v.codes:
                self.stats["code:" + code.value] += 1
        if chosen is None:
            self.stats["no_executable"] += 1
        elif rejected:
            self.stats["fell_back"] += 1
        return chosen

    def _select_official(self, ordered, box, state, started, ranking_elapsed_s, generation_elapsed_s):
        """Compare only measured successful checks in a bounded geometric shortlist.

        Ranking wall time is common to this call's candidates. It is recorded
        separately from each candidate's predicted robot cycle time; neither is
        a measured physical cycle. Candidate generation happens before this API.
        """
        check_started = time.perf_counter()
        generation_s = 0. if generation_elapsed_s is None else _finite_metric(generation_elapsed_s)
        scope = ("ranking_robot_checks_candidate_generation_not_measured" if generation_elapsed_s is None else
                 "candidate_generation_hard_mask_ranking_robot_checks_excludes_highlevel_search")
        started = check_started if started is None else started
        details = self.ranker.placer.last_details
        shortlist = list(ordered[:max(0, min(self.max_checks, OFFICIAL_MAX_CHECKS))])
        rows, accepted, rejected = [], [], {}
        checked = 0
        common_time = ranking_elapsed_s / OFFICIAL_DECISION_REFERENCE_S
        for index, candidate in enumerate(shortlist):
            if checked and time.perf_counter()-check_started >= OFFICIAL_CHECK_BUDGET_S:
                rows.extend({"candidate_id": rest.candidate_id, "status": "unchecked_time_budget"}
                            for rest in shortlist[index:])
                break
            verdict = self.robot.validate_robot_motion(box, candidate, state)
            checked += 1
            cid = candidate.candidate_id
            row = {"candidate_id": cid}
            if not verdict.success:
                rejected[cid] = verdict
                row.update(status="robot_rejected", codes=[code.value for code in verdict.codes])
                rows.append(row)
                continue
            try:
                cycle = _finite_metric(verdict.details["cycle_time_s"])
                clearance = _finite_metric(verdict.details["arm_clearance_m"])
                if cycle < 0 or clearance < 0:
                    raise ValueError("Negative robot time or clearance")
                geometry = _finite_metric(details[cid]["score"])
                official = dict(details[cid]["official"])
                for key in ("space_cost", "balance_cost", "stability_cost"):
                    official[key] = _finite_metric(official[key])
            except (KeyError, TypeError, ValueError, OverflowError):
                row.update(status="unscored_missing_or_invalid_metrics")
                rows.append(row)
                continue
            time_cost = OFFICIAL_TIME_WEIGHT * (cycle/OFFICIAL_CYCLE_REFERENCE_S + common_time)
            execution_cost = OFFICIAL_EXECUTION_WEIGHT * (
                1.-max(0., min(1., clearance/OFFICIAL_CLEARANCE_REFERENCE_M)))
            score = geometry + time_cost + execution_cost
            if not math.isfinite(score):
                row.update(status="unscored_nonfinite_total")
                rows.append(row)
                continue
            official.update(status="complete", time_cost=time_cost, execution_cost=execution_cost,
                robot_seconds=cycle, cycle_time_s=cycle, arm_clearance_m=clearance,
                ranking_elapsed_s=ranking_elapsed_s, candidate_generation_elapsed_s=generation_s,
                computation_time_cost=OFFICIAL_TIME_WEIGHT*common_time,
                robot_cycle_time_cost=OFFICIAL_TIME_WEIGHT*cycle/OFFICIAL_CYCLE_REFERENCE_S,
                robot_check_elapsed_s=time.perf_counter()-check_started,
                elapsed_s=time.perf_counter()-started,
                geometry_score=geometry, score_total=score,
                assumptions=official_robot_assumptions())
            details[cid]["official"] = official
            row.update(official)
            rows.append(row)
            p = candidate.target_pose
            accepted.append((score, round(p.z, 6), round(p.y, 6), round(p.x, 6), cid, candidate))
        check_elapsed = time.perf_counter()-check_started
        pattern_elapsed = generation_s + ranking_elapsed_s + check_elapsed
        common_cost = OFFICIAL_TIME_WEIGHT*pattern_elapsed/OFFICIAL_DECISION_REFERENCE_S
        for row in rows:
            if row["status"] == "complete":
                delta = common_cost-row["computation_time_cost"]
                row.update(time_cost=row["time_cost"]+delta, score_total=row["score_total"]+delta,
                           computation_time_cost=common_cost, pattern_evaluation_elapsed_s=pattern_elapsed,
                           robot_check_elapsed_s=check_elapsed, scope=scope)
                details[row["candidate_id"]]["official"].update({k: v for k, v in row.items()
                                                                if k != "candidate_id"})
        # The added complete-call wall-time component is identical for every row.
        chosen = min(accepted, key=lambda item: item[:5])[-1] if accepted else None
        self.last_rejected = rejected
        self.stats["robot_rejected"] += len(rejected)
        for verdict in rejected.values():
            for code in verdict.codes:
                self.stats["code:"+code.value] += 1
        self.stats["official_checked"] += checked
        self.stats["official_unscored"] += len(ordered)-len(accepted)
        if chosen is None:
            self.stats["no_executable"] += 1
        elif rejected:
            self.stats["fell_back"] += 1
        self.last_evaluation = {
            "box_id": box.box_id, "state_version": state.state_version,
            "selected_candidate_id": chosen.candidate_id if chosen else None,
            "rows": rows, "elapsed_s": time.perf_counter()-started,
            "ranking_elapsed_s": ranking_elapsed_s,
            "robot_check_elapsed_s": check_elapsed,
            "candidate_generation_elapsed_s": generation_s,
            "pattern_evaluation_elapsed_s": pattern_elapsed, "scope": scope,
            "checked_count": checked, "scored_count": len(accepted),
            "unscored_count": len(ordered)-len(accepted),
            "shortlist_count": len(shortlist), "candidate_count": len(ordered),
            "assumptions": official_robot_assumptions(),
        }
        return chosen


def layer_ranker(config=None):
    """Order by the 5-3 flat-layer / side-contact score (``LayerPlacer``)."""
    from pac_highlevel.placement import LayerPlacer

    placer = LayerPlacer(config)

    def rank(valid, box, state, backend):
        return [t[-1] for t in sorted(placer.scores(valid, box, state, backend), key=lambda t: t[:5])]

    rank.name = "layer"
    return rank


class _MergedRanker:
    """Copyable candidate ordering with explicit per-call future context."""

    name = "merged"

    def __init__(self, config=None):
        from pac_highlevel.merged_placement import MergedPlacer

        self.placer = MergedPlacer(config)

    def __call__(self, valid, box, state, backend):
        return [row[-1] for row in sorted(self.placer.scores(valid, box, state, backend),
                                         key=lambda row: row[:5])]

    def rank_in_world(self, valid, box, state, backend, world):
        from pac_highlevel.merged_placement import future_context_from_world

        future = future_context_from_world(world, box=box, horizon=self.placer.cfg.horizon)
        return [row[-1] for row in sorted(self.placer.scores(
            valid, box, state, backend, future_context=future), key=lambda row: row[:5])]

    def provenance(self):
        from dataclasses import asdict

        return {"name": self.name, "config": asdict(self.placer.cfg),
                "future_context": "current_world_only", "new_search_or_learning": False,
                "official_robot_scoring": official_robot_assumptions()}


def merged_ranker(config=None):
    return _MergedRanker(config)


def donghan_ranker(model_path=None, planner_config=None, seed=7):
    """Rank with donghan's EMS-backed 5-3~5-6 planner.

    Candidates rejected by the planner are intentionally omitted instead of
    being appended again as DBLF fallbacks.
    """
    from pac_planning import PlannerConfig, TeamRuntimeRanker

    return TeamRuntimeRanker(
        planner_config or PlannerConfig(), model_path=model_path,
        seed=seed, mode="ahead", use_time_budget=False,
    )
