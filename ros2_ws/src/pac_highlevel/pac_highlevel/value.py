"""Per-option future value ("선택지별 AI Future Value 값, 학습·실전 동일").

The provider name is stored in the trained policy and checked on load, so a
policy trained with one provider is never deployed with another.

* ``proxy``  : heightmap flatness after the placement (1 - std/H). Fast, no
               team dependency; used for training in this repository.
* ``donghan``: donghan's 5-4 value head (``PlacementPlanner.plan`` in
               ``ranking`` mode, no rollout): predicted mean future added
               volume / pallet capacity for the chosen candidate.
"""


def proxy_value(world, box, candidate, state, option):
    return option.flatness_after


def _load_ranker(model_path):
    from pac_planning.model import DualHeadRanker

    if model_path is None:
        raise ValueError("donghan value provider needs model_path (dual_head_ranker.json)")
    return DualHeadRanker.load(model_path)


class DonghanValue:
    """Future value from donghan's 5-4 value head (``plan`` in ranking mode).

    The trained model is loaded once; without a model the planner would
    return zero future values, so a model path is required.
    """

    name = "donghan"

    def __init__(self, model_path=None, planner_config=None):
        from pac_planning import PlacementPlanner, PlannerConfig

        self._planner_cls = PlacementPlanner
        self._config = planner_config or PlannerConfig()
        self._model = _load_ranker(model_path)

    def __call__(self, world, box, candidate, state, option):
        backend = world.backend()
        planner = self._planner_cls(
            context=backend.context,
            config=self._config,
            generate_candidates=backend.generate_candidates,
            validate_constraints=backend.validate_constraints,
            model=self._model,
        )
        box = state.inventory.tracked_boxes.get(box.box_id, box)
        result = planner.plan(box, state, [candidate], mode="ranking", use_time_budget=False)
        if not result.evaluations:
            return 0.0
        return float(result.evaluations[0].future.mean)


def DonghanPlacer(model_path=None, planner_config=None, seed=7):
    """Low-level placement by donghan's 5-3~5-6 planner (instead of DBLF).

    Called with the hard-mask-valid candidates of 5-1/5-2; returns the
    planner's rank-1 candidate. Slow (one ``plan`` per option and decision),
    used for evaluation, not for PPO training. Same adapter as the runtime
    (``pac_planning.team_bridge``), so the EMS mapping and the model's rollout
    contract are checked in one place.
    """
    from pac_planning.team_bridge import TeamPlacer

    return TeamPlacer(planner_config, seed=seed, model_path=model_path)


def make_value_provider(name, **kwargs):
    if name == "proxy":
        return proxy_value
    if name == "donghan":
        return DonghanValue(**kwargs)
    raise ValueError(f"Unknown value provider {name}")
