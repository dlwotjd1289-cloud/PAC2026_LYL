"""Runtime stages 1-3, 7-8 and the 1 -> 8 loop (pac_runtime)."""

from collections import Counter
from dataclasses import replace
import random

import pytest

from th_helpers import REPO, HALF_PI, make_box

from pac_common import BoxStatus, PlacementCandidate, Pose3D, RejectCode, Size3D, SkuSpec
from pac_candidates import CandidateConfig
from pac_highlevel import HighLevelConfig, load_policy
from pac_robot_check import RobotFeasibility
from pac_runtime import (
    Anomaly,
    CellSpec,
    ExecutorSim,
    FieldBox,
    Mode,
    PerceptionSim,
    RuntimeConfig,
    RuntimeLoop,
    StateManager,
    StateValidator,
    Supervisor,
    TrueBox,
    load_runtime_config,
)

PALLET = Size3D(1.2, 1.0, 1.35)
CAT = {
    "K": SkuSpec("K", Size3D(0.4, 0.3, 0.2), 6.0, (0.0, HALF_PI), 2000.0),
    "H": SkuSpec("H", Size3D(0.6, 0.4, 0.3), 20.0, (0.0, HALF_PI), 3000.0),
}
RANGES = {"K": (2.0, 6.0), "H": (12.0, 20.0)}


def truth(i, sku="K", weight=None, size=None):
    s = CAT[sku].size if size is None else Size3D(*size)
    return replace(make_box(f"B{i:03d}", (s.x, s.y, s.z), weight or CAT[sku].weight_kg - 1, sku=sku),
                   status=BoxStatus.ON_CONVEYOR)


def quiet(**kw):
    """Perception without random anomalies (tests inject them explicitly)."""
    cfg = RuntimeConfig()
    base = dict(label_fail_probability=0.0, uncertain_probability=0.0, weight_noise_std_ratio=0.0,
                size_noise_std_m=0.0)
    p = replace(cfg.perception, **{**base, **kw})
    return replace(cfg, perception=p)


# ---------------------------------------------------------------- stage 1 + 2

def test_validator_ok_uses_nominal_size_and_measured_weight():
    cfg = quiet()
    per = PerceptionSim(cfg.perception, random.Random(0))
    val = StateValidator(CAT, cfg.validator, RANGES)
    fb = FieldBox(truth(1, weight=4.2))
    v = val.validate(per.observe(fb, 0.0), per, fb)
    assert v.kind == Anomaly.OK and v.route == "PLAN"
    assert v.box.size == CAT["K"].size and v.box.weight_kg == pytest.approx(4.2)
    assert v.box.status == BoxStatus.MEASURED and not v.uncertain


def test_label_failure_uses_the_base_view_then_inspection():
    cfg = quiet(label_fail_probability=1.0, base_view_recover_probability=1.0)
    per = PerceptionSim(cfg.perception, random.Random(0))
    val = StateValidator(CAT, cfg.validator, RANGES)
    fb = FieldBox(truth(1))
    v = val.validate(per.observe(fb, 0.0), per, fb)
    assert v.kind == Anomaly.OK and v.used_base_view
    cfg = quiet(label_fail_probability=1.0, base_view_recover_probability=0.0)
    per = PerceptionSim(cfg.perception, random.Random(0))
    v = StateValidator(CAT, cfg.validator, RANGES).validate(per.observe(fb, 0.0), per, fb)
    assert v.kind == Anomaly.RECOGNITION_FAIL and v.route == "INSPECTION"


def test_damage_policy_reject_or_no_load():
    cfg = quiet(damage_detect_probability=1.0)
    per = PerceptionSim(cfg.perception, random.Random(0))
    fb = FieldBox(truth(1), damaged=True)
    v = StateValidator(CAT, cfg.validator, RANGES).validate(per.observe(fb, 0.0), per, fb)
    assert v.kind == Anomaly.DAMAGED and v.route == "INSPECTION"
    keep = replace(cfg.validator, damage_policy="place_no_load")
    v = StateValidator(CAT, keep, RANGES).validate(per.observe(fb, 0.0), per, fb)
    assert v.kind == Anomaly.DAMAGED and v.route == "PLAN" and v.no_load_on_top


def test_spec_mismatch_is_remeasured_and_continues_with_measured_values():
    cfg = quiet()
    per = PerceptionSim(cfg.perception, random.Random(0))
    val = StateValidator(CAT, cfg.validator, RANGES)
    fb = FieldBox(truth(1, size=(0.4, 0.3, 0.26)))  # 6 cm taller than the SKU
    v = val.validate(per.observe(fb, 0.0), per, fb)
    assert v.kind == Anomaly.SPEC_MISMATCH and v.route == "PLAN" and v.uncertain
    assert v.box.size.z == pytest.approx(0.26, abs=0.005)
    assert RejectCode.SIZE_MISMATCH in v.codes
    heavy = FieldBox(truth(2, weight=9.0))  # SKU range 2..6 kg
    v = val.validate(per.observe(heavy, 0.0), per, heavy)
    assert v.kind == Anomaly.SPEC_MISMATCH and RejectCode.WEIGHT_MISMATCH in v.codes
    assert v.box.weight_kg == pytest.approx(9.0)
    # x/y swapped is the same carton, not a mismatch
    turned = FieldBox(truth(3, size=(0.3, 0.4, 0.2)))
    assert val.validate(per.observe(turned, 0.0), per, turned).kind == Anomaly.OK


# ---------------------------------------------------------------- stage 3 + 8

def test_supervisor_modes_and_missing():
    sup = Supervisor(RuntimeConfig().supervisor, {"K": 3, "H": 1})
    sup.on_arrival("K")
    assert sup.can_pick()
    blocked = sup.pallet_change(10.0)
    assert blocked == 60.0 and sup.mode == Mode.NORMAL
    assert [e["to"] for e in sup.log] == ["PALLET_CHANGE", "NORMAL"]
    assert sup.confirm_missing(100.0, idle_s=5.0) == Counter()  # not idle long enough
    out = sup.confirm_missing(100.0, idle_s=60.0)
    assert out == Counter({"K": 2, "H": 1}) and sup.outstanding() == Counter()
    sup.hold(200.0, "L4")
    assert sup.time_in["HOLD"] == 120.0


def test_state_manager_versions_inventory_and_snapshot():
    sm = StateManager(PALLET, CAT, {"K": 2, "H": 1}, pallet_max_weight_kg=500, buffer_slots=2)
    b = replace(truth(1), status=BoxStatus.MEASURED)
    sm.arrive(b, uncertain=True)
    s = sm.snapshot()
    assert s.state_version == 1 and s.inventory.remaining_by_sku == {"K": 1, "H": 1}
    assert s.inventory.tracked_boxes["B001"].status == BoxStatus.READY_FOR_PICK
    assert sm.context().uncertain_box_ids == ("B001",)
    sm.to_buffer("B001", 1)
    assert sm.buffer_slots() == {1: "B001"} and sm.current_id() is None
    sm.place("B001", Pose3D("pallet", 0.0, 0.0, 0.0))
    assert sm.buffer_slots() == {} and len(sm.snapshot().pallet.boxes) == 1
    (cx, cy), w = sm.cog_and_weight()
    assert (cx, cy) == pytest.approx((0.2, 0.15)) and w == pytest.approx(5.0)
    sm.confirm_missing({"H": 1})
    assert sm.snapshot().inventory.remaining_by_sku == {"K": 1}
    v = sm.version
    sm.close_pallet()
    assert sm.version == v + 1 and sm.pallet_id.endswith("-02") and not sm.placed


# ---------------------------------------------------------------- stage 7

def test_executor_levels_and_true_geometry():
    cfg = RuntimeConfig()
    ex = ExecutorSim(replace(cfg.execution, place_xy_noise_std_m=0.0), cfg.verify, random.Random(0))
    box = replace(truth(1), status=BoxStatus.MEASURED)
    cand = PlacementCandidate("c", "B001", Pose3D("pallet", 0.1, 0.1, 0.0), 0)
    out = ex.place(box, box.size, cand, [], PALLET)
    assert out.level == "L0" and out.true_pose.x == pytest.approx(0.1)
    # a 6 cm wider true box at the pallet edge protrudes -> L4
    edge = PlacementCandidate("c", "B001", Pose3D("pallet", 0.8, 0.0, 0.0), 0)
    out = ex.place(box, Size3D(0.46, 0.3, 0.2), edge, [], PALLET)
    assert out.level == "L4" and "PROTRUSION" in out.issues
    # settles on what is really under it; half support -> L4 LOW_SUPPORT
    under = [TrueBox("U", Size3D(0.2, 0.3, 0.2), Pose3D("pallet", 0.1, 0.1, 0.0))]
    top = PlacementCandidate("c", "B001", Pose3D("pallet", 0.1, 0.1, 0.2), 0)
    out = ex.place(box, box.size, top, under, PALLET)
    assert out.true_pose.z == pytest.approx(0.2) and any(i.startswith("LOW_SUPPORT") for i in out.issues)


def test_grip_retry_and_other_grasp():
    cfg = RuntimeConfig()
    always = ExecutorSim(replace(cfg.execution, grip_fail_probability=1.0), cfg.verify, random.Random(0))
    assert always.grip() == (False, cfg.execution.max_grip_retries + 2, True)
    never = ExecutorSim(replace(cfg.execution, grip_fail_probability=0.0), cfg.verify, random.Random(0))
    assert never.grip() == (True, 1, False)


# ---------------------------------------------------------------- 1 -> 8

def _cell(stream, expected):
    return CellSpec(stream=tuple(stream), expected_by_sku=expected, pallet_size=PALLET, catalog=CAT,
                    weight_ranges=RANGES, pallet_max_weight_kg=800.0, pallet_prefix="T")


def test_full_loop_places_everything_safely_and_handles_anomalies():
    rng = random.Random(1)
    stream = []
    for i in range(24):
        sku = "H" if rng.random() < 0.3 else "K"
        lo, hi = RANGES[sku]
        stream.append(FieldBox(truth(i, sku, weight=round(rng.uniform(lo, hi), 2))))
    stream[5] = FieldBox(stream[5].truth, damaged=True)
    expected = Counter(fb.truth.sku_id for fb in stream)
    expected["K"] += 2  # two ordered boxes never arrive
    hl = replace(HighLevelConfig(), buffer=replace(HighLevelConfig().buffer, slots=2))
    cfg = replace(quiet(damage_detect_probability=1.0), seed=3)
    out = RuntimeLoop(_cell(stream, dict(expected)), CandidateConfig(), hl, cfg, RobotFeasibility(),
                      load_policy("rule", config=hl)).run()
    assert out["placed"] + len(out["inspection"]) == 24
    assert {"box_id": "B005", "reason": "DAMAGED", "stage": 2} in out["inspection"]
    assert out["missing"] == {"K": 2}
    assert out.get("decisions", {}).get("L4", 0) == 0
    assert out["anomalies"]["DAMAGED"] == 1
    # every placed box was robot-checked: true layout within the pallet, no penetration
    for p in out["pallet_list"]:
        for b in p["layout"]:
            x, y, z, yaw = b["pose"]
            assert -0.003 <= x and -0.003 <= y and z >= 0
    events = [e["event"] for e in out["events"]]
    assert events.count("PLACE") == out["placed"] and "MISSING" in events


def test_runtime_yaml_round_trip():
    assert load_runtime_config(REPO / "config/taehyeon/runtime.yaml") == RuntimeConfig()


def test_core_event_api_like_a_real_cell():
    """Drive RuntimeCore by hand: observation -> command -> measured result."""
    from pac_runtime import ExecutionReport, RawObservation, RuntimeCore

    hl = replace(HighLevelConfig(), buffer=replace(HighLevelConfig().buffer, slots=2))
    core = RuntimeCore(_cell((), {"K": 2}), CandidateConfig(), hl, RuntimeConfig(), RobotFeasibility(),
                       load_policy("rule", config=hl))
    assert core.next_command().action == "WAIT"
    obs = RawObservation("A", "K", 4.0, Size3D(0.4, 0.3, 0.2), 1.0, False, "top", 0.0)
    assert core.on_observation(obs).kind == Anomaly.OK
    cmd = core.next_command()
    assert cmd.action == "PLACE_CURRENT" and cmd.box_id == "A"
    assert cmd.robot["cycle_time_s"] > 0 and len(cmd.robot["q_place"]) == 6
    p = cmd.candidate.target_pose
    level = core.on_result(cmd, ExecutionReport(measured_pose=Pose3D("pallet", p.x + 0.003, p.y, p.z, yaw=p.yaw)))
    assert level == "L0" and core.sm.snapshot().pallet.boxes[0].pose.x == pytest.approx(p.x + 0.003)
    # second box: the top view reports a protrusion -> L4, HOLD, box recorded at the planned pose
    core.on_observation(RawObservation("B", "K", 4.0, Size3D(0.4, 0.3, 0.2), 1.0, False, "top", 1.0))
    cmd = core.next_command()
    t0 = core.sm.t
    assert core.on_result(cmd, ExecutionReport(measured_pose=cmd.candidate.target_pose,
                                               issues=("PROTRUSION",))) == "L4"
    assert core.sm.t - t0 == pytest.approx(RuntimeConfig().supervisor.operator_time_s)
    assert core.supervisor.log[-2]["to"] == "HOLD"
    # unreadable label and no base view -> inspection, order list keeps the box for MISSING
    core.on_observation(RawObservation("C", None, 4.0, Size3D(0.4, 0.3, 0.2), 1.0, False, "top", 2.0))
    assert core.inspection[-1] == {"box_id": "C", "reason": "RECOGNITION_FAIL", "stage": 2}
    assert core.on_conveyor_idle(60.0) == Counter() and not core.has_work()


def test_core_rejects_result_after_state_changed():
    """A matching command id/version is still stale if another event changed state."""
    from pac_runtime import ExecutionReport, RawObservation, RuntimeCore

    hl = replace(HighLevelConfig(), buffer=replace(HighLevelConfig().buffer, slots=2))
    core = RuntimeCore(_cell((), {"K": 2}), CandidateConfig(), hl, RuntimeConfig(), RobotFeasibility(),
                       load_policy("rule", config=hl))
    core.on_observation(RawObservation(
        "A", "K", 4.0, Size3D(0.4, 0.3, 0.2), 1.0, False, "top", 0.0))
    cmd = core.next_command()
    core.on_observation(RawObservation(
        "B", "K", 4.0, Size3D(0.4, 0.3, 0.2), 1.0, False, "top", 1.0))
    with pytest.raises(ValueError, match="STALE_RESULT"):
        core.on_result(cmd, ExecutionReport(measured_pose=cmd.candidate.target_pose))
    assert not core.sm.placed and {"A", "B"} <= set(core.sm.tracked)


def test_order_file_and_ros_bridge_messages():
    import json

    from pac_runtime import RuntimeCore
    from pac_runtime.order import cell_from_order, load_order
    from pac_runtime.ros_node import CoreBridge

    cell = load_order(REPO / "config/taehyeon/example_order.json", CandidateConfig())
    assert cell.pallet_size.z == pytest.approx(1.35) and sum(cell.expected_by_sku.values()) > 0
    sku, spec = next(iter(cell.catalog.items()))
    assert spec.top_load_capacity_n > 0 and cell.weight_ranges[sku][0] <= spec.weight_kg
    order = {"pallet": {"size_m": [1.2, 1.0, 1.35], "id_prefix": "R"},
             "skus": {"K": {"size_m": [0.4, 0.3, 0.2], "weight_kg": [2.0, 6.0], "count": 2}}}
    hl = replace(HighLevelConfig(), buffer=replace(HighLevelConfig().buffer, slots=2))
    bridge = CoreBridge(RuntimeCore(cell_from_order(order, CandidateConfig()), CandidateConfig(), hl,
                                    RuntimeConfig(), RobotFeasibility(), load_policy("rule", config=hl)))
    obs = {"box_id": "A", "label_sku": "K", "weight_kg": 4.0, "size_m": [0.4, 0.3, 0.2]}
    verdict, cmd = bridge.on_observation(json.dumps(obs))
    assert verdict.kind == Anomaly.OK and cmd["action"] == "PLACE_CURRENT"
    x, y, z, yaw = cmd["target_min_corner"]
    cx, cy, cz, _ = cmd["target_center"]
    assert cz == pytest.approx(z + 0.1) and len(cmd["robot"]["q_place"]) == 6
    with pytest.raises(ValueError):
        bridge.on_result(json.dumps({"state_version": cmd["state_version"] + 5}))
    level, nxt = bridge.on_result(json.dumps({"state_version": cmd["state_version"], "ok": True,
                                              "measured_pose": [x, y, z, yaw]}))
    assert level == "L0" and nxt is None and bridge.status()["placed"] == 1
    missing, _ = bridge.on_idle(json.dumps({"idle_s": 60}))
    assert missing == {"K": 1}


def test_runtime_ranker_selection_and_status_provenance(tmp_path):
    """The ROS-free factory exposes Donghan's EMS ranker without changing stage 4."""
    import json

    from pac_runtime import RuntimeCore
    from pac_runtime.order import cell_from_order
    from pac_runtime.ros_node import CoreBridge, make_runtime_ranker

    assert make_runtime_ranker("dblf").name == "dblf"
    assert make_runtime_ranker().name == "layer"  # runtime default: same rule as the look-ahead search
    with pytest.raises(ValueError, match="unknown runtime ranker"):
        make_runtime_ranker("unvalidated")

    pytest.importorskip("pac_planning")  # Donghan ranker: not part of this repository
    ranker = make_runtime_ranker("donghan", seed=11)
    assert ranker.name == "donghan_ems_rollout_v1" and ranker.model is None
    order = {"pallet": {"size_m": [1.2, 1.0, 1.35]},
             "skus": {"K": {"size_m": [0.4, 0.3, 0.2], "weight_kg": [2.0, 6.0], "count": 1}}}
    hl = replace(HighLevelConfig(), buffer=replace(HighLevelConfig().buffer, slots=2))
    core = RuntimeCore(cell_from_order(order, CandidateConfig()), CandidateConfig(), hl, RuntimeConfig(),
                       RobotFeasibility(), load_policy("rule", config=hl), ranker=ranker)
    bridge = CoreBridge(core)
    before = bridge.status()
    assert before["ranker"] == "donghan_ems_rollout_v1"
    assert before["ranker_provenance"]["calls"] == 0

    obs = {"box_id": "A", "label_sku": "K", "weight_kg": 4.0, "size_m": [0.4, 0.3, 0.2]}
    _, cmd = bridge.on_observation(json.dumps(obs))
    assert cmd["action"] == "PLACE_CURRENT"
    after = bridge.status()["ranker_provenance"]
    assert after["calls"] > 0 and after["candidate_evaluations"] > 0
    assert after["ems_verified"] is True

    # A trained h2/s3 model must be deployed with its matching rollout config.
    config = tmp_path / "ranker.yaml"
    config.write_text("schema_version: 1\nplanning:\n  horizon: 2\n  scenario_count: 3\n")
    configured = make_runtime_ranker("donghan", config_path=str(config))
    assert configured.config.horizon == 2 and configured.config.scenario_count == 3


def test_state_manager_reconciles_measurement_noise():
    sm = StateManager(PALLET, CAT, {"K": 3}, pallet_max_weight_kg=500, buffer_slots=2)
    for i, pose in enumerate([Pose3D("pallet", 0.0, 0.0, 0.0), Pose3D("pallet", 0.4, 0.0, 0.0)]):
        sm.arrive(replace(truth(i), status=BoxStatus.MEASURED))
        sm.place(f"B{i:03d}", pose)
    # 1.5 mm into the right neighbour and 1 mm into the box below -> pushed out / lifted
    pose, shift = sm.reconcile(Size3D(0.4, 0.3, 0.2), Pose3D("pallet", 0.0015 + 0.4 - 0.4, 0.0, 0.199), 0.002)
    assert pose.z == pytest.approx(0.2) and shift > 0
    side, _ = sm.reconcile(Size3D(0.4, 0.3, 0.2), Pose3D("pallet", 0.8 - 0.0015, 0.0, 0.0), 0.002)
    assert side.x == pytest.approx(0.8)
    # lower box stored 3.7 mm taller than it is: the box on top is lifted onto the stored top
    lifted, _ = sm.reconcile(Size3D(0.4, 0.3, 0.2), Pose3D("pallet", 0.0, 0.0, 0.1963), 0.002)
    assert lifted.z == pytest.approx(0.2)
    # 1 mm over the pallet edge -> clamped; 1 cm penetration -> left for L4
    edge, _ = sm.reconcile(Size3D(0.4, 0.3, 0.2), Pose3D("pallet", 0.801, 0.701, 0.0), 0.002)
    assert edge.x == pytest.approx(0.8) and edge.y == pytest.approx(0.7)
    deep, shift = sm.reconcile(Size3D(0.4, 0.3, 0.2), Pose3D("pallet", 0.79, 0.0, 0.0), 0.002)
    assert deep.x == pytest.approx(0.79) and shift == 0.0


def test_review_fixes_bridge_yaw_duplicates_missing_and_base_view_sku():
    import json

    from pac_runtime import RawObservation, RuntimeCore
    from pac_runtime.order import cell_from_order
    from pac_runtime.ros_node import CoreBridge, report_from_dict

    # measured yaw stays yaw (not roll)
    r = report_from_dict({"measured_pose": [0.1, 0.2, 0.0, 1.5708]})
    assert r.measured_pose.yaw == pytest.approx(1.5708) and r.measured_pose.roll == 0.0
    hl = replace(HighLevelConfig(), buffer=replace(HighLevelConfig().buffer, slots=2))
    order = {"pallet": {"size_m": [1.2, 1.0, 1.35]},
             "skus": {"K": {"size_m": [0.4, 0.3, 0.2], "weight_kg": [2.0, 6.0], "count": 3}}}
    core = RuntimeCore(cell_from_order(order, CandidateConfig()), CandidateConfig(), hl, RuntimeConfig(),
                       RobotFeasibility(), load_policy("rule", config=hl))
    bridge = CoreBridge(core)
    obs = {"box_id": "A", "label_sku": "K", "weight_kg": 4.0, "size_m": [0.4, 0.3, 0.2]}
    _, cmd = bridge.on_observation(json.dumps(obs))
    bridge.on_result(json.dumps({"state_version": cmd["state_version"], "measured_pose": cmd["target_min_corner"]}))
    # the same box observed again is ignored, the state stays consistent
    verdict, again = bridge.on_observation(json.dumps(obs))
    assert verdict is None and again is None
    core.sm.snapshot()
    # dented box whose label only the base view read: inspection, but counted as arrived (not MISSING)
    dented = RawObservation("B", None, 4.0, Size3D(0.4, 0.3, 0.2), 1.0, True, "top", 1.0)
    v = core.on_observation(dented, base_view=lambda prev: replace(prev, label_sku="K", view="base"))
    assert v.route == "INSPECTION" and v.sku == "K"
    assert core.sm.remaining == Counter({"K": 1})
    assert core.on_conveyor_idle(60.0) == Counter({"K": 1})


def test_missing_confirmed_when_last_box_goes_to_inspection():
    stream = [FieldBox(truth(i)) for i in range(3)] + [FieldBox(truth(3), damaged=True)]
    hl = replace(HighLevelConfig(), buffer=replace(HighLevelConfig().buffer, slots=2))
    out = RuntimeLoop(_cell(stream, {"K": 6}), CandidateConfig(), hl, quiet(damage_detect_probability=1.0),
                      RobotFeasibility(), load_policy("rule", config=hl)).run()
    assert out["placed"] == 3 and out["missing"] == {"K": 2}


def test_gazebo_driver_closes_the_loop_with_the_runtime_bridge():
    """runtime CoreBridge <-> GazeboDriverCore exchanging the JSON messages of the topics."""
    import json

    from pac_robot_check import load_robot_check_config
    from pac_runtime import RuntimeCore
    from pac_runtime.gazebo_driver import GazeboCell, GazeboDriverCore, boxes_from_order
    from pac_runtime.order import cell_from_order
    from pac_runtime.ros_node import CoreBridge

    order = {"pallet": {"size_m": [1.2, 1.0, 1.35]},
             "skus": {"K": {"size_m": [0.4, 0.3, 0.2], "weight_kg": [2.0, 6.0], "count": 8},
                      "H": {"size_m": [0.6, 0.4, 0.3], "weight_kg": [12.0, 20.0], "count": 3}}}
    hl = replace(HighLevelConfig(), buffer=replace(HighLevelConfig().buffer, slots=2))
    robot = RobotFeasibility(load_robot_check_config(REPO / "config/taehyeon/robot_check_gazebo.yaml"))
    bridge = CoreBridge(RuntimeCore(cell_from_order(order, CandidateConfig()), CandidateConfig(), hl,
                                    RuntimeConfig(), robot, load_policy("lookahead", config=hl)))
    driver = GazeboDriverCore(boxes_from_order(order, seed=1), robot, GazeboCell(), visible_boxes=3)
    first = driver.first_observation()
    queue = [("preview", driver.preview()), ("obs", first)]
    spawned, removed, actions = {}, set(), Counter()
    for _ in range(200):
        if not queue:
            break
        kind, payload = queue.pop(0)
        if kind == "preview":
            bridge.on_preview(json.dumps(payload))
            assert len(payload["boxes"]) <= 3
            continue
        if kind == "obs":
            _, cmd = bridge.on_observation(json.dumps(payload))
        elif kind == "idle":
            _, cmd = bridge.on_idle(json.dumps(payload))
        else:
            _, cmd = bridge.on_result(json.dumps(payload))
        if cmd is None:
            continue
        actions[cmd["action"]] += 1
        acts = driver.on_command(cmd)
        times = [t for _, t in acts.trajectory]
        assert times == sorted(times) and all(len(q) == 6 for q, _ in acts.trajectory)
        if cmd["action"] in ("PLACE_CURRENT", "RETRIEVE_BUFFER"):
            # down and up through every checked waypoint of the vertical line, not a 2-point joint move
            path = [list(q) for q in cmd["robot"]["q_path"]]
            assert len(path) > 2 and path[0] == list(cmd["robot"]["q_approach"])
            assert path[-1] == list(cmd["robot"]["q_place"])
            moved = [q for q, _ in acts.trajectory[-(2 * len(path) - 1):]]
            assert moved == path + path[-2::-1]
        for name in acts.remove:
            removed.add(name)
            spawned.pop(name, None)
        for name, sdf, (x, y, z, yaw) in acts.spawn:
            assert "<mass>" in sdf
            # inside the Gazebo pallet (centre 1.35, -1.0; 1.2 x 1.0; deck top 0.15)
            assert 0.75 <= x <= 1.95 and -1.5 <= y <= -0.5 and z > 0.15
            spawned[name] = (x, y, z)
        queue.append(("res", acts.result))
        if acts.preview is not None:
            queue.append(("preview", acts.preview))
        if acts.observation is not None:
            queue.append(("obs", acts.observation))
        if acts.idle is not None:
            queue.append(("idle", acts.idle))
    core = bridge.core
    placed_total = len(core.sm.placed) + sum(len(b) for _, b in core.closed)
    assert placed_total + len(core.inspection) == 11 and not core.has_work()
    assert actions["PLACE_CURRENT"] + actions["RETRIEVE_BUFFER"] == placed_total
    assert set(spawned) == {p.box_id for p in core.sm.placed}
    assert core.decider.policy.name == "lookahead" and core.decider.policy.choose.stats.searched > 0


def test_runtime_always_decides_with_the_lookahead_over_the_conveyor_preview():
    import json

    from pac_highlevel import HighLevelDecider
    from pac_runtime import RuntimeCore
    from pac_runtime.order import cell_from_order
    from pac_runtime.ros_node import CoreBridge

    assert HighLevelDecider(None).policy.name == "lookahead"
    order = {"pallet": {"size_m": [1.2, 1.0, 1.35]},
             "skus": {"K": {"size_m": [0.4, 0.3, 0.2], "weight_kg": [2.0, 6.0], "count": 4}}}
    hl = replace(HighLevelConfig(), buffer=replace(HighLevelConfig().buffer, slots=2))
    core = RuntimeCore(cell_from_order(order, CandidateConfig()), CandidateConfig(), hl, RuntimeConfig(),
                       RobotFeasibility(), load_policy(config=hl))
    bridge = CoreBridge(core)
    obs = lambda i, sku="K": {"box_id": i, "label_sku": sku, "weight_kg": 4.0, "size_m": [0.4, 0.3, 0.2]}  # noqa: E731
    # an unreadable label cannot be planned: left out of the window
    bridge.on_preview(json.dumps({"boxes": [obs("B"), obs("C", None), obs("D")]}))
    assert [b.box_id for b in core.preview] == ["B", "D"]
    assert bridge.status()["policy"] == "lookahead" and bridge.status()["visible_boxes"] == 2
    version = core.sm.version
    _, cmd = bridge.on_observation(json.dumps(obs("A")))
    assert cmd["action"] in ("PLACE_CURRENT", "BUFFER_CURRENT") and cmd["decided_by"] == "policy:lookahead"
    assert core.decider.policy.choose.stats.searched == 1  # window B, D searched
    assert core.sm.version == version + 1                  # the preview itself changes no state


def _k_bridge(count):
    from pac_runtime import RuntimeCore
    from pac_runtime.order import cell_from_order
    from pac_runtime.ros_node import CoreBridge

    order = {"pallet": {"size_m": [1.2, 1.0, 1.35]},
             "skus": {"K": {"size_m": [0.4, 0.3, 0.2], "weight_kg": [2.0, 6.0], "count": count}}}
    hl = replace(HighLevelConfig(), buffer=replace(HighLevelConfig().buffer, slots=2))
    return CoreBridge(RuntimeCore(cell_from_order(order, CandidateConfig()), CandidateConfig(), hl,
                                  RuntimeConfig(), RobotFeasibility(), load_policy("rule", config=hl)))


def _k_obs(box_id):
    import json

    return json.dumps({"box_id": box_id, "label_sku": "K", "weight_kg": 4.0, "size_m": [0.4, 0.3, 0.2]})


def _placed(cmd):
    import json

    return json.dumps({"state_version": cmd["state_version"], "ok": True, "measured_pose": cmd["target_min_corner"]})


def test_bridge_defers_an_observation_that_arrives_during_a_placement():
    """The next box reaching the pick point while the robot places the previous
    one must not make the outstanding result STALE (or crash the node)."""
    bridge = _k_bridge(3)
    _, cmd_a = bridge.on_observation(_k_obs("A"))
    assert cmd_a["box_id"] == "A"
    verdict, cmd = bridge.on_observation(_k_obs("B"))        # robot still busy with A
    assert verdict is None and cmd is None and bridge.status()["deferred"] == 1
    assert "B" not in bridge.core.sm.tracked
    level, cmd_b = bridge.on_result(_placed(cmd_a))          # accepted, then B applied and planned
    assert level == "L0" and cmd_b["action"] == "PLACE_CURRENT" and cmd_b["box_id"] == "B"
    assert bridge.status()["deferred"] == 0 and bridge.core.counts["stale_result"] == 0
    # the strict core check itself is unchanged
    assert bridge.on_result(_placed(cmd_b))[0] == "L0"


def test_bridge_defers_conveyor_idle_during_a_placement():
    bridge = _k_bridge(3)
    _, cmd = bridge.on_observation(_k_obs("A"))
    missing, nxt = bridge.on_idle('{"idle_s": 60}')
    assert missing == {} and nxt is None and not bridge.core.supervisor.missing
    bridge.on_result(_placed(cmd))
    assert dict(bridge.core.supervisor.missing) == {"K": 2}


def test_bridge_full_order_with_every_box_arriving_mid_placement():
    bridge = _k_bridge(10)
    _, cmd = bridge.on_observation(_k_obs("B000"))
    for i in range(1, 10):
        bridge.on_observation(_k_obs(f"B{i:03d}"))           # always during the previous command
        while cmd is not None and cmd["action"] != "PLACE_CURRENT":
            _, cmd = bridge.on_result(json_ok(cmd))
        _, cmd = bridge.on_result(_placed(cmd))
    while cmd is not None:
        _, cmd = bridge.on_result(_placed(cmd) if cmd.get("target_min_corner") else json_ok(cmd))
    status = bridge.status()
    assert status["counts"].get("stale_result", 0) == 0 and status["deferred"] == 0
    assert status["placed"] + sum(len(boxes) for _, boxes in bridge.core.closed) == 10


def json_ok(cmd):
    import json

    return json.dumps({"state_version": cmd["state_version"], "ok": True})


def _gazebo_run(plan_ahead, noise=0.0, rt=None, clock=None):
    import json

    from pac_robot_check import load_robot_check_config
    from pac_runtime import RuntimeCore
    from pac_runtime.gazebo_driver import GazeboCell, GazeboDriverCore, boxes_from_order
    from pac_runtime.order import cell_from_order
    from pac_runtime.ros_node import CoreBridge

    order = {"pallet": {"size_m": [1.2, 1.0, 1.35]},
             "skus": {"K": {"size_m": [0.4, 0.3, 0.2], "weight_kg": [2.0, 6.0], "count": 6},
                      "S": {"size_m": [0.25, 0.2, 0.15], "weight_kg": [2.0, 4.0], "count": 6}}}
    hl = replace(HighLevelConfig(), buffer=replace(HighLevelConfig().buffer, slots=2))
    robot = RobotFeasibility(load_robot_check_config(REPO / "config/taehyeon/robot_check_gazebo.yaml"))
    if rt is None:  # budgets large enough that the search never stops early: deterministic commands
        from pac_runtime.config import PlanningConfig

        rt = RuntimeConfig(planning=PlanningConfig(ahead_ratio=1.0, max_budget_s=600.0, wait_budget_s=600.0))
    now = [0.0]
    bridge = CoreBridge(RuntimeCore(cell_from_order(order, CandidateConfig()), CandidateConfig(), hl,
                                    rt, robot, load_policy("lookahead", config=hl)),
                        plan_ahead=plan_ahead, clock=(lambda: now[0]) if clock else __import__("time").monotonic)
    driver = GazeboDriverCore(boxes_from_order(order, seed=2), robot, GazeboCell(), visible_boxes=3,
                              place_noise_m=noise, seed=5)
    queue = [("preview", driver.preview()), ("obs", driver.first_observation())]
    commands = []
    while queue:
        kind, payload = queue.pop(0)
        if kind == "preview":
            bridge.on_preview(json.dumps(payload))
            continue
        call = {"obs": bridge.on_observation, "idle": bridge.on_idle, "res": bridge.on_result}[kind]
        _, cmd = call(json.dumps(payload))
        if cmd is None:
            continue
        commands.append((cmd["action"], cmd["box_id"], cmd.get("target_min_corner")))
        acts = driver.on_command(cmd)
        now[0] += acts.duration_s * (clock or 1.0)  # the robot motion (simulated clock)
        queue.append(("res", acts.result))
        if acts.preview is not None:
            queue.append(("preview", acts.preview))
        if acts.observation is not None:
            queue.append(("obs", acts.observation))
        if acts.idle is not None:
            queue.append(("idle", acts.idle))
    return commands, bridge


def test_planning_during_the_motion_gives_the_same_commands():
    """The command planned on the forecast while the robot moves is the one
    the cell would have planned after the result (exact execution)."""
    plain, _ = _gazebo_run(plan_ahead=False)
    ahead, bridge = _gazebo_run(plan_ahead=True)
    assert ahead == plain
    status = bridge.status()
    assert status["planned_ahead"] > 0.5 * len(ahead)
    placed = sum(1 for a, _, _ in ahead if a in ("PLACE_CURRENT", "RETRIEVE_BUFFER"))
    assert placed + len(bridge.core.inspection) == 12


def test_planning_ahead_falls_back_when_the_result_differs():
    """Placement error beyond the L0 band: the forecast is dropped and the
    next command is planned on the real (measured) state."""
    ahead, bridge = _gazebo_run(plan_ahead=True, noise=0.02)
    core = bridge.core
    assert core.counts["ahead_replanned"] > 0
    assert len(core.sm.placed) + sum(len(b) for _, b in core.closed) + len(core.inspection) == 12
    for action, box_id, corner in ahead:
        assert action != "WAIT"


def test_search_time_follows_the_robot_action_and_its_measured_duration():
    """Planned during a placement: ahead_ratio of its duration; during a buffer
    move: ahead_ratio of the travel; while the robot waits: wait_budget_s. Measured
    durations (here twice the driver's motion) correct the next budgets."""
    from pac_runtime.config import PlanningConfig

    rt = RuntimeConfig(planning=PlanningConfig(max_budget_s=600.0))  # no cap: see the correction
    _, bridge = _gazebo_run(plan_ahead=True, rt=rt, clock=2.0)
    core, pl = bridge.core, rt.planning
    by_action = {}
    for action, budget in bridge.budgets:
        by_action.setdefault(action, []).append(budget)
    assert by_action["WAIT"][0] == pl.wait_budget_s          # the first box: nothing to plan during
    place = by_action["PLACE_CURRENT"]
    assert all(pl.min_budget_s <= b <= pl.max_budget_s for b in place)
    first_place = next(b for a, b in bridge.budgets if a == "PLACE_CURRENT")
    assert place[-1] > first_place                           # learned: the robot is slower than expected
    scale = core.duration_scale["PLACE_CURRENT"]
    assert scale > 1.2
    if "BUFFER_CURRENT" in by_action:
        assert max(by_action["BUFFER_CURRENT"]) < max(place)  # a buffer move leaves less time
    status = bridge.status()
    assert status["last_budget_s"] is not None and "PLACE_CURRENT" in status["duration_scale"]
