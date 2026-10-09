"""Stage 6 (pac_robot_check): HDR50-22 kinematics and robot feasibility."""

from dataclasses import replace
import math

import numpy as np
import pytest

from th_helpers import REPO, make_box, make_state, placed

from pac_common import PlacementCandidate, Pose3D, RejectCode, Size3D
from pac_robot_check import HOME, RobotCheckConfig, RobotFeasibility, fk, joint_frames, load_robot_check_config
from pac_robot_check.kinematics import LOWER, UPPER, ik, ik_all

PALLET = Size3D(1.2, 1.0, 1.35)


def cand(box, x, y, z, yaw=0.0, version=1, cid="c"):
    return PlacementCandidate(cid, box.box_id, Pose3D("pallet", x, y, z, yaw=yaw), version)


def test_home_pose_matches_the_urdf():
    frames = joint_frames(HOME)
    # j2 at (0.32, 0, 0.48); home lifts the 0.87 m upper arm vertical
    assert np.allclose(frames[1][:3, 3], [0.32, 0.0, 0.48], atol=1e-3)
    assert np.allclose(frames[2][:3, 3], [0.32, 0.0, 1.35], atol=1e-3)
    # forearm horizontal: j5 1.03 m ahead of j4
    assert np.allclose(frames[4][:3, 3], [0.32 + 1.03, 0.0, 1.55], atol=1e-3)


def test_closed_form_ik_round_trip():
    rng = np.random.default_rng(3)
    hits = 0
    for _ in range(100):
        q = rng.uniform(LOWER, UPPER)
        q[3], q[5] = rng.uniform(-3, 3, 2)
        T = fk(q, 0.22)
        sols = ik_all(T, 0.22, ref=q)
        assert all(np.allclose(fk(s, 0.22), T, atol=1e-3) for s in sols)
        assert all(np.all(s >= LOWER) and np.all(s <= UPPER) for s in sols)
        hits += any(np.max(np.abs(s - q)) < 1e-3 for s in sols)
    assert hits >= 95
    far = np.eye(4)
    far[:3, 3] = (3.0, 0.0, 1.0)
    assert ik(far, 0.22, seeds=[HOME]) is None


def test_every_spot_of_the_three_pallets_is_reachable_with_the_default_cell():
    robot = RobotFeasibility()
    box = make_box("C", (0.4, 0.3, 0.25), weight=10)
    for pallet in (PALLET, Size3D(1.1, 1.1, 1.35), Size3D(1.2, 0.8, 1.35)):
        state = make_state([], version=1, pallet=pallet)
        for x in (0.0, pallet.x - 0.4):
            for y in (0.0, pallet.y - 0.3):
                for z in (0.0, 1.1):
                    v = robot.validate_robot_motion(box, cand(box, x, y, z), state)
                    assert v.success, (pallet, x, y, z, v.codes, v.details)
                    assert 3.0 < v.details["cycle_time_s"] < 20.0
                    q = np.asarray(v.details["q_place"])
                    T = robot.pallet_to_base(pallet)
                    tcp = fk(q, robot.cfg.gripper.tcp_offset_m)
                    top = np.linalg.inv(T) @ tcp
                    assert np.allclose(top[:3, 3], [x + 0.2, y + 0.15, z + 0.25], atol=2e-3)
                    assert top[2, 2] == pytest.approx(-1.0, abs=1e-3)  # gripper points down


def test_payload_stale_and_frame_codes():
    robot = RobotFeasibility()
    state = make_state([], version=4, pallet=PALLET)
    heavy = make_box("H", (0.4, 0.3, 0.25), weight=40)  # 40 + 15 kg gripper > 50
    v = robot.validate_robot_motion(heavy, cand(heavy, 0.3, 0.3, 0.0, version=4), state)
    assert v.codes == (RejectCode.PAYLOAD_EXCEEDED,) and v.details["payload_kg"] == pytest.approx(55.0)
    light = make_box("L", (0.4, 0.3, 0.25), weight=5)
    assert robot.validate_robot_motion(light, cand(light, 0.3, 0.3, 0.0, version=3), state).codes == (RejectCode.STALE_PLAN,)


def test_gripper_hits_a_taller_neighbour_during_descent():
    robot = RobotFeasibility()
    # 0.2 m wide box between two 0.5 m tall columns 0.2 m apart: the box
    # fits (5-2 passes) but the 0.34 x 0.26 gripper does not
    walls = [placed("W1", 0.2, 0.3, 0.0, size=(0.2, 0.4, 0.5)), placed("W2", 0.6, 0.3, 0.0, size=(0.2, 0.4, 0.5))]
    state = make_state(walls, version=1, pallet=PALLET)
    box = make_box("S", (0.2, 0.2, 0.2), weight=3)
    v = robot.validate_robot_motion(box, cand(box, 0.4, 0.4, 0.0), state)
    assert not v.success and RejectCode.ROBOT_COLLISION in v.codes
    assert {a["gripper_hits"] for a in v.details["attempts"]} <= {"W1", "W2"}
    # same box beside a single column, gripper side free -> passes after rotating
    v = robot.validate_robot_motion(box, cand(box, 0.9, 0.4, 0.0), make_state(walls[1:], version=1, pallet=PALLET))
    assert v.success, v.details


def test_unreachable_cell_and_first_executable():
    cfg = RobotCheckConfig()
    far = RobotFeasibility(replace(cfg, cell=replace(cfg.cell, base_from_pallet_center=(-1.35, 1.0, -0.15, 0.0))))
    state = make_state([], version=1, pallet=PALLET)
    box = make_box("C", (0.4, 0.3, 0.25), weight=10)
    v = far.validate_robot_motion(box, cand(box, 0.8, 0.0, 0.0), state)
    assert not v.success and set(v.codes) <= {RejectCode.IK_FAIL, RejectCode.APPROACH_FAIL}
    ranked = [cand(box, 0.8, 0.0, 0.0, cid="far"), cand(box, 0.0, 0.7, 0.0, cid="near")]
    chosen, verdict, rejected = far.first_executable(box, ranked, state)
    assert chosen.candidate_id == "near" and verdict.success and set(rejected) == {"far"}
    times = far.robot_time_sec_by_candidate(box, ranked, state)
    assert set(times) == {"near"}


def test_arm_collides_with_a_tall_stack_on_the_robot_side():
    # placing low behind a 1.3 m wall between the robot and the spot: every
    # IK branch drives the forearm through the wall -> not executable;
    # a 0.6 m wall leaves room for the arm
    robot = RobotFeasibility()
    box = make_box("C", (0.4, 0.3, 0.25), weight=10)
    target = cand(box, 0.4, 0.65, 0.0)
    tall = make_state([placed("T", 0.0, 0.0, 0.0, size=(1.2, 0.2, 1.3))], version=1, pallet=PALLET)
    v = robot.validate_robot_motion(box, target, tall)
    assert v.codes == (RejectCode.ROBOT_COLLISION,)
    assert all(a.get("arm_hits") == "T" for a in v.details["attempts"])
    low = make_state([placed("T", 0.0, 0.0, 0.0, size=(1.2, 0.3, 0.6))], version=1, pallet=PALLET)
    v = robot.validate_robot_motion(box, target, low)
    assert v.success and v.details["arm_clearance_m"] > 0


def test_yaml_config_round_trip():
    cfg = load_robot_check_config(REPO / "config/taehyeon/robot_check.yaml")
    assert cfg == RobotCheckConfig()
    with pytest.raises(ValueError):
        from pac_robot_check import config_from_dict
        config_from_dict({"robot_check": {"robot_model": "ur20"}})


def test_held_box_sweep_is_checked_outside_the_gripper_footprint():
    robot = RobotFeasibility()
    big = make_box("B", (0.8, 0.6, 0.2), weight=10)
    # an overhang above the target column, outside the 0.34 x 0.26 gripper
    over = [placed("S", 0.65, 0.0, 0.0, size=(0.4, 0.3, 0.4)), placed("O", 0.65, 0.25, 0.4, size=(0.4, 0.3, 0.2))]
    v = robot.validate_robot_motion(big, cand(big, 0.0, 0.3, 0.0), make_state(over, version=1, pallet=PALLET))
    assert not v.success and v.details["box_hits"] == "O"
