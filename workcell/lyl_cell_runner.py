#!/usr/bin/env python3
"""PAC2026 LYL algorithm on the Gazebo V4.6 cell, one order from the generator to the last box.

    python3 workcell/lyl_cell_runner.py --dataset DATASET --scenario S0003 [--count 12]

Needs Gazebo V4.6 (hdr50_workcell_v4_6.launch.py) and MoveIt (hdr50_moveit_v44.launch.py)
running; ``workcell/run_gazebo_demo.sh`` starts everything (no PyBullet viewer).

  conveyor   boxes enter every ``pitch_s`` of running time while fewer than ``max_queue`` are
             between scale and stopper; the line stops to weigh a box and while a box waits at
             the stopper (one roller drive, so gaps are kept); scale -> stopper ``scale_to_stopper_s``
  stage 1    scale weight + Donghan ``measure_box`` on the rgbd scale camera's depth image
  2 - 6      PAC2026 LYL ``RuntimeCore`` (integration profile + config/workcell_v46.yaml)
  7          V4.6 MoveIt executor (scripts/run_moveit_pick_place_v44.sh) with its own recovery
             (pick retries, re-place, re-plan); NG boxes to the NG zone (Donghan Hold/NG grid order)
  8          placed pose measured by the cell CCTV -> ``RuntimeCore.on_result``
  dashboard  logs/lyl_cell/<run>/: cycle log, boxes.json, events.jsonl; active run registered for
             the V5.3 dashboard (live Gazebo poses over the full V4.6 cell)
"""
from __future__ import annotations

import argparse
from collections import deque
from dataclasses import replace
import json
import math
import os
from pathlib import Path
import random
import re
import subprocess
import sys
import threading
import time

LYL = Path(__file__).resolve().parents[1]
WORKCELL = Path(os.environ.get("PAC_WORKCELL_REPO", LYL.parent / "pac2026_integrated")).resolve()
sys.path[:0] = [str(p) for p in sorted((LYL / "ros2_ws/src").glob("pac_*")) if (p / p.name).is_dir()]
sys.path[:0] = [str(LYL), str(LYL / "tools/virtual_data"), str(LYL / "tools/pac_failure_analysis_dashboard")]

from workcell.process_stream import run_logged

import numpy as np  # noqa: E402
import yaml  # noqa: E402

from pac_candidates import load_candidate_config  # noqa: E402
from pac_candidates.geometry import rotated_dims  # noqa: E402
from pac_common import Pose3D  # noqa: E402
from pac_highlevel import load_lookahead_config, load_policy  # noqa: E402
from pac_highlevel.config import config_from_dict as highlevel_from_dict  # noqa: E402
from pac_perception.depth_measurement import CameraModel, measure_box  # noqa: E402
from pac_perception.perception_config import load_perception_config  # noqa: E402
from pac_perception.raw_observation import build_raw_observation  # noqa: E402
from pac_robot_check import RobotFeasibility, load_robot_check_config  # noqa: E402
from pac_runtime import RuntimeCore, load_runtime_config  # noqa: E402
from pac_runtime.core import ExecutionReport  # noqa: E402
from pac_runtime.order import configure_cell  # noqa: E402
from pac_runtime.perception import RawObservation  # noqa: E402
from pac_runtime.ros_node import make_runtime_ranker  # noqa: E402
from virtual_data import load_virtual_config  # noqa: E402
from virtual_data.runtime_cell import runtime_cell  # noqa: E402
from virtual_data.scenario_source import load_dataset  # noqa: E402

PROFILE = LYL / "config/jaesung_taehyeon"
G = 9.80665


def deep_merge(base, over):
    out = dict(base)
    for k, v in over.items():
        out[k] = deep_merge(out.get(k, {}), v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


class Run:
    """Run folder: cycle log, events (dashboard contract), summary."""

    def __init__(self, run_id, world):
        self.id, self.world = run_id, world
        self.dir = WORKCELL / "logs/lyl_cell" / run_id
        self.dir.mkdir(parents=True, exist_ok=True)
        self.log_f = (self.dir / "lyl_cell_cycle.log").open("a", encoding="utf-8")
        self.lock = threading.Lock()

    def log(self, msg):
        line = time.strftime("%H:%M:%S ") + msg
        with self.lock:
            print(line, flush=True)
            self.log_f.write(line + "\n")
            self.log_f.flush()

    def event(self, box, event, success=True, reason=""):
        rec = {"run_id": self.id, "world": self.world, "box_id": box, "event": event, "success": success,
               "timestamp_epoch_s": round(time.time(), 3), "reason": str(reason)[:200]}
        with self.lock, (self.dir / "events.jsonl").open("a", encoding="utf-8") as f:
            f.write(json.dumps(rec, ensure_ascii=False, separators=(",", ":")) + "\n")


class ConveyorBox:
    def __init__(self, k, field):
        self.k, self.field = k, field
        self.name = f"box_{k:02d}"
        self.weighed = False
        self.obs = None
        self.measure = None
        self.picked = False
        self.at_stopper_logged = False
        self.last_x = None
        self.still_since = None


class Cell:
    """Gazebo I/O over ROS 2 (bridged topics) and the ign CLI."""

    def __init__(self, cfg, run):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from geometry_msgs.msg import WrenchStamped
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import CameraInfo, Image
        from std_msgs.msg import Float64
        from tf2_msgs.msg import TFMessage

        self.cfg, self.run = cfg, run
        g = cfg["geometry"]
        self.world = g["world"]
        rclpy.init()
        self.node = Node("lyl_cell_runner")
        self.pub = self.node.create_publisher(Float64, "/pac/conveyor/roller_cmd_vel", 10)
        self.Float64 = Float64
        self.poses, self.fz, self.depth, self.info, self.clock = {}, deque(maxlen=200), None, None, 0.0
        self.lock = threading.Lock()
        n = self.node
        n.create_subscription(TFMessage, f"/world/{self.world}/dynamic_pose/info", self._on_tf, qos_profile_sensor_data)
        n.create_subscription(WrenchStamped, "/pac/scale/wrench", self._on_wrench, qos_profile_sensor_data)
        n.create_subscription(Image, "/pac/scale_camera/depth_image", self._on_depth, qos_profile_sensor_data)
        n.create_subscription(CameraInfo, "/pac/scale_camera/camera_info", self._on_info, qos_profile_sensor_data)
        n.create_subscription(Clock, "/clock", self._on_clock, qos_profile_sensor_data)
        self.rclpy = rclpy

    # -- subscriptions ----------------------------------------------------------------------
    def _on_tf(self, msg):
        with self.lock:
            for t in msg.transforms:
                name = t.child_frame_id.split("::")[-1]
                if name.startswith("box_"):
                    p, q = t.transform.translation, t.transform.rotation
                    self.poses[name] = (p.x, p.y, p.z, q.x, q.y, q.z, q.w, time.time())

    def _on_wrench(self, msg):
        self.fz.append((self.clock, msg.wrench.force.z))

    def _on_depth(self, msg):
        self.depth = (self.clock, msg)

    def _on_info(self, msg):
        self.info = msg

    def _on_clock(self, msg):
        self.clock = msg.clock.sec + msg.clock.nanosec * 1e-9

    def spin(self, seconds=0.0):
        end = time.time() + seconds
        while True:
            if not self.rclpy.ok():
                return
            self.rclpy.spin_once(self.node, timeout_sec=0.02)
            if time.time() >= end:
                return

    def roller(self, omega):
        m = self.Float64()
        m.data = float(omega)
        self.pub.publish(m)

    def pose(self, name):
        with self.lock:
            return self.poses.get(name)

    def depth_image(self, after_clock):
        if self.depth is None or self.depth[0] < after_clock:
            return None
        msg = self.depth[1]
        return np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width)

    def force_mean(self, since):
        vals = [f for t, f in list(self.fz) if t >= since]
        return (float(np.mean(vals)), len(vals)) if vals else (None, 0)

    # -- ign CLI ---------------------------------------------------------------------------
    def spawn(self, box, size, mass, dent):
        sdf = self.run.dir / "boxes" / f"{box}.sdf"
        sdf.parent.mkdir(exist_ok=True)
        z = subprocess.run([sys.executable, str(WORKCELL / "scripts/make_box_sdf_v44.py"), "--size", *map(str, size),
                            "--mass", str(mass), "--out", str(sdf), "--dent", str(dent)],
                           capture_output=True, text=True, check=True).stdout.strip()
        create = subprocess.run(["ros2", "pkg", "prefix", "ros_gz_sim"], capture_output=True,
                                text=True, check=True).stdout.strip()
        g = self.cfg["geometry"]
        for _ in range(2):
            r = subprocess.run([f"{create}/lib/ros_gz_sim/create", "-name", box, "-file", str(sdf),
                                "-x", str(g["inlet_x"]), "-y", str(g["conveyor_y"]), "-z", z, "-Y", "0"],
                               capture_output=True, text=True, timeout=25)
            if r.returncode == 0:
                break
            time.sleep(1.0)
        else:
            detail = (r.stderr or r.stdout or "no output").strip()[-400:]
            raise RuntimeError(f"Gazebo spawn failed for {box} (rc {r.returncode}): {detail}")
        for _ in range(3):   # release the gripper's detachable joint for this box (V4.6 cycle)
            subprocess.run(["ign", "topic", "-t", f"/pac/gripper/{box}/detach", "-m", "ignition.msgs.Empty", "-p", " "],
                           capture_output=True, timeout=5)


class Runner:
    def __init__(self, args):
        self.args = args
        base = yaml.safe_load((LYL / "config/workcell_v46.yaml").read_text())
        self.cfg = base
        g, c = base["geometry"], base["conveyor"]
        self.g, self.c = g, c
        self.v = (g["stopper_face_x"] - g["scale_centre_x"]) / c["scale_to_stopper_s"]
        self.omega = self.v / c["roller_radius_m"]
        self.pitch_m = self.v * c["pitch_s"]
        ds = load_dataset(args.dataset)
        index, spec = next((i, s) for i, s in enumerate(ds.scenarios) if s.scenario_id == args.scenario)
        self.cc = load_candidate_config(PROFILE / "candidates.yaml")
        hl_raw = yaml.safe_load((PROFILE / "highlevel.yaml").read_text())
        hl_raw = {"highlevel": deep_merge(hl_raw.get("highlevel", hl_raw), base["algorithm_overrides"]["highlevel"])}
        self.hl = highlevel_from_dict(hl_raw)
        la_raw = yaml.safe_load((PROFILE / "lookahead.yaml").read_text())
        la_raw["lookahead"] = deep_merge(la_raw["lookahead"], base["algorithm_overrides"].get("lookahead", {}))
        from pac_highlevel.lookahead import lookahead_config_from_dict
        self.la = lookahead_config_from_dict(la_raw["lookahead"])
        self.rt = load_runtime_config(PROFILE / "runtime.yaml")
        vc = load_virtual_config(LYL / "config/taehyeon/virtual_data.yaml")
        cell = configure_cell(runtime_cell(spec, ds, self.cc, vc, scenario_index=index, family_index=0, seed=0,
                                           spec_mismatch_probability=0.02, missing_probability=0.02,
                                           pallet_xy=(1.1, 1.1)), self.rt)
        self.cellspec = cell
        stream = list(cell.stream)[: args.count] if args.count else list(cell.stream)
        self.boxes = [ConveyorBox(k, f) for k, f in enumerate(stream, 1)]
        self.by_id = {b.field.truth.box_id: b for b in self.boxes}
        self.core = RuntimeCore(cell, self.cc, self.hl, self.rt,
                                RobotFeasibility(load_robot_check_config(PROFILE / "robot_check.yaml")),
                                load_policy("lookahead", self.la, config=self.hl), make_runtime_ranker("layer"))
        p = base["perception"]
        dcfg, self.rcfg = load_perception_config(LYL / p["config"])
        self.dcfg = replace(dcfg, roi_xy_m=tuple(p["roi_xy_m"]), min_box_height_m=p["min_box_height_m"])
        run_id = f"{args.scenario}_{time.strftime('%Y%m%d_%H%M%S')}"
        self.run = Run(run_id, g["world"])
        self.cell = Cell(base, self.run)
        self.state_file = self.run.dir / "pallet_1_state.json"
        self.pallet_no, self.placed_now = 1, []
        self.buffer = {}                    # slot -> {"box", "size", "pose"}
        self.ng = []                        # NG boxes: (x, y, z_top, sx, sy)
        self.robot_busy = False
        self.stop = False
        self.t0 = None
        self.decisions = []
        self.write_catalog()
        self.register_dashboard()

    # ------------------------------------------------------------------ dashboard contract
    def write_catalog(self):
        cat = {b.name: {"size_m": [b.field.truth.size.x, b.field.truth.size.y, b.field.truth.size.z]} for b in self.boxes}
        (self.run.dir / "boxes.json").write_text(json.dumps(cat))

    def register_dashboard(self):
        try:
            from pac_run_contract_writer import register_active_run
            rel = self.run.dir.relative_to(WORKCELL)
            w = self.g["world"]
            register_active_run(
                WORKCELL, run_id=self.run.id, world=w, world_sdf=self.g["world_sdf"], log_dir=str(rel),
                box_catalog=str(rel / "boxes.json"), events_file=str(rel / "events.jsonl"),
                topics={"joint_states": "/joint_states", "scale_camera": "/pac/scale_camera/image",
                        "cctv_camera": "/pac/top_camera/image", "gazebo_pose": f"/world/{w}/dynamic_pose/info"},
                zones={"pallet": {"x": [-0.55, 0.55], "y": [0.65, 1.75], "z": [0.0, 2.2]},
                       "buffer": {"x": [-2.40, -0.60], "y": [0.11, 0.77], "z": [0.5, 2.0]},
                       "ng": {"x": [-1.45, -0.65], "y": [-0.65, 0.05], "z": [0.0, 1.5]},
                       "scale": {"x": [-6.335, -5.635], "y": [0.85, 1.55], "z": [0.8, 2.0]},
                       "conveyor": {"x": [-5.635, -0.80], "y": [0.85, 1.55], "z": [0.8, 2.0]}})
            self.run.log(f"[dashboard] active run registered: {self.run.id}")
        except Exception as error:  # noqa: BLE001 - the cell runs without the dashboard
            self.run.log(f"[dashboard] WARN could not register the active run: {error}")

    # ------------------------------------------------------------------ frames
    def pallet_to_world_centre(self, box_size, pose):
        """Core pallet frame (AABB min corner, origin at the near corner, world axes) -> world centre."""
        dx, dy, dz = rotated_dims(box_size, pose.yaw)
        ox, oy = self.g["pallet_near_corner"]
        return ox + pose.x + dx / 2, oy + pose.y + dy / 2, self.g["pallet_deck_z"] + pose.z + dz / 2

    def world_to_pallet_corner(self, box_size, x, y, z, yaw):
        q = round(yaw / (math.pi / 2))
        yaw_snap = (q % 2) * (math.pi / 2)
        dx, dy, dz = rotated_dims(box_size, yaw_snap)
        ox, oy = self.g["pallet_near_corner"]
        return Pose3D("pallet", x - ox - dx / 2, y - oy - dy / 2, max(0.0, z - self.g["pallet_deck_z"] - dz / 2), yaw=yaw_snap)

    # ------------------------------------------------------------------ conveyor thread
    def conveyor_worker(self):
        try:
            self.conveyor_loop()
        except Exception as error:  # noqa: BLE001 - propagate daemon failure to the main loop
            self.fail(None, f"conveyor failed: {error}")
            try:
                self.cell.roller(0.0)
            except Exception as stop_error:  # noqa: BLE001
                self.run.log(f"[conveyor] ERROR stop command failed: {stop_error}")

    def conveyor_loop(self):
        g, c, cell = self.g, self.c, self.cell
        sx, stop_face = g["scale_centre_x"], g["stopper_face_x"]
        next_k, last_spawn, weighing, tare = 0, None, None, None
        spawner = None
        cell.spin(1.0)
        tare_from = cell.clock
        cell.spin(1.5)
        tare, n = cell.force_mean(tare_from)
        if tare is None or n <= 0 or not math.isfinite(tare):
            raise RuntimeError("No valid scale samples for tare; check /pac/scale/wrench bridge")
        if cell.clock <= tare_from:
            raise RuntimeError("Simulation clock is not advancing; check /clock and Gazebo")
        self.run.log(f"[conveyor] tare {tare:.1f} N ({n} samples), speed {self.v:.3f} m/s ({self.omega:.2f} rad/s), "
                     f"pitch {self.pitch_m:.2f} m, max queue {c['max_queue']}")
        while not self.stop:
            cell.spin(0.05)
            now = cell.clock
            hold = False
            queue = []
            for b in self.boxes[:next_k]:
                p = cell.pose(b.name)
                if b.picked or p is None:
                    continue
                x, y, z = p[:3]
                if z > g["roller_top_z"] + b.field.truth.size.z + 0.08 or abs(y - g["conveyor_y"]) > 0.45:
                    b.picked = True          # lifted by the robot
                    continue
                queue.append((b, x))
                L = max(b.field.truth.size.x, b.field.truth.size.y)
                if not b.weighed and x >= sx - 0.06:
                    hold = True
                    if weighing is None or weighing[0] is not b:
                        weighing = (b, now)
                    elif now - weighing[1] >= c["settle_s"] + c["weigh_window_s"]:
                        self.weigh(b, tare, weighing[1] + c["settle_s"], x - sx, p)
                        weighing = None
                still = b.last_x is not None and abs(x - b.last_x) < 0.002
                b.still_since = (b.still_since or now) if still else None
                b.last_x = x
                if x >= stop_face - L / 2 - 0.035 or (x >= stop_face - L / 2 - 0.35 and b.weighed
                                                       and b.still_since is not None and now - b.still_since >= 1.0
                                                       and not hold):
                    hold = True
                    if not b.at_stopper_logged:
                        b.at_stopper_logged = True
                        self.run.event(b.name, "at_pick")
            cell.roller(0.0 if hold else self.omega)
            between = sum(1 for _, x in queue if x >= sx - 0.40)
            inlet_clear = last_spawn is None or (cell.pose(last_spawn) is not None
                                                 and cell.pose(last_spawn)[0] - g["inlet_x"] >= self.pitch_m)
            if (next_k < len(self.boxes) and not hold and between < c["max_queue"] and inlet_clear
                    and (spawner is None or not spawner.is_alive())):
                b = self.boxes[next_k]
                next_k += 1
                last_spawn = b.name
                t = b.field.truth
                dent = self.cfg["perception"]["dent_m"] if b.field.damaged else 0.0
                spawner = threading.Thread(target=self.spawn, args=(b, dent), daemon=True)
                spawner.start()
        cell.roller(0.0)

    def spawn(self, b, dent):
        t = b.field.truth
        try:
            self.cell.spawn(b.name, (t.size.x, t.size.y, t.size.z), t.weight_kg, dent)
        except Exception as error:  # noqa: BLE001 - never report a failed spawn as on_conveyor
            self.fail(b, f"spawn failed: {error}")
            return
        self.run.event(b.name, "on_conveyor", reason=f"{t.box_id} {t.sku_id}" + (" damaged" if dent else ""))
        self.run.log(f"[conveyor] {b.name} = {t.box_id} {t.sku_id} {t.size.x:.2f}x{t.size.y:.2f}x{t.size.z:.2f} m "
                     f"{t.weight_kg:.1f} kg spawned" + (f" (damaged: {dent * 1000:.0f} mm dent)" if dent else ""))

    def weigh(self, b, tare, since, x_rel, pose):
        cell, t = self.cell, b.field.truth
        f, n = cell.force_mean(since)
        mass = (f - tare) / G if f is not None else float("nan")
        img = cell.depth_image(since)
        k = cell.info.k if cell.info is not None else None
        g = self.g
        obs, m = None, None
        if img is not None and k is not None:
            T = np.eye(4)
            T[:3, :3] = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]]).T   # optical axes in world (straight down)
            cam = g["scale_camera"]
            T[:3, 3] = [cam["x"] - g["scale_centre_x"], cam["y"] - g["conveyor_y"], cam["z"] - g["roller_top_z"]]
            try:
                m = measure_box(img, CameraModel(k[0], k[4], k[2], k[5], T), self.dcfg, target_x=x_rel)
                obs = build_raw_observation(t.box_id, mass, m, cell.clock, label_sku=t.sku_id,
                                            catalog=self.cellspec.catalog, weight_ranges=self.cellspec.weight_ranges,
                                            resolver_config=self.rcfg)
            except ValueError as error:
                self.run.log(f"[PERCEPTION] FAIL {b.name}: {error}")
        if obs is None:
            obs = RawObservation(t.box_id, None, round(mass, 3) if math.isfinite(mass) else 0.0,
                                 t.size, 0.0, False, "top", cell.clock)
        b.obs, b.measure, b.weighed = obs, m, True
        self.run.log(f"[PERCEPTION] {b.name}: {mass:.2f} kg (true {t.weight_kg:.2f}), depth "
                     + (f"{m.size.x:.3f}x{m.size.y:.3f}x{m.size.z:.3f} m (true {t.size.x:.3f}x{t.size.y:.3f}x{t.size.z:.3f})"
                        f", damage {list(m.damage_reasons) or '-'}, conf {m.confidence}" if m else "no measurement"))

    # ------------------------------------------------------------------ robot
    def moveit(self, b, tag, extra):
        t = b.field.truth
        log = self.run.dir / f"{b.name}_moveit_{tag}.log"
        result = self.run.dir / f"{b.name}_placed_{tag}.json"
        obstacles = self.run.dir / "buffer_obstacles.json"
        obstacles.write_text(json.dumps({v["box"]: v["size"] for v in self.buffer.values()}))
        state = {"placed": [{"box_id": n, "sku_id": s, "physical_size": dict(zip("xyz", sz))} for n, s, sz in self.placed_now]}
        self.state_file.write_text(json.dumps(state))
        env = dict(os.environ, PERCEPTION_EXTERNAL="1", PAC_LAYOUT="v46", PAC_SCALE_SHIFT_M="-2.185", PAC_NO_WRIST="1",
                   PAC_BUFFER_EXT_DX="0.90",
                   PAC_BUFFER_X="-1.05", PAC_BUFFER_Y="0.44", PAC_BUFFER_DEPTH="0.66", IGN_IP=os.environ.get("IGN_IP", "127.0.0.1"),
                   PAC_FREE_SPEED=str(self.cfg["robot"]["free_speed"]), PAC_CART_SPEED=str(self.cfg["robot"]["cart_speed"]),
                   PAC_PRESS_M=str(self.cfg["robot"].get("press_depth_m", 0.001)))
        # The old cycle runner's next-box gate has no producer in this integration.
        env.pop("SPAWN_GATE", None)
        cmd = ["bash", str(LYL / "workcell/run_pick_place_v46.sh"), "--box", b.name,
               "--size", *map(str, (t.size.x, t.size.y, t.size.z)), "--mass", f"{b.obs.weight_kg:.3f}",
               "--gripper", self.cfg["robot"]["gripper"], "--pose-source", "camera", "--placed-state", str(self.state_file),
               "--catalog", str(LYL / "tools/ahead_dataset_generator/config/default.yaml"),
               "--extra-placed-json", str(obstacles), "--result-json", str(result), *extra]
        started = time.time()
        self.run.log(f"[executor] START {b.name} {tag}; 목적지 인자(박스 중심, world m): {' '.join(extra)}; log {log}")
        phase = {"last": "MoveIt 실행기 준비", "since": time.monotonic()}

        def progress(line):
            if re.search(r">>>|\[vertical\]|\[alignment\]|\[pickup\]|\[suction\]|\[camera\]|FAIL|STOP|recover", line):
                phase.update(last=line.strip()[:220], since=time.monotonic())
                self.run.log("    " + line.strip())

        def waiting(elapsed):
            self.run.log(f"[executor] 진행 대기 {elapsed:.0f}s; 마지막 출력 후 {time.monotonic()-phase['since']:.0f}s: "
                         f"{phase['last']} (실제 동작 성공을 뜻하지 않음)")

        rc = run_logged(cmd, log_path=log, env=env, cwd=WORKCELL, on_line=progress, on_wait=waiting)
        text = log.read_text(errors="replace")
        ok = rc == 0 and "MOVEIT PICK&PLACE PASS" in text
        self.run.log(f"[executor] {'PASS' if ok else 'FAIL'} {b.name} {tag} ({time.time() - started:.0f} s, rc {rc})")
        measured = json.loads(result.read_text()) if result.exists() else None
        return ok, measured, text

    def ng_slot(self, size):
        """Donghan Hold/NG order: 3 columns x 2 rows over the zone, then the next layer."""
        (x0, x1), (y0, y1) = self.g["ng_zone"]
        px, py = (x1 - x0) / 3, (y1 - y0) / 2
        sx, sy = sorted(size[:2], reverse=True)
        for k in range(12):
            col, row, lay = k % 3, (k // 3) % 2, k // 6
            cx, cy = x0 + (col + 0.5) * px, y0 + (row + 0.5) * py
            under = [n for n in self.ng if abs(n[0] - cx) < (n[3] + sx) / 2 and abs(n[1] - cy) < (n[4] + sy) / 2]
            z = max([0.0] + [n[2] for n in under])
            if (lay == 0 and not under) or (lay > 0 and under):
                return cx, cy, z + size[2] / 2 + 0.005, sx, sy
        return (x0 + x1) / 2, (y0 + y1) / 2, max([0.0] + [n[2] for n in self.ng]) + size[2] / 2 + 0.005, sx, sy

    def to_ng(self, b, reason):
        t = b.field.truth
        x, y, z, sx, sy = self.ng_slot((t.size.x, t.size.y, t.size.z))
        yaw = 0.0 if t.size.x >= t.size.y else math.pi / 2
        self.run.log(f"[NG] {b.name} -> NG zone ({x:.2f}, {y:.2f}) reason {reason}")
        ok, _, _ = self.moveit(b, "ng", ["--slot", f"{x:.4f}", f"{y:.4f}", f"{z:.4f}", "--slot-yaw", f"{yaw:.4f}",
                                         "--place-on", "ng"])
        if ok:
            self.ng.append((x, y, z + t.size.z / 2, sx, sy))
            self.run.event(b.name, "ng_diverted", reason=reason)
        return ok

    def decision_loop(self):
        core, sm = self.core, self.core.sm
        prev = None
        while not self.stop:
            self.cell_time()
            if sm.current_id() is None and core.supervisor.can_pick():
                b = next((x for x in self.boxes if x.weighed and x.at_stopper_logged and not x.picked
                          and x.field.truth.box_id not in sm.tracked
                          and not any(p.box_id == x.field.truth.box_id for p in sm.placed)
                          and not getattr(x, "done", False)), None)
                if b is not None:
                    verdict = core.on_observation(b.obs)
                    if verdict is None:
                        b.done = True
                        continue
                    if verdict.route == "INSPECTION":
                        self.run.log(f"[validator] WARN {b.name} -> inspection ({verdict.kind.value})")
                        if not self.to_ng(b, verdict.kind.value):
                            return self.fail(b, "NG move failed")
                        b.done = True
                        continue
                    self.run.log(f"[validator] {b.name} {verdict.box.sku_id} {verdict.kind.value}")
                elif self.more_coming() and not core.has_work():
                    time.sleep(0.2)
                    continue
                elif self.more_coming() and sm.current_id() is None:
                    time.sleep(0.2)          # next box is on its way: wait for it (the virtual loop does)
                    continue
            if not self.more_coming() and not getattr(self, "missing_done", False):
                self.missing_done = True     # stream over: boxes of the order list that never came
                out = core.on_conveyor_idle(self.rt.supervisor.missing_timeout_s)
                if out:
                    self.run.log(f"[cycle] MISSING {dict(out)}")
            if not core.has_work():
                if not self.more_coming():
                    break
                time.sleep(0.2)
                continue
            core.on_preview([x.obs for x in self.boxes if x.weighed and not x.at_stopper_logged][: self.hl.close.visible_boxes])
            budget = core.ahead_budget(prev) if prev is not None else self.rt.planning.wait_budget_s
            started = time.perf_counter()
            cmd = core.next_command(budget_s=budget)
            elapsed = time.perf_counter() - started
            self.decisions.append({"s": round(elapsed, 3), "action": cmd.action, "box": cmd.box_id, "reason": cmd.reason})
            limit = self.rt.planning.decision_budget_s
            self.run.log(f"[planner] {cmd.action} {cmd.box_id or ''} ({cmd.reason}) decided in {elapsed:.2f} s "
                         f"(budget {budget:.1f} s)" + (f" WARN over {limit:.0f} s, best command kept" if elapsed > limit else ""))
            prev = cmd if cmd.action != "WAIT" else None
            started = time.time()
            if not self.execute(cmd):
                return
            if prev is not None:
                # measured duration: the next search gets 70 % of it (at most 8 s)
                core.observe_duration(prev, time.time() - started)
        if self.stop:
            return
        self.run.log(f"[pipeline] PASS order finished: placed {sum(len(b) for _, b in core.closed) + len(sm.placed)}, "
                     f"inspection {len(core.inspection)}, pallets {self.pallet_no}")

    def more_coming(self):
        return any(not getattr(b, "done", False) and not b.picked for b in self.boxes)

    def cell_time(self):
        if self.t0 is None and self.cell.clock > 0:
            self.t0 = self.cell.clock
        self.core.sm.t = max(self.core.sm.t, self.cell.clock - (self.t0 or 0.0))

    def fail(self, b, why):
        self.run.log(f"[executor] ERROR cell stopped at {b.name if b else '-'}: {why} "
                     "(operator check required; resolve the reported failure before restarting)")
        if b is not None:
            self.run.event(b.name, "failed", success=False, reason=why)
        self.stop = True

    def box_of(self, box_id):
        return self.by_id[box_id]

    def execute(self, cmd):
        core, a = self.core, cmd.action
        if a == "WAIT":
            time.sleep(0.5)
            return True
        b = self.box_of(cmd.box_id) if cmd.box_id else None
        if a == "BUFFER_CURRENT":
            bx, by = self.g["buffer_bays"][cmd.slot]
            t = b.field.truth
            z = self.g["buffer_shelf_top_z"] + t.size.z / 2
            yaw = math.pi / 2 if t.size.x > t.size.y else 0.0     # long side along the shelf depth
            ok, measured, _ = self.moveit(b, "buffer", ["--slot", f"{bx:.4f}", f"{by:.4f}", f"{z:.4f}",
                                                        "--slot-yaw", f"{yaw:.4f}", "--place-on", "buffer"])
            if not ok:
                return self.fail(b, "buffer put failed") or False
            pose = [measured["x"], measured["y"], measured["z"], measured["yaw"]] if measured else [bx, by, z, yaw]
            self.buffer[cmd.slot] = {"box": b.name, "size": [t.size.x, t.size.y, t.size.z], "pose": pose}
            b.done = True
            core.on_result(cmd)
            self.run.event(b.name, "buffer_stored", reason=f"bay {cmd.slot + 1}")
            return True
        if a == "REJECT_NG":
            ok = self.to_ng(b, cmd.reason or "REJECT_NG")
            if not ok:
                return self.fail(b, "NG move failed") or False
            b.done = True
            core.on_result(cmd)
            return True
        if a == "PALLET_CLOSE":
            self.run.log(f"[cycle] CLOSE pallet {self.pallet_no} ({cmd.reason}, {len(self.placed_now)} boxes) -> pallet change")
            self.state_file.write_text(json.dumps({"placed": [{"box_id": n} for n, _, _ in self.placed_now]}))
            r = subprocess.run([sys.executable, str(WORKCELL / "scripts/pallet_swap_v44.py"), "--state", str(self.state_file),
                                "--index", str(self.pallet_no)], capture_output=True, text=True, cwd=WORKCELL)
            self.run.log("    " + (r.stdout.strip().splitlines() or ["(no output)"])[-1])
            if r.returncode != 0:
                self.run.log(f"[pallet_swap] FAIL rc {r.returncode}: {r.stderr.strip()[-400:]}")
                return self.fail(None, "pallet swap failed") or False
            core.on_result(cmd)
            core.supervisor.pallet_change(core.sm.t)
            self.pallet_no += 1
            self.placed_now = []
            self.state_file = self.run.dir / f"pallet_{self.pallet_no}_state.json"
            return True
        if a in ("PLACE_CURRENT", "RETRIEVE_BUFFER"):
            t = b.field.truth
            tracked = core.sm.tracked[cmd.box_id]
            x, y, z = self.pallet_to_world_centre(t.size, cmd.candidate.target_pose)
            yaw = cmd.candidate.target_pose.yaw
            extra = ["--slot", f"{x:.4f}", f"{y:.4f}", f"{z:.4f}", "--slot-yaw", f"{yaw:.4f}"]
            tag = "pallet"
            if a == "RETRIEVE_BUFFER":
                slot = next(s for s, v in self.buffer.items() if v["box"] == b.name)
                extra += ["--pick-from", "buffer", "--pick-pose", *[f"{v:.4f}" for v in self.buffer[slot]["pose"]]]
                tag = "frombuffer"
            ok, measured, _ = self.moveit(b, tag, extra)
            if not ok:
                return self.fail(b, f"{a} failed") or False
            if a == "RETRIEVE_BUFFER":
                del self.buffer[slot]
            b.done = True
            mx, my, mz, myaw = ((measured["x"], measured["y"], measured["z"], measured["yaw"]) if measured
                                else (x, y, z, yaw))
            pose = self.world_to_pallet_corner(tracked.size, mx, my, mz, myaw)
            level = core.on_result(cmd, ExecutionReport(True, 1, False, pose))
            self.placed_now.append((b.name, t.sku_id, (t.size.x, t.size.y, t.size.z)))
            dev = math.hypot(mx - x, my - y) * 1000
            self.run.log(f"[executor] {'PASS' if level == 'L0' else 'WARN'} place {b.name} post-check {level} "
                         f"dxy={dev:.1f}mm (slot {x:.3f}, {y:.3f}, {z:.3f})")
            self.run.event(b.name, "pallet_placed", reason=level)
            return True
        self.run.log(f"[planner] WARN unsupported command {a}; stopping")
        self.stop = True
        return False

    def main(self):
        self.run.log(f"[pipeline] order {self.args.scenario}: {len(self.boxes)} boxes, run {self.run.dir}")
        conv = threading.Thread(target=self.conveyor_worker, daemon=True)
        conv.start()
        exit_code, outcome = 1, "failed"
        try:
            self.decision_loop()
            if not self.stop:
                exit_code, outcome = 0, "complete"
        except KeyboardInterrupt:
            exit_code, outcome = 130, "interrupted"
            self.run.log("[pipeline] interrupted")
        except Exception as error:  # noqa: BLE001
            import traceback
            self.run.log(f"[pipeline] ERROR {error}")
            for line in traceback.format_exc().splitlines():
                self.run.log("    " + line)
        finally:
            self.stop = True
            conv.join(timeout=3)
            try:
                self.cell.roller(0.0)
            except Exception as error:  # noqa: BLE001 - preserve failure status and summary
                exit_code, outcome = 1, "failed"
                self.run.log(f"[pipeline] ERROR conveyor stop failed: {error}")
            summary = {"run_id": self.run.id, "scenario": self.args.scenario, "boxes": len(self.boxes),
                       "status": outcome, "exit_code": exit_code,
                       "placed": sum(len(b) for _, b in self.core.closed) + len(self.core.sm.placed),
                       "inspection": self.core.inspection, "pallets": self.pallet_no,
                       "decisions": self.decisions, "counts": dict(self.core.counts)}
            (self.run.dir / "summary.json").write_text(json.dumps(summary, indent=2, default=str))
            self.run.log(f"[pipeline] summary {self.run.dir / 'summary.json'}")
        return exit_code


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dataset", type=Path, required=True)
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--count", type=int, default=0, help="first N arriving boxes (0 = whole order)")
    return Runner(ap.parse_args()).main()


if __name__ == "__main__":
    raise SystemExit(main())
