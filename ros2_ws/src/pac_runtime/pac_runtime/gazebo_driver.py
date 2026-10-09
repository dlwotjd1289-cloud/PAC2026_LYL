"""Gazebo cell driver: plays the plant around the runtime node in the
pac2026-ahead workcell (``hdr50_workcell.launch.py``).

    conveyor (boxes from the order list) --/pac/observation--> runtime_node
    runtime_node --/pac/command--> this driver
        PLACE_CURRENT / RETRIEVE_BUFFER: HDR50-22 joint trajectory
            (home -> above pick -> pick -> above pick -> above place -> place -> above place)
            on /joint_trajectory_controller/joint_trajectory, then the box is
            spawned at the placed pose in Gazebo (no vacuum plugin in the
            workcell, so the box "appears" when the robot reaches it) and settles
            under Gazebo physics
        BUFFER_CURRENT / REJECT_NG: box leaves the conveyor (no motion)
        PALLET_CLOSE: placed boxes are removed (pallet change)
        PARTIAL_REPACK: moved boxes are re-spawned at their new poses
    this driver --/pac/execution_result--> runtime_node (measured pose = commanded
        pose + optional noise), then the next box arrives
    conveyor camera --/pac/conveyor_preview--> runtime_node: the next
        ``visible_boxes`` boxes after the current one (look-ahead window), sent
        right before each /pac/observation

``GazeboDriverCore`` holds the logic (unit-tested without ROS); ``main``
is the rclpy node. Joint values come from stage 6 (``cmd["robot"]``) and
``RobotFeasibility.pick_path`` with the Gazebo cell configuration
``config/taehyeon/robot_check_gazebo.yaml`` (robot on a 0.5 m pedestal at world
(1.35, 0.15), started by ``tools/runtime/launch/hdr50_pedestal_workcell.launch.py``).
"""

from dataclasses import dataclass, field
import json
import math
import random

import numpy as np

from pac_robot_check import HOME
from pac_robot_check.kinematics import VELOCITY


@dataclass
class GazeboCell:
    """Pallet frame (corner origin, z on the deck) -> Gazebo world."""
    pallet_center_xy: tuple = (1.35, -1.0)
    deck_top_z: float = 0.15
    pallet_size_xy: tuple = (1.2, 1.0)
    world: str = "ahead_workcell_v2"

    def box_center(self, min_corner, size):
        x, y, z, yaw = min_corner
        sx, sy, sz = size
        if int(round(yaw / (math.pi / 2))) % 2:
            sx, sy = sy, sx
        cx = self.pallet_center_xy[0] - self.pallet_size_xy[0] / 2 + x + sx / 2
        cy = self.pallet_center_xy[1] - self.pallet_size_xy[1] / 2 + y + sy / 2
        return cx, cy, self.deck_top_z + z + sz / 2, yaw


def box_sdf(name, size, mass):
    sx, sy, sz = size
    m = max(0.05, float(mass))
    ixx, iyy, izz = (m / 12 * (sy * sy + sz * sz), m / 12 * (sx * sx + sz * sz), m / 12 * (sx * sx + sy * sy))
    shade = max(0.2, 0.85 - 0.02 * m)
    geom = f"<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
    return (f'<sdf version="1.6"><model name="{name}"><link name="link">'
            f"<inertial><mass>{m}</mass><inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz>"
            f"<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>"
            f'<collision name="c">{geom}<surface><friction><ode><mu>0.6</mu><mu2>0.6</mu2></ode></friction></surface></collision>'
            f'<visual name="v">{geom}<material><ambient>{shade} {shade * 0.75} {shade * 0.45} 1</ambient>'
            f"<diffuse>{shade} {shade * 0.75} {shade * 0.45} 1</diffuse></material></visual>"
            f"</link></model></sdf>")


def boxes_from_order(order, seed=0):
    """Arrival list from an order list (SKU counts), shuffled, weights inside the SKU range."""
    rng = random.Random(seed)
    boxes = []
    for sku, s in sorted(order["skus"].items()):
        lo, hi = s["weight_kg"]
        for _ in range(int(s["count"])):
            boxes.append({"sku": sku, "size_m": list(s["size_m"]), "weight_kg": round(rng.uniform(lo, hi), 3)})
    rng.shuffle(boxes)
    for i, b in enumerate(boxes):
        b["box_id"] = f"G{i + 1:03d}"
    return boxes


@dataclass
class Actions:
    trajectory: list = field(default_factory=list)   # [(q list of 6, t_from_start_s)]
    spawn: list = field(default_factory=list)        # [(name, sdf, (x, y, z, yaw))]
    remove: list = field(default_factory=list)       # [model name]
    result: dict | None = None                       # -> /pac/execution_result (after the motion)
    preview: dict | None = None                      # -> /pac/conveyor_preview (before the observation)
    observation: dict | None = None                  # -> /pac/observation (after the result)
    idle: dict | None = None                         # -> /pac/conveyor_idle
    duration_s: float = 0.0


class GazeboDriverCore:
    def __init__(self, boxes, robot, cell=None, speed_scale=0.3, settle_s=1.0, place_noise_m=0.0, seed=0,
                 visible_boxes=5):
        self.boxes = list(boxes)
        self.by_id = {b["box_id"]: b for b in self.boxes}
        self.robot = robot
        self.cell = cell or GazeboCell()
        self.speed = speed_scale
        self.settle_s = settle_s
        self.noise = place_noise_m
        self.rng = random.Random(seed)
        self.next_index = 0
        self.visible_boxes = int(visible_boxes)
        self.q = HOME.copy()
        self.on_pallet = []
        self.poses = {}

    # -- conveyor -------------------------------------------------------
    @staticmethod
    def _message(b):
        return {"box_id": b["box_id"], "label_sku": b["sku"], "weight_kg": b["weight_kg"], "size_m": b["size_m"]}

    def _observation(self):
        if self.next_index >= len(self.boxes):
            return None
        b = self.boxes[self.next_index]
        self.next_index += 1
        return self._message(b)

    def preview(self):
        """Boxes behind the one at the pick point (what the camera sees)."""
        ahead = self.boxes[self.next_index: self.next_index + self.visible_boxes]
        return {"boxes": [self._message(b) for b in ahead]}

    def first_observation(self):
        return self._observation()

    def _after_conveyor_box_left(self, actions):
        actions.observation = self._observation()
        actions.preview = self.preview()
        if actions.observation is None:
            actions.idle = {"idle_s": 60.0}

    # -- motion ---------------------------------------------------------
    def _segment(self, points, q, t, min_s=0.6):
        q = np.asarray(q, dtype=float)
        dt = max(min_s, float(np.max(np.abs(q - self.q) / (VELOCITY * self.speed))))
        t += dt
        points.append(([float(v) for v in q], round(t, 3)))
        self.q = q
        return t

    def _pick_and_place(self, box, robot_details):
        points, t = [], 0.0
        up, down = self.robot.pick_path(box["size_m"][2])
        if up is not None and down is not None:
            t = self._segment(points, up, t)
            t = self._segment(points, down, t)
            t = self._segment(points, up, t)
        # down and up along the vertical line stage 6 checked: between the
        # approach and place poses a plain joint move swings the gripper
        # sideways into the neighbours (gaps are only a few mm)
        path = robot_details.get("q_path") or [robot_details["q_approach"], robot_details["q_place"]]
        t = self._segment(points, path[0], t)
        for q in path[1:]:
            t = self._segment(points, q, t, min_s=0.2)
        for q in path[-2::-1]:
            t = self._segment(points, q, t, min_s=0.2)
        return points, t

    def _measured(self, corner):
        x, y, z, yaw = corner
        if self.noise > 0:
            x += self.rng.gauss(0.0, self.noise)
            y += self.rng.gauss(0.0, self.noise)
        return [x, y, z, yaw]

    # -- commands -------------------------------------------------------
    def on_command(self, cmd):
        a = Actions()
        action, version = cmd["action"], cmd["state_version"]
        if action in ("PLACE_CURRENT", "RETRIEVE_BUFFER"):
            box = self.by_id[cmd["box_id"]]
            a.trajectory, a.duration_s = self._pick_and_place(box, cmd["robot"])
            measured = self._measured(cmd["target_min_corner"])
            a.spawn.append((box["box_id"], box_sdf(box["box_id"], box["size_m"], box["weight_kg"]),
                            self.cell.box_center(measured, box["size_m"])))
            self.on_pallet.append(box["box_id"])
            self.poses[box["box_id"]] = measured
            a.result = {"state_version": version, "ok": True, "attempts": 1, "measured_pose": measured}
            if action == "PLACE_CURRENT":
                self._after_conveyor_box_left(a)
        elif action in ("BUFFER_CURRENT", "REJECT_NG"):
            a.result = {"state_version": version, "ok": True}
            self._after_conveyor_box_left(a)
        elif action == "PALLET_CLOSE":
            a.remove = list(self.on_pallet)
            self.on_pallet, self.poses = [], {}
            a.result = {"state_version": version, "ok": True}
        elif action == "PARTIAL_REPACK":
            poses = {}
            for move in cmd.get("repack", []):
                bid = move["box_id"]
                box = self.by_id[bid]
                measured = self._measured(move["target_min_corner"])
                poses[bid] = measured
                self.poses[bid] = measured
                a.remove.append(bid)
                a.spawn.append((bid, box_sdf(bid, box["size_m"], box["weight_kg"]),
                                self.cell.box_center(measured, box["size_m"])))
                if move.get("robot", {}).get("q_place"):
                    a.trajectory.append((move["robot"]["q_place"], round(a.duration_s + 3.0, 3)))
                    a.duration_s += 3.0
            a.result = {"state_version": version, "ok": True, "repack_poses": poses}
        a.duration_s += self.settle_s if a.trajectory else 0.0
        return a


# ---------------------------------------------------------------------- ROS 2
def main(args=None):  # pragma: no cover - needs ROS 2 + Gazebo
    import subprocess

    import rclpy
    from builtin_interfaces.msg import Duration
    from rclpy.node import Node
    from std_msgs.msg import String
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    from pac_robot_check import RobotFeasibility, load_robot_check_config

    class GazeboCellNode(Node):
        def __init__(self):
            super().__init__("pac_gazebo_cell")
            p = {n: self.declare_parameter(n, d).value for n, d in (
                ("order_file", ""), ("robot_config", ""), ("world", "ahead_workcell_v2"),
                ("trajectory_topic", "/joint_trajectory_controller/joint_trajectory"),
                ("speed_scale", 0.3), ("place_noise_m", 0.0), ("seed", 0), ("start_delay_s", 3.0),
                ("visible_boxes", 5))}
            order = json.loads(open(p["order_file"], encoding="utf-8").read())
            pallet = order["pallet"]["size_m"]
            self.world = p["world"]
            self.core = GazeboDriverCore(
                boxes_from_order(order, int(p["seed"])),
                RobotFeasibility(load_robot_check_config(p["robot_config"])),
                GazeboCell(pallet_size_xy=(pallet[0], pallet[1]), world=self.world),
                speed_scale=float(p["speed_scale"]), place_noise_m=float(p["place_noise_m"]), seed=int(p["seed"]),
                visible_boxes=int(p["visible_boxes"]))
            self.traj_pub = self.create_publisher(JointTrajectory, p["trajectory_topic"], 10)
            self.obs_pub = self.create_publisher(String, "/pac/observation", 10)
            self.res_pub = self.create_publisher(String, "/pac/execution_result", 10)
            self.idle_pub = self.create_publisher(String, "/pac/conveyor_idle", 10)
            self.preview_pub = self.create_publisher(String, "/pac/conveyor_preview", 10)
            self.create_subscription(String, "/pac/command", self._command, 10)
            self.busy = False
            self.start = self.create_timer(float(p["start_delay_s"]), self._first)

        def _send(self, pub, payload):
            pub.publish(String(data=json.dumps(payload)))

        def _first(self):
            self.start.cancel()
            obs = self.core.first_observation()
            self._send(self.preview_pub, self.core.preview())
            if obs:
                self.get_logger().info(f"box {obs['box_id']} arrives")
                self._send(self.obs_pub, obs)

        def _spawn(self, name, sdf, pose):
            x, y, z, yaw = pose
            subprocess.Popen(["ros2", "run", "ros_gz_sim", "create", "-world", self.world, "-name", name,
                              "-string", sdf, "-x", str(x), "-y", str(y), "-z", str(z + 0.002), "-Y", str(yaw)],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        def _remove(self, name):
            for tool in ("ign", "gz"):
                try:
                    subprocess.Popen([tool, "service", "-s", f"/world/{self.world}/remove",
                                      "--reqtype", "ignition.msgs.Entity", "--reptype", "ignition.msgs.Boolean",
                                      "--timeout", "2000", "--req", f'name: "{name}" type: MODEL'],
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    return
                except FileNotFoundError:
                    continue

        def _command(self, msg):
            cmd = json.loads(msg.data)
            self.get_logger().info(f"command {cmd['action']} {cmd.get('box_id')}")
            acts = self.core.on_command(cmd)
            for name in acts.remove:
                self._remove(name)
            if acts.trajectory:
                jt = JointTrajectory(joint_names=["j1", "j2", "j3", "j4", "j5", "j6"])
                for q, t in acts.trajectory:
                    sec = int(t)
                    jt.points.append(JointTrajectoryPoint(positions=list(q),
                                                          time_from_start=Duration(sec=sec, nanosec=int((t - sec) * 1e9))))
                self.traj_pub.publish(jt)

            def finish():
                timer.cancel()
                for name, sdf, pose in acts.spawn:
                    self._spawn(name, sdf, pose)
                if acts.result is not None:
                    self._send(self.res_pub, acts.result)
                if acts.preview is not None:
                    self._send(self.preview_pub, acts.preview)
                if acts.observation is not None:
                    self.get_logger().info(f"box {acts.observation['box_id']} arrives")
                    self._send(self.obs_pub, acts.observation)
                if acts.idle is not None:
                    self.get_logger().info("conveyor empty")
                    self._send(self.idle_pub, acts.idle)

            timer = self.create_timer(max(0.2, acts.duration_s), finish)

    rclpy.init(args=args)
    node = GazeboCellNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
