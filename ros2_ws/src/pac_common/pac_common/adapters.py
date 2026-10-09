"""External JSON and optional ROS boundaries; never use dicts between algorithms."""

import math
from .models import (
    BoxState,
    BoxStatus,
    InventoryState,
    PalletState,
    PlacedBox,
    PlacementCandidate,
    Pose3D,
    Size3D,
    SystemState,
)
from .planning import PlanningContext, SkuSpec


def box_from_json(row):
    return BoxState(
        **{
            **row,
            "size": Size3D(**row["size"]),
            "pose": Pose3D(**row["pose"]),
            "allowed_yaws_rad": tuple(row["allowed_yaws_rad"]),
            "status": BoxStatus(row["status"]),
        }
    )


def state_from_json(row):
    p = row["pallet"]
    boxes = tuple(
        PlacedBox(
            **{**b, "size": Size3D(**b["size"]), "pose": Pose3D(**b["pose"])}
        )
        for b in p["boxes"]
    )
    inv = row["inventory"]
    return SystemState(
        row["state_version"],
        row["stamp_sec"],
        PalletState(p["pallet_id"], Size3D(**p["size"]), boxes),
        InventoryState(
            {k: box_from_json(b) for k, b in inv["tracked_boxes"].items()},
            inv["remaining_by_sku"],
        ),
    )


def context_from_json(row):
    data = dict(row)
    data["catalog"] = {
        k: SkuSpec(
            **{
                **v,
                "size": Size3D(**v["size"]),
                "allowed_yaws_rad": tuple(v["allowed_yaws_rad"]),
            }
        )
        for k, v in row["catalog"].items()
    }
    data["observed_preview"] = tuple(
        box_from_json(b) for b in row.get("observed_preview", ())
    )
    data["ems_upper_by_candidate"] = {
        k: Pose3D(**p)
        for k, p in row.get("ems_upper_by_candidate", {}).items()
    }
    return PlanningContext(**data)


def candidate_from_json(row):
    return PlacementCandidate(
        **{**row, "target_pose": Pose3D(**row["target_pose"])}
    )


def pose_to_ros_pose_stamped(pose, stamp_sec, factory=None):
    """ROS Clock stamp only. Factory injection permits a ROS-free contract test.

    This adapter transports the supplied reference point without silently moving
    it to the center. Robot callers explicitly use geometry.center_pose first.
    """
    if not math.isfinite(stamp_sec) or stamp_sec < 0:
        raise ValueError("Invalid ROS timestamp")
    if factory is None:
        from geometry_msgs.msg import PoseStamped

        factory = PoseStamped
    msg = factory()
    total_ns = round(stamp_sec * 1e9)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(total_ns, 10**9)
    msg.header.frame_id = pose.frame_id
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
        pose.x,
        pose.y,
        pose.z,
    )
    cr, sr = math.cos(pose.roll / 2), math.sin(pose.roll / 2)
    cp, sp = math.cos(pose.pitch / 2), math.sin(pose.pitch / 2)
    cy, sy = math.cos(pose.yaw / 2), math.sin(pose.yaw / 2)
    q = msg.pose.orientation
    q.x, q.y = sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy
    q.z, q.w = cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy
    return msg
