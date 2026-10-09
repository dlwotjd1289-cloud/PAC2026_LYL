from dataclasses import asdict, is_dataclass
from enum import Enum
import json
from pathlib import Path
from typing import Any

from pac_common import (
    BoxState,
    BoxStatus,
    InventoryState,
    PalletState,
    PlacedBox,
    Pose3D,
    Size3D,
    SystemState,
)


def to_primitive(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if is_dataclass(value):
        return {key: to_primitive(val) for key, val in asdict(value).items()}
    if isinstance(value, dict):
        return {str(key): to_primitive(val) for key, val in value.items()}
    if isinstance(value, (tuple, list)):
        return [to_primitive(item) for item in value]
    return value


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(to_primitive(payload), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as file_obj:
        for row in rows:
            file_obj.write(
                json.dumps(to_primitive(row), ensure_ascii=False, sort_keys=True)
                + "\n"
            )


def size_from_dict(raw: dict[str, Any]) -> Size3D:
    return Size3D(x=float(raw["x"]), y=float(raw["y"]), z=float(raw["z"]))


def pose_from_dict(raw: dict[str, Any]) -> Pose3D:
    return Pose3D(
        frame_id=str(raw["frame_id"]),
        x=float(raw["x"]),
        y=float(raw["y"]),
        z=float(raw["z"]),
        roll=float(raw.get("roll", 0.0)),
        pitch=float(raw.get("pitch", 0.0)),
        yaw=float(raw.get("yaw", 0.0)),
    )


def box_from_dict(raw: dict[str, Any]) -> BoxState:
    return BoxState(
        box_id=str(raw["box_id"]),
        sku_id=str(raw["sku_id"]),
        size=size_from_dict(raw["size"]),
        weight_kg=float(raw["weight_kg"]),
        pose=pose_from_dict(raw["pose"]),
        allowed_yaws_rad=tuple(float(x) for x in raw["allowed_yaws_rad"]),
        status=BoxStatus(str(raw["status"])),
        confidence=float(raw["confidence"]),
        stamp_sec=float(raw["stamp_sec"]),
        source=str(raw["source"]),
    )


def placed_box_from_dict(raw: dict[str, Any]) -> PlacedBox:
    return PlacedBox(
        box_id=str(raw["box_id"]),
        sku_id=str(raw["sku_id"]),
        size=size_from_dict(raw["size"]),
        weight_kg=float(raw["weight_kg"]),
        pose=pose_from_dict(raw["pose"]),
    )


def pallet_from_dict(raw: dict[str, Any]) -> PalletState:
    return PalletState(
        pallet_id=str(raw["pallet_id"]),
        size=size_from_dict(raw["size"]),
        boxes=tuple(placed_box_from_dict(item) for item in raw.get("boxes", [])),
    )


def inventory_from_dict(raw: dict[str, Any]) -> InventoryState:
    return InventoryState(
        tracked_boxes={
            str(key): box_from_dict(value)
            for key, value in raw.get("tracked_boxes", {}).items()
        },
        remaining_by_sku={
            str(key): int(value)
            for key, value in raw.get("remaining_by_sku", {}).items()
        },
    )


def system_state_from_dict(raw: dict[str, Any]) -> SystemState:
    return SystemState(
        state_version=int(raw["state_version"]),
        stamp_sec=float(raw["stamp_sec"]),
        pallet=pallet_from_dict(raw["pallet"]),
        inventory=inventory_from_dict(raw["inventory"]),
    )


def load_system_state(path: Path) -> SystemState:
    payload = json.loads(path.read_text(encoding="utf-8"))
    return system_state_from_dict(payload["initial_system_state"])
