"""Load ``config/donghan/perception_depth.yaml`` into the stage 1 config dataclasses."""
from __future__ import annotations

from pathlib import Path

import yaml

from .depth_measurement import DepthConfig
from .sku_resolver import ResolverConfig


def load_perception_config(path: str | Path) -> tuple[DepthConfig, ResolverConfig]:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if data.get("schema_version") != 1:
        raise ValueError(f"unsupported schema_version: {data.get('schema_version')}")
    depth = dict(data["depth"])
    depth["roi_xy_m"] = tuple(depth["roi_xy_m"])
    return DepthConfig(**depth), ResolverConfig(**data["resolver"])
