"""Shared configuration: config/default.yaml is the single source of team values.

Search order for the file: explicit path argument, $PAC_COMMON_CONFIG, then the
first config/default.yaml found walking up from this package, then from the
current directory (works from the monorepo and from a colcon symlink install).
"""

from dataclasses import dataclass
import os
from pathlib import Path

import yaml

from .models import Size3D, finite, identifier

ENV_VAR = "PAC_COMMON_CONFIG"


@dataclass(frozen=True)
class PalletSpec:
    pallet_id: str
    size_x_m: float
    size_y_m: float
    deck_height_m: float
    max_stack_height_m: float   # above the deck top
    max_load_kg: float

    def __post_init__(self):
        identifier(self.pallet_id, "pallet_id")
        for name in ("size_x_m", "size_y_m", "deck_height_m",
                     "max_stack_height_m", "max_load_kg"):
            finite(getattr(self, name), name, 0)
        if min(self.size_x_m, self.size_y_m, self.max_stack_height_m) <= 0:
            raise ValueError("Pallet footprint and stack height must be positive")

    @property
    def cargo_size(self) -> Size3D:
        """PalletState.size: deck footprint x max cargo height above the deck."""
        return Size3D(self.size_x_m, self.size_y_m, self.max_stack_height_m)

    @property
    def max_total_height_m(self) -> float:
        """Floor to top of the highest allowed box."""
        return self.deck_height_m + self.max_stack_height_m


@dataclass(frozen=True)
class CommonConfig:
    path: Path
    pallet: PalletSpec
    min_support_ratio: float
    raw: dict


def _is_team_config(path: Path) -> bool:
    """Module configs (e.g. the generator's) also have a `pallet:` section."""
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError:
        return False
    return isinstance(raw, dict) and "max_stack_height_m" in (raw.get("pallet") or {})


def find_common_config(path=None) -> Path:
    if path is not None:
        return Path(path)
    if os.environ.get(ENV_VAR):
        return Path(os.environ[ENV_VAR])
    # Package location first (monorepo / colcon --symlink-install), then cwd.
    for start in (Path(__file__).resolve(), Path.cwd()):
        for parent in (start, *start.parents):
            candidate = parent / "config" / "default.yaml"
            if candidate.is_file() and _is_team_config(candidate):
                return candidate
    raise FileNotFoundError(
        f"config/default.yaml not found; set ${ENV_VAR} to the team config"
    )


def load_common_config(path=None) -> CommonConfig:
    file = find_common_config(path)
    raw = yaml.safe_load(file.read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or raw.get("schema_version") != 1:
        raise ValueError(f"Unsupported common config schema: {file}")
    p = raw["pallet"]
    pallet = PalletSpec(
        pallet_id=str(p["pallet_id"]),
        size_x_m=float(p["size_x_m"]),
        size_y_m=float(p["size_y_m"]),
        deck_height_m=float(p["deck_height_m"]),
        max_stack_height_m=float(p["max_stack_height_m"]),
        max_load_kg=float(p["max_load_kg"]),
    )
    support = float(raw["constraint"]["min_support_ratio"])
    finite(support, "min_support_ratio", 0)
    return CommonConfig(file, pallet, support, raw)
