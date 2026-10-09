from pathlib import Path
import json

from ahead_dataset_generator.config import load_yaml
from ahead_dataset_generator.generator import generate_dataset
from ahead_dataset_generator.serialization import (
    box_from_dict,
    system_state_from_dict,
)
from pac_common import BoxState, SystemState


def test_generated_json_loads_into_pac_common(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    cfg = load_yaml(root / "config" / "default.yaml")
    output = tmp_path / "dataset"
    generate_dataset(cfg, output, sample_per_family=1, repo_root=root)

    scenario = json.loads(
        (output / "test_data" / "S0001.json").read_text(encoding="utf-8")
    )
    state = system_state_from_dict(scenario["initial_system_state"])
    assert isinstance(state, SystemState)
    assert state.pallet.size.x == 1.1
    assert state.pallet.size.y == 1.1

    truth = json.loads(
        (output / "ground_truth" / "S0001.json").read_text(encoding="utf-8")
    )
    first = box_from_dict(truth["arrival_events"][0]["box_state"])
    assert isinstance(first, BoxState)
    assert first.pose.frame_id == "conveyor"
    assert first.allowed_yaws_rad == (0.0, 1.5707963267948966)
