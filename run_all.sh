#!/usr/bin/env bash
# Generate data only, then run the full Gazebo cell and its API dashboard.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATASET=""; SCENARIO="S0002"; COUNT=0; BOXES=""; OUTPUT=""
usage() {
  cat <<'HELP'
Usage: ./run_all.sh [--dataset DIR] [--scenario S0002] [--count 0]
                    [--boxes N] [--output DIR]
Default: generate the configured dataset, then run all boxes of S0002 in Gazebo.
--dataset DIR   Reuse existing generator output; skip generation.
--count 0       Entire order (default); positive N limits arriving boxes.
--boxes N       Boxes per generated scenario (default: config/pipeline.yaml).
--output DIR    New generation output directory (default: runs/full_TIMESTAMP_PID).
Set PAC_WORKCELL_REPO to the prepared V4.6 repository. No order time limit is imposed.
HELP
}
while (($#)); do
  case "$1" in
    --dataset|--scenario|--count|--boxes|--output)
      [[ $# -ge 2 && -n "$2" ]] || { echo "ERROR: missing value for $1" >&2; exit 2; }
      case "$1" in
        --dataset) DATASET="$2";; --scenario) SCENARIO="$2";; --count) COUNT="$2";;
        --boxes) BOXES="$2";; --output) OUTPUT="$2";;
      esac
      shift 2;;
    -h|--help) usage; exit 0;;
    *) echo "ERROR: unknown option $1" >&2; usage >&2; exit 2;;
  esac
done
[[ "$COUNT" =~ ^[0-9]+$ ]] || { echo 'ERROR: --count must be a nonnegative integer' >&2; exit 2; }
[[ -z "$BOXES" || "$BOXES" =~ ^[1-9][0-9]*$ ]] || { echo 'ERROR: --boxes must be positive' >&2; exit 2; }
if [[ -n "$DATASET" ]]; then
  [[ -d "$DATASET" ]] || { echo "ERROR: dataset directory not found: $DATASET" >&2; exit 2; }
  DATASET="$(cd "$DATASET" && pwd)"
  [[ -z "$BOXES" ]] || { echo 'ERROR: --boxes applies only when generating; omit it with --dataset' >&2; exit 2; }
fi
if [[ -n "$OUTPUT" ]]; then
  mkdir -p "$OUTPUT"
  OUTPUT="$(cd "$OUTPUT" && pwd)"
else
  OUTPUT="$ROOT/runs/full_$(date +%Y%m%d_%H%M%S)_$$"
fi
cd "$ROOT"
# The helper owns API-key handling; never put credentials on command lines or in logs.
source "$ROOT/workcell/dashboard_api_env.sh"
if [[ -z "$DATASET" ]]; then
  mkdir -p "$OUTPUT"
  DATASET="$OUTPUT/dataset"
  [[ ! -e "$DATASET" ]] || { echo "ERROR: generation target already exists: $DATASET" >&2; exit 2; }
  python3 - "$ROOT/config/pipeline.yaml" "$DATASET" "$BOXES" <<'PY'
import sys
from pathlib import Path
from pipeline.run import generate, load_pipeline_config
cfg = load_pipeline_config(Path(sys.argv[1]))
if sys.argv[3]:
    cfg['generator']['boxes_per_scenario'] = int(sys.argv[3])
generate(cfg, Path(sys.argv[2]))
PY
fi
printf 'Dataset: %s\nScenario: %s; count: %s (0 = entire order)\n' "$DATASET" "$SCENARIO" "$COUNT"
exec bash "$ROOT/workcell/run_gazebo_demo.sh" "$DATASET" "$SCENARIO" "$COUNT"
