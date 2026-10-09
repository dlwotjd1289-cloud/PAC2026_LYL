#!/usr/bin/env bash
# Data generation -> depth perception -> stacking -> evaluation, one command.
#   ./run_pipeline.sh                      # 6 orders x 40 boxes -> runs/<timestamp>
#   ./run_pipeline.sh --ids S0001 --boxes 80
# Extra arguments go to `python -m pipeline.run` (see --help).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
OUT="runs/$(date +%Y%m%d_%H%M%S)"
if [[ " $* " == *" --output "* ]]; then
  exec python3 -m pipeline.run "$@"
fi
exec python3 -m pipeline.run --output "$OUT" "$@"
