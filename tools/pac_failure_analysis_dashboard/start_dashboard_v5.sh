#!/usr/bin/env bash
# Compatibility alias. V5.2 always uses the explicit Claude opt-in launcher.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$HERE/start_dashboard_claude.sh" "$@"
