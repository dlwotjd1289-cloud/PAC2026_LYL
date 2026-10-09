#!/usr/bin/env bash
# Optional offline browser fallback, original upstream plotly.js MIT-licensed.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="$HERE/web/vendor/plotly.min.js"
mkdir -p "$(dirname "$DEST")"
if [ -s "$DEST" ]; then
  echo "Plotly bundle already present: $DEST"
  exit 0
fi
TMP="${DEST}.tmp.$$"
trap 'rm -f "$TMP"' EXIT
curl -fL --retry 2 --connect-timeout 10 --max-time 120 \
  'https://cdn.plot.ly/plotly-3.3.1.min.js' -o "$TMP"
if [ "$(stat -c%s "$TMP")" -lt 1000000 ]; then
  echo 'Downloaded asset unexpectedly small, refusing to install.' >&2
  exit 1
fi
mv "$TMP" "$DEST"
echo "Plotly downloaded: $DEST"
