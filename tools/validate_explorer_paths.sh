#!/usr/bin/env bash
# Copyright (c) 2026 Shuyang Shi
# Licensed under the BSD 3-Clause License
#
# Headless browser validation for the Explorer's browser-side ASA planner.
#
# Usage:
#   tools/validate_explorer_paths.sh [scene-dir ...]
#
# Environment:
#   PORT=8003
#   CHROME=google-chrome
#   VIRTUAL_TIME_BUDGET=30000
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SITE="$(cd "$HERE/.." && pwd)"
PORT="${PORT:-8003}"
CHROME="${CHROME:-google-chrome}"
VIRTUAL_TIME_BUDGET="${VIRTUAL_TIME_BUDGET:-30000}"

if ! command -v "$CHROME" >/dev/null 2>&1; then
  echo "ERROR: Chrome executable not found: $CHROME" >&2
  exit 127
fi

if [ "$#" -gt 0 ]; then
  SCENES=("$@")
else
  mapfile -t SCENES < <(python3 - "$SITE/static/scenes/index.json" <<'PY'
import json
import sys

for scene in json.load(open(sys.argv[1])):
    print(scene["dir"])
PY
)
fi

python3 -m http.server "$PORT" --bind 127.0.0.1 --directory "$SITE" >/tmp/se2_explorer_http.log 2>&1 &
SRV=$!
cleanup() {
  kill "$SRV" 2>/dev/null || true
}
trap cleanup EXIT
sleep 2

status=0
printf '%-7s | %-18s | %-28s | %-10s | %-10s | %-8s | %-9s | %-6s\n' \
  "scene" "length" "cost" "startRef" "goalRef" "corridor" "crossings" "nodes"
printf '%s\n' "--------+--------------------+------------------------------+------------+------------+----------+-----------+--------"

for scene in "${SCENES[@]}"; do
  out="$(mktemp --suffix=.html)"
  err="$(mktemp --suffix=.log)"
  url="http://127.0.0.1:$PORT/tools/preview_harness.html?scene=$scene"
  profile="/tmp/se2-chrome-${scene}-validate-$$"

  if ! timeout 120 "$CHROME" \
      --headless=new \
      --no-sandbox \
      --disable-gpu \
      --use-gl=swiftshader \
      --enable-unsafe-swiftshader \
      --virtual-time-budget="$VIRTUAL_TIME_BUDGET" \
      --dump-dom \
      --user-data-dir="$profile" \
      "$url" >"$out" 2>"$err"; then
    echo "$scene | ERROR: headless Chrome failed" >&2
    tail -n 20 "$err" >&2 || true
    status=1
    continue
  fi

  if ! python3 - "$scene" "$out" <<'PY'; then
import html
import json
import re
import sys

scene, path = sys.argv[1], sys.argv[2]
dom = open(path).read()

def text_for_id(element_id):
    match = re.search(r'id="' + re.escape(element_id) + r'">([^<]*)', dom)
    return html.unescape(match.group(1)) if match else ""

hint_match = re.search(r'id="query-hint" class="viewer-note">([^<]*)', dom)
hint = html.unescape(hint_match.group(1)) if hint_match else ""
debug_match = re.search(r'<pre id="plan-debug-json" hidden="">(.*?)</pre>', dom, re.S)
debug = json.loads(html.unescape(debug_match.group(1))) if debug_match and debug_match.group(1).strip() else {}

if "error" in debug:
    print(f"{scene} | ERROR: {debug['error']}", file=sys.stderr)
    raise SystemExit(1)
if hint != "ASA path computed.":
    print(f"{scene} | ERROR: {hint or 'missing planner hint'}", file=sys.stderr)
    raise SystemExit(1)

print(
    f"{scene:<7} | "
    f"{text_for_id('stat-len'):<18} | "
    f"{text_for_id('stat-cost'):<28} | "
    f"{debug.get('start', {}).get('ref')!s:<10} | "
    f"{debug.get('goal', {}).get('ref')!s:<10} | "
    f"{len(debug.get('corridorRefs', [])):<8} | "
    f"{len(debug.get('crossings', [])):<9} | "
    f"{len(debug.get('secondNodeRefs', [])):<6}"
)
PY
    status=1
  fi
done

exit "$status"
