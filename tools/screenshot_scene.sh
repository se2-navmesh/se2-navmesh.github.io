#!/usr/bin/env bash
# Headless-Chrome screenshot of one scene in the explorer (visual QA for
# alignment: the green/amber field should sit on the textured floor).
#
#   tools/screenshot_scene.sh <scene-dir> [out.png] [yaw=0]
#   e.g. tools/screenshot_scene.sh 00473 /tmp/00473.png
#
# Serves the site on a scratch port, loads tools/preview_harness.html?scene=<dir>
# (canvas at viewport top so headless WebGL renders reliably), then crops the
# canvas. Requires google-chrome and python3+Pillow.
set +e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SITE="$(cd "$HERE/.." && pwd)"
DIR="${1:?usage: screenshot_scene.sh <scene-dir> [out.png] [yaw]}"
OUT="${2:-/tmp/scene_$DIR.png}"
YAW="${3:-0}"
PORT=8079

python3 -m http.server "$PORT" --bind 127.0.0.1 --directory "$SITE" >/dev/null 2>&1 &
SRV=$!
trap 'kill $SRV 2>/dev/null' EXIT
sleep 2

RAW="$(mktemp --suffix=.png)"
timeout 90 google-chrome --headless=new --no-sandbox \
  --use-gl=angle --use-angle=swiftshader --enable-unsafe-swiftshader \
  --window-size=1280,720 --hide-scrollbars --virtual-time-budget=40000 \
  --screenshot="$RAW" \
  "http://127.0.0.1:$PORT/tools/preview_harness.html?scene=$DIR" >/dev/null 2>&1

python3 - "$RAW" "$OUT" <<'PY'
import sys
from PIL import Image
im = Image.open(sys.argv[1])
im.crop((40, 40, 910, 700)).resize((1300, 990)).save(sys.argv[2])
print("wrote", sys.argv[2])
PY
