#!/usr/bin/env bash
# Build web-ready scene.glb for every scene in the table (HM3D glb -> aligned,
# textured, meshopt-compressed). Safe to re-run; skips scenes whose glb exists.
set +e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SITE="$(cd "$HERE/.." && pwd)"
DATA=/home/kaixqu/workspaces/data/hm3d-train-glb-v0.2

# dir : hash : obj-basename  (scene id = <dir>-<hash>; glb yaw is auto-aligned to the OBJ)
SCENES=(
  "00114:Coer9RdivP7:00114-Coer9RdivP7.obj"
  "00403:t3t9ofFLcFU:00403-t3t9ofFLcFU.obj"
  "00473:xccdSFAEPau:00473-xccdSFAEPau_1.obj"
  "00654:VZy9kKQJcUF:00654-VZy9kKQJcUF_1.obj"
  "00700:aosjAwX5Lnq:00700-aosjAwX5Lnq.obj"
  "00797:99ML7CGPqsQ:00797-99ML7CGPqsQ_1.obj"
)

for entry in "${SCENES[@]}"; do
  IFS=: read -r dir hash obj <<< "$entry"; full="${dir}-${hash}"
  out="$SITE/static/scenes/$dir/scene.glb"
  src="$DATA/$full/$hash.glb"
  if [ -f "$out" ]; then echo "SKIP $full (glb exists)"; continue; fi
  if [ ! -f "$src" ]; then echo "MISSING $src"; continue; fi
  echo "=== GLB $full ==="
  "$HERE/make_scene_glb.sh" "$src" "$out" 0.3 512 80 "/home/kaixqu/workspaces/objectnav_ws/src/mesh/$obj"
done
echo "ALL_GLBS_DONE"
