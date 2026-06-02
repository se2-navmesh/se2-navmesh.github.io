#!/usr/bin/env bash
# Build the SE(2) navmesh for every scene and export its traversability field
# (field.bin) + scene.json. Launches the static node per scene, waits for the
# latched topics via export_field.py, then tears the node down. Run as a
# background task -- one full pass takes several minutes per scene.
set +e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SITE="$(cd "$HERE/.." && pwd)"
WS=/home/kaixqu/workspaces/objectnav_ws
SRC=$WS/src
CFG=$SRC/novo_3d/deps/se2_navmesh/se2_navmesh_ros/config
SG=$SRC/novo_3d/deps/se2_navmesh/se2_navmesh_ros_testing/config
source $WS/devel/setup.bash

# dir : full-scene-id : obj-basename  (OBJ name differs from id for the _1 scenes)
SCENES=(
  "00114:00114-Coer9RdivP7:00114-Coer9RdivP7.obj"
  "00403:00403-t3t9ofFLcFU:00403-t3t9ofFLcFU.obj"
  "00473:00473-xccdSFAEPau:00473-xccdSFAEPau_1.obj"
  "00654:00654-VZy9kKQJcUF:00654-VZy9kKQJcUF_1.obj"
  "00700:00700-aosjAwX5Lnq:00700-aosjAwX5Lnq.obj"
  "00797:00797-99ML7CGPqsQ:00797-99ML7CGPqsQ_1.obj"
)

teardown() { pkill -x se2_navmesh_static; pkill -x rosmaster; pkill -x roscore; sleep 4; }

for entry in "${SCENES[@]}"; do
  IFS=: read -r dir full obj <<< "$entry"
  out="$SITE/static/scenes/$dir"
  if [ -f "$out/field.bin" ] && grep -q asaStages "$out/scene.json" 2>/dev/null; then
    echo "SKIP $full (already has asaStages)"; continue
  fi
  mkdir -p "$out"
  echo "=== FIELD $full ==="
  teardown
  roslaunch "$HERE/web_export_field.launch" \
    mesh_file:=$SRC/mesh/$obj \
    recast_cfg:=$CFG/$full.yaml \
    startgoal:=$SG/${full}_start_goal_hard.yaml --screen >/tmp/node_$dir.log 2>&1 &
  RL=$!
  python3 "$HERE/export_field.py" --scene-dir "$out" --timeout 500
  echo "export $full rc=$?"
  kill $RL 2>/dev/null
  teardown
done
echo "ALL_FIELDS_DONE"
