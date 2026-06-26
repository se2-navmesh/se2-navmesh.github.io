#!/usr/bin/env bash
# Produce a web-ready, navmesh-aligned, compressed scene.glb from an HM3D .glb.
#
#   tools/make_scene_glb.sh <src.glb> <out.glb> [simplify=0.25] [tex=1024] [quality=82]
#
# Pipeline:  Blender (align HM3D->navmesh frame, Z-up export)
#         -> PIL  (downscale embedded textures)
#         -> gltfpack (-si simplify + -c meshopt compression; KHR_mesh_quantization)
#
# The output is in the navmesh frame and loads in three.js with GLTFLoader +
# MeshoptDecoder. Geometry ~10x smaller via meshopt; textures via JPEG resize.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$1"; OUT="$2"
SIMPLIFY="${3:-0.3}"; TEX="${4:-512}"; QUALITY="${5:-80}"; OBJ="${6:-}"
TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT

OBJ_ARG=()
[ -n "$OBJ" ] && OBJ_ARG=(--obj "$OBJ")   # auto-align glb yaw to the navmesh OBJ

echo "[1/3] Blender align + Z-up export ..."
blender --background --python "$HERE/blender_export_scene.py" -- \
  --in "$SRC" --out "$TMP/aligned.glb" --tris 99999999 --tex 4096 --draco 0 "${OBJ_ARG[@]}" \
  2>&1 | grep -E "ALIGNED|ALIGN_YAW|EXPORT" || true

echo "[2/3] PIL texture downscale (max=$TEX q=$QUALITY) ..."
python3 "$HERE/glb_shrink_textures.py" "$TMP/aligned.glb" "$TMP/tex.glb" "$TEX" "$QUALITY"

echo "[3/3] gltfpack simplify=$SIMPLIFY + meshopt ..."
mkdir -p "$(dirname "$OUT")"
"$HERE/bin/gltfpack" -i "$TMP/tex.glb" -o "$OUT" -si "$SIMPLIFY" -sa -c

echo "DONE -> $OUT ($(du -h "$OUT" | cut -f1))"
