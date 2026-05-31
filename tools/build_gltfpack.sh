#!/usr/bin/env bash
# Build tools/bin/gltfpack from source (meshoptimizer). The prebuilt release
# binaries need a newer glibc than Ubuntu 20.04, so we compile locally.
# Requires: git, cmake, a C++ compiler. No KTX2/basisu (textures stay JPEG;
# glb_shrink_textures.py handles texture size).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT

git clone --depth 1 https://github.com/zeux/meshoptimizer.git "$TMP/m"
cmake -DCMAKE_BUILD_TYPE=Release -DMESHOPT_BUILD_GLTFPACK=ON -B "$TMP/m/build" -S "$TMP/m"
cmake --build "$TMP/m/build" -j --target gltfpack

mkdir -p "$HERE/bin"
cp "$TMP/m/build/gltfpack" "$HERE/bin/gltfpack"
chmod +x "$HERE/bin/gltfpack"
"$HERE/bin/gltfpack" 2>&1 | head -1
echo "installed -> $HERE/bin/gltfpack"
