# SE(2) NavMesh Project Page

Anonymous static project page for `se2-navmesh.github.io`. Everything needed for
the site lives in this folder and can be uploaded directly to GitHub Pages.

## Interactive Path-Planning Explorer

The headline interactive (`#explorer`) is a from-scratch Three.js viewer driven by
**real, ROS-generated SE(2) NavMesh data** on **textured HM3D scenes**:

- **Textured environment** — the original HM3D `.glb`, decimated + meshopt-compressed
  and baked into the navmesh coordinate frame (`static/scenes/<id>/scene.glb`).
- **Yaw-dependent traversability field** — one cell per walkable surface patch, each
  carrying a bitmask of feasible headings (`field.bin`), colored **safe** (fits at
  every heading) vs **restricted** (fits at only some). This makes the paper's core
  claim tangible.
- **Live in-browser planning** — click a start and a goal and a yaw-aware path is
  planned by a browser-side polygon ASA pipeline over `polyfield.bin`: exact
  yaw-layer start/goal snapping, first-stage polygon/yaw A\*, Detour-style
  string pulling with all portal crossings, and a second crossing-constrained
  A\*. Directional cost makes lateral motion more expensive than forward motion,
  so the route prefers to face the way it travels.
- **Scene selector** across several HM3D scenes; each scene opens with its
  configured start/goal query and computes the route live in the browser.

Scenes are listed in `static/scenes/index.json`; each `static/scenes/<dir>/` holds
`scene.glb`, `field.bin`, `polyfield.bin`, `polyfield.json`, and `scene.json`
(agent, yaw layers, bounds, and start/goal).

> **Full write-up:** [`docs/explorer-rebuild.md`](docs/explorer-rebuild.md) — the
> rebuild rationale, architecture/data-flow, coordinate frames, the planner, how to
> add a scene, and environment gotchas. Read that first to extend the explorer.

## Regenerating scene assets

Two independent pipelines feed each scene. Both are scripted in `tools/`.

### 1. Textured mesh (no ROS)

```bash
tools/make_scene_glb.sh <hm3d_source.glb> static/scenes/<dir>/scene.glb [simplify=0.3] [tex=512] [quality=80]
```

Pipeline: **Blender** (`blender_export_scene.py`) imports the HM3D glb, rotates it
into the navmesh frame (`(x,y,z) -> (-x,-z,-y)`, derived by matching the workspace OBJ
bounds — see `blender_probe_bounds.py`), exports Z-up → **PIL** (`glb_shrink_textures.py`)
downscales the embedded textures → **gltfpack** (`tools/bin/gltfpack`) simplifies and
meshopt-compresses. Result is a few MB, in the navmesh frame, loaded in three.js with
`GLTFLoader` + `MeshoptDecoder`.

`tools/bin/gltfpack` is built from [zeux/meshoptimizer](https://github.com/zeux/meshoptimizer)
(`cmake -DMESHOPT_BUILD_GLTFPACK=ON`); rebuild it for your platform if needed.

`tools/make_all_glbs.sh` runs this for every scene in its table.

### 2. SE(2) traversability field (ROS)

```bash
# one scene
roslaunch tools/web_export_field.launch \
  mesh_file:=$WS/src/mesh/<id>.obj \
  recast_cfg:=.../config/<id>.yaml \
  startgoal:=.../config/<id>_start_goal_hard.yaml
python3 tools/export_field.py --scene-dir static/scenes/<dir>
```

The launch builds the SE(2) NavMesh with `store_span_data` + `display_span_area`
enabled; `export_field.py` subscribes to the latched span-area markers (per-cell yaw
bitmask) and the planner mission (continuous-yaw ASA path) and writes `field.bin` +
`scene.json`. `tools/export_all_fields.sh` runs this for every scene sequentially.

### Verify & preview

```bash
python3 tools/verify_scenes.py            # QA all scenes: alignment + a planned path
tools/validate_explorer_paths.sh          # headless-Chrome ASA planner sweep
tools/screenshot_scene.sh 00473 out.png   # headless-Chrome render of one scene
python3 -m http.server 8020               # then /tools/preview_harness.html?scene=00473
```

## Other interactive components

Static widgets in `static/js/interactives.js` (vanilla JS, no build step): video
segmented control, NavMesh vs. SE(2) comparison slider, yaw-feasibility demo, ASA
stepper, and the SPC benchmark chart. MathJax and Chart.js load from CDN; Three.js
(+ addons) loads from the import map in `index.html`.

## Videos

The Videos section expects three MP4s (posters show until the files exist):
`static/videos/overview.mp4`, `static/videos/real_world.mp4`,
`static/videos/online_generation.mp4`.

## Local preview

```bash
python3 -m http.server 8020
# open http://127.0.0.1:8020/
```
