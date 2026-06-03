# Rebuilding the Interactive Path-Planning Explorer

This documents the May 2026 from-scratch rebuild of the `#explorer` section of
`se2-navmesh.github.io`, the full data pipeline behind it, and how to reproduce,
extend, and verify it. It is meant as a hand-off so the website can be improved
later without re-deriving any of this.

## 1. Why it was rebuilt

The previous explorer had three structural problems:

- **It didn't show the environment.** The mesh was cropped to a ~1 m-wide ribbon
  of upward-facing triangles hugging one path, so you saw a floating sliver of
  floor, not a place.
- **The "SE(2)" idea was invisible.** The navmesh was drawn as a flat, uniformly
  light-blue triangle soup. Yaw-dependent traversability — the entire point of the
  paper — was nowhere on screen, even though the intro text promised a yaw slider.
- **There was no planning.** Despite the title, it scrubbed a single pre-baked path.

## 2. What the new explorer does

Per scene (selectable, 6 HM3D scenes):

1. **Textured environment** — the real HM3D `.glb`, decimated + meshopt-compressed,
   baked into the navmesh coordinate frame.
2. **Yaw-dependent traversability field** — a colored carpet of surface cells, drawn
   binary: **safe** (fits at every heading) vs **restricted** (fits at only some).
   This makes the paper's core claim tangible. The layer is available through the
   `Traversability field` checkbox, but it is off by default so the textured scene
   and final polygon graph are the first visual signal.
3. **Final SE(2) NavMesh polygons** — a toggleable overlay of the Detour polygon
   graph exported by `se2_navmesh_static`, drawn as uniform semi-transparent blue
   polygons with light-blue outlines. The layer is enabled by default alongside
   the scene mesh and traversability field.
4. **Start/goal inspection markers** — the configured start and goal are shown as
   editable pose arrows, but route computation is currently disabled. The pose tool
   mirrors `se2_navmesh_rviz_plugin`'s `StartYawTool` / `GoalYawTool`: click
   **Set Start** or **Set Goal** to arm a one-shot edit, press on the scene to set
   `(x,y,z)`, drag to set yaw, and release to commit the pose. Start uses a green
   arrow; goal uses a red arrow. The arrow display origin is raised by
   `scene.json`'s `agent.height / 2` so it appears at the robot body height while
   preserving the actual pose `z`. Clicking the armed button again cancels the
   pending edit, which leaves normal left-drag orbit controls available.

### Current feature status

The old **Planned path**, **Robot footprint**, and **ASA reference (ROS)** features
were commented out in June 2026 because they no longer match the Explorer design
target. The old browser route used a span-cell lattice A\* over `field.bin`, while
the intended Explorer planner should use the exported Detour polygon graph in
`polyfield.bin` and reproduce the polygon-level ASA semantics of
`se2_navmesh_static`. The disabled code remains in `static/js/explorer.js`,
`index.html`, and `tools/preview_harness.html` with comments naming the affected
feature.

The active start/goal tool is separate from the disabled robot-footprint feature:
the arrows visualize query poses only and do not imply that route computation or
robot footprint sweeping has been re-enabled.

## 3. Architecture / data flow

Offline pipelines produce five static files per scene; the web app
consumes them. Everything stays in **one coordinate frame** (the navmesh / OBJ frame,
Z-up), so nothing is transformed at runtime.

```
 HM3D .glb (workspaces/data, 21-66 MB)         mesh/<id>.obj  (navmesh source, Z-up)
         |                                               |
         | blender_export_scene.py                       |  recast config (identity TRS)
         |   1. rotate (x,y,z) -> (-x,-z,-y)              v
         |   2. auto-align yaw to OBJ (0 or 180 deg)   se2_navmesh_static
         |   3. export Z-up (export_yup=False)         (web_export_field.launch:
         v                                              store_span_data + display_span_area)
  glb_shrink_textures.py  (PIL: downscale embedded JPEGs)        |
         |                                               | export_field.py
         | gltfpack  (-si simplify + -c meshopt)         |   (scrape latched markers)
         v                                               v
  static/scenes/<dir>/scene.glb  <---- same Z-up navmesh frame ---->  field.bin + scene.json
                                                         |
                                                         |  map_input_polygon_export.launch
                                                         |  ExportNavMeshPolygons.srv
                                                         v
                                                    polyfield.bin + polyfield.json
                         \                                          /
                          \                                        /
                           v                                      v
                         static/js/explorer.js  (Three.js GLTFLoader + MeshoptDecoder,
                          instanced safe/restricted field, polygon overlay;
                          legacy span-cell planner commented out)
```

`field.bin` is a flat little-endian array of `[float32 x, y, z, uint32 mask]` per
walkable surface cell; the low `yawBits` (=20) bits of `mask` are the feasible
headings over `[0, pi)` (the cuboid footprint is 180-deg symmetric, so heading `L`
and `L+bits` share feasibility). `scene.json` carries agent dims, yaw-layer info,
bounds, the configured start/goal, and the ASA reference path. The current Explorer
loads this metadata, but the ASA reference display is commented out.

`polyfield.bin` is the final Detour polygon graph, exported directly from
`dtNavMesh`. `polyfield.json` is its small sidecar with format, coordinate frame,
counts, bounds, and section metadata. The binary contains:

- header: magic/version/yaw metadata/counts;
- global vertices in ROS/navmesh coordinates;
- polygon records with `dtPolyRef`, vertex range, neighbor range, yaw mask, area,
  flags, tile ids, and centroid;
- a packed polygon vertex-index table;
- adjacency records with neighbor polygon id, neighbor ref, edge id, and portal
  endpoints.

The current explorer uses the polygon vertices and indices for display. The exported
neighbor records are intentionally kept for future polygon-level browser planning.

## 4. File inventory

**Web app**
- `static/js/explorer.js` — the viewer, traversability field, and polygon overlay.
  The legacy span-cell lattice-A\* planner, route display, robot footprint sweep,
  and ROS ASA reference display are commented out pending polygon-level ASA.
- `index.html` (`#explorer` section) + `static/css/index.css` (legend swatches).
- `static/scenes/index.json` — scene list (`dir`, `id`, `name`, `blurb`).
- `static/scenes/<dir>/{scene.glb, field.bin, scene.json, polyfield.bin, polyfield.json}` — per-scene assets.

**Mesh pipeline (no ROS)**
- `tools/blender_probe_bounds.py` — print a glb's bounds; used to derive the
  HM3D->navmesh rotation.
- `tools/blender_export_scene.py` — import glb, apply the rotation, **auto-align yaw to
  the OBJ**, optional crop/decimate, export Z-up.
- `tools/glb_shrink_textures.py` — PIL downscale of embedded textures + buffer repack
  (Blender 2.82's exporter re-emits original-res images, so we resize here).
- `tools/bin/gltfpack` — meshopt compressor/simplifier (built from source).
- `tools/build_gltfpack.sh` — build `tools/bin/gltfpack` from meshoptimizer.
- `tools/make_scene_glb.sh` — run the three steps for one scene.
- `tools/make_all_glbs.sh` — batch over all scenes (scene table inside).

**Field pipeline (ROS)**
- `tools/web_export_field.launch` — build the SE(2) NavMesh headless with span +
  per-cell area (yaw bitmask) publishing enabled; parametrized per scene.
- `tools/export_field.py` — subscribe to the latched span-area markers + planner
  mission, write `field.bin` + `scene.json`.
- `tools/export_all_fields.sh` — batch over all scenes (launch node, export, tear down).

**Polygon graph pipeline (ROS)**
- `se2_navmesh_ros/launch/map_input_polygon_export.launch` — build a scene and export
  `polyfield.bin` + `polyfield.json`.
- `se2_navmesh_ros/utils/NavMeshPolygonExporter` — reusable C++ exporter that reads
  final `dtNavMesh` tiles, polygons, links, masks, and portal edges.
- `se2_navmesh_msgs/ExportNavMeshPolygons.srv` — service used by
  `se2_navmesh_static` for on-demand polygon graph export.

**QA / preview**
- `tools/verify_scenes.py` — for every scene: field record count vs `scene.json`,
  glb world-AABB vs field footprint (catches mis-rotation), and a path for the
  configured query (mirrors the JS planner). Exits non-zero on any failure.
- `tools/preview_harness.html` — minimal explorer-only page (`?scene=<dir>`), for
  quick single-scene testing and reliable headless screenshots.
- `tools/screenshot_scene.sh` — headless-Chrome screenshot of one scene.

## 5. Reproduce / extend

Regenerate every scene from scratch:

```bash
tools/build_gltfpack.sh          # once, if tools/bin/gltfpack is missing/incompatible
tools/make_all_glbs.sh           # textured scene.glb for every scene (skips existing)
source <ws>/devel/setup.bash
tools/export_all_fields.sh       # field.bin + scene.json for every scene (skips existing)
# Use map_input_polygon_export.launch, or the ~export_navmesh_polygons service,
# to regenerate polyfield.bin + polyfield.json for each scene.
python3 tools/verify_scenes.py   # QA all scenes
```

Add a new scene:

1. Add it to the `SCENES` table in **both** `make_all_glbs.sh` and
   `export_all_fields.sh` as `dir:hash:objname` / `dir:id:objname`. (The OBJ name
   may differ from the scene id — some workspace meshes carry a `_1` suffix.)
2. It needs an HM3D glb in `workspaces/data/hm3d-train-glb-v0.2/<id>/`, a recast
   config and a `_start_goal_hard.yaml` under `se2_navmesh_ros[_testing]/config/`,
   and the navmesh OBJ in `mesh/`.
3. Run the two batches, add an entry to `static/scenes/index.json`, and verify.

## 6. Coordinate frames & alignment (gotchas)

- The navmesh, path, and field live in the **OBJ frame** (Z-up; all scenes use
  identity recast TRS).
- The HM3D glb, after Blender's glTF import, maps to the OBJ frame by
  **`(x,y,z) -> (-x,-z,-y)`** (a proper rotation, derived by matching vertex bounds).
- **The HM3D->OBJ export is not consistent across scenes:** some scenes (00403,
  00700) are additionally rotated **180 deg about Z** relative to the others. So
  `blender_export_scene.py` does not trust a fixed transform — given `--obj`, it
  picks the Z-rotation (0/90/180/270) whose AABB best matches the OBJ. `verify_scenes.py`
  re-checks this by comparing the glb's world AABB to the field footprint.
- Blender exports with `export_yup=False` (keeps Z-up), and Three.js renders with
  `camera.up=(0,0,1)`, so the glb and field overlay with **no runtime transform**.

## 7. Disabled legacy planner

The following span-cell planner is no longer active in the Explorer. Its code is
commented out in `static/js/explorer.js` under **Disabled feature: Planned path**
comments. It is kept only as implementation history and as a reference while the
browser planner is rebuilt on `polyfield.bin`.

The legacy planner used lattice A\* over nodes `(cell, headingLayer)`,
`headingLayer in [0, 2*bits)`:

- **Rotate in place**: `(c,L) -> (c,L±1)` if cell `c` is feasible at `L±1`.
- **Translate, optionally turning ±1 layer while stepping**: from a feasible `(c,L)`
  to a spatial neighbour `j` at layer `L+dL`, `dL in {-1,0,+1}`, if `j` is feasible at
  that layer. Spatial neighbours are the 8-neighbourhood with `|Δz| <= maxClimb`.

The translate-and-turn edge is essential: stair landings are a *patchwork* of cells
each feasible only at a narrow heading band, and pure translate-then-rotate cannot
cross a cell whose feasible headings don't overlap its neighbour's.

**Cost = traversal time**, mirroring the paper (`Method.tex` eq. 6-7) and the C++
ASA planner (`se2_navmesh_ros_testing/src/utils/testing_utils.cpp`). A translation
holding heading `psi` is split into longitudinal / lateral components and charged
`1/v_long` and `1/v_lat` per metre; an in-place yaw step costs `yawStep/omega`. With
the default `anymal` params (`v_long=0.5`, `v_lat=0.1`, `omega=0.5`), **sideways
motion is 5x dearer than forward**, so the planned path prefers to face the way it
travels instead of strafing. Params come from `scene.json`'s `agent` block when
present, else these defaults. The heuristic is the straight-line distance to the goal
scaled by the cheapest per-metre cost (`min(1/v_long, 1/v_lat)`), keeping A\*
admissible.

The traversability carpet is drawn as a **binary safe/restricted** field (safe = fits
at every heading, restricted = fits at only some); the exporter drops cells feasible
at no heading, so there is no third "blocked" state and no heading slider. The
checkbox is unchecked by default, and `explorer.js` applies that checkbox state after
each scene load so the field remains hidden until explicitly enabled.

The `SE(2) NavMesh` checkbox displays the final polygon graph from `polyfield.bin`.
It is checked by default, and `explorer.js` applies the checkbox state directly when
each scene finishes loading, so the polygon overlay appears on first load as well as
after scene changes. Polygon faces are always semi-transparent blue; the display does
not color polygons by yaw mask.

The currently displayed Explorer therefore does **not** compute or display a path,
does **not** animate the robot footprint, and does **not** draw the ROS ASA reference
path. The old UI controls and JavaScript hooks for those features are commented out
with feature-specific labels.

## 8. Known limitations / future ideas

- **Textures** are JPEG at 512 px (the ~2.5 MB floor per scene). KTX2/Basis (build
  gltfpack with basisu, add `KTX2Loader` + transcoder) would roughly halve size and
  sharpen them.
- **Planner fidelity** — the next active planner should use the exported Detour
  polygon graph for browser-side polygon-level ASA. The old span-cell planner is
  commented out because it does not match that target.
- **Cell rendering** is one `InstancedMesh` of quads; fine to ~15 k cells. Larger
  scenes may want a merged colored mesh.
- Start/goal snapping is a brute-force nearest-cell scan (fine at <16 k cells).

## 9. Environment notes (this workspace)

- Run the ROS node as a **background task**, not foreground (long-lived ROS children
  hold the stdout pipe and trip the tool timeout). Tear down with the task manager,
  not `pkill`.
- The login shell has `set -e`; guard scripts/commands (`set +e` / `|| true`) so a
  non-matching `pkill`/`grep` doesn't abort everything.
- Never `pkill -f <pattern>` where the pattern appears in the command's own path —
  it kills the running shell. Use `pkill -x <procname>`.
- `tools/bin/gltfpack` is built locally because the upstream release binaries need a
  newer glibc than Ubuntu 20.04.
