# Rebuilding the Interactive Path-Planning Explorer

This document records the May/June 2026 rebuild of the `#explorer` section of
`se2-navmesh.github.io`: what the Explorer shows, which static assets feed it,
how browser-side ASA planning now works, and how to regenerate and validate the
scene bundle.

The important current state is simple:

- the textured scene, traversability field, final polygon graph, and browser
  planned path are all displayed in the same Z-up navmesh frame;
- `field.bin` is now a visualization overlay only;
- `polyfield.bin` / `polyfield.json` are the route-planning graph;
- `scene.json` stores metadata and configured start/goal, but no longer stores
  `referencePath`;
- paths are computed online in the browser with a polygon-level ASA pipeline.

## 1. Why It Was Rebuilt

The previous explorer had three structural problems:

- **It did not show the environment.** The mesh was cropped to a narrow ribbon of
  floor near one path, so the viewer did not communicate place or scale.
- **Yaw-dependent traversability was hidden.** The old SE(2) overlay looked like
  a flat blue triangle soup even though the paper's core idea is footprint/yaw
  feasibility.
- **The path was pre-baked.** The old interaction scrubbed a stored route instead
  of computing a path from the scene data.

The rebuild turns the Explorer into a real static-web demo: the browser loads
ROS-generated scene assets and computes a yaw-aware route live.

## 2. What the Explorer Does

Per scene, the Explorer loads and displays:

1. **Textured environment**: the real HM3D `.glb`, decimated and
   meshopt-compressed, baked into the navmesh coordinate frame.
2. **Yaw-dependent traversability field**: an optional safe/restricted carpet of
   span cells from `field.bin`. Safe means the robot fits at every heading;
   restricted means it fits at only some headings.
3. **Final SE(2) NavMesh polygons**: the exported Detour polygon graph from
   `polyfield.bin`, drawn as semi-transparent blue polygons with light outlines.
4. **Editable start/goal pose arrows**: click **Set Start** or **Set Goal**, then
   left-drag on the scene to set position and yaw. The arrows are raised by
   `agent.height / 2` for display, while the stored pose `z` remains on the
   navmesh surface.
5. **Browser-computed ASA path**: the configured query is planned on load, and
   every pose edit replans immediately.

The path tube is also raised by `scene.json`'s `agent.height / 2`, so the drawn
route appears at robot-body height instead of lying directly on the floor.

The robot footprint sweep/playback remains commented out. It can be re-enabled
later using the browser-computed route, but it is not part of the current UI.

## 3. Architecture and Data Flow

Offline pipelines produce the static files consumed by `static/js/explorer.js`.
Everything stays in the navmesh / OBJ frame, which is Z-up.

```text
 HM3D .glb                                      mesh/<id>.obj
    |                                               |
    | blender_export_scene.py                       | recast config
    |   rotate/align to OBJ frame                   v
    v                                         se2_navmesh_static
 glb_shrink_textures.py                            / \
    |                                             /   \
 gltfpack                                      field   polygon graph
    |                                           export  export
    v                                             |       |
 static/scenes/<dir>/scene.glb                    |       |
                                                  v       v
                                      field.bin + scene.json
                                             polyfield.bin + polyfield.json
```

The browser loads:

- `scene.glb` for textured rendering;
- `field.bin` for the optional traversability overlay;
- `scene.json` for agent parameters, bounds, yaw metadata, and configured query;
- `polyfield.bin` / `polyfield.json` for polygon display and planning.

## 4. Scene Asset Format

Each `static/scenes/<dir>/` contains:

- `scene.glb`
- `field.bin`
- `scene.json`
- `polyfield.bin`
- `polyfield.json`

`scene.json` currently contains:

- `field`: record layout, counts, cell size, yaw bits, yaw step, bounds;
- `agent`: length, width, height, climb/slope limits;
- `start`: configured `{ x, y, z, layer }`;
- `goal`: configured `{ x, y, z, layer }`.

`referencePath` has been removed from all scene JSON files. The Explorer does
not depend on a ROS reference path at runtime.

`field.bin` is a flat little-endian array of:

```text
float32 x, y, z
uint32 mask
```

The low `yawBits` bits of `mask` encode feasible headings over `[0, pi)`. The
robot footprint is 180-degree symmetric, so layer `L` and `L + yawBits` share the
same mask bit.

`polyfield.bin` is a compact Detour polygon graph snapshot. It contains:

- header: magic/version/yaw metadata/counts;
- global polygon vertices in navmesh coordinates;
- polygon records with `dtPolyRef`, vertex range, neighbor range, yaw mask, area,
  flags, tile ids, and centroid;
- polygon index buffer;
- directed adjacency records with neighbor compact id, neighbor `dtPolyRef`, edge
  id, and clipped portal endpoints.

`NavMeshPolygonExporter` exports Detour-equivalent clipped portal endpoints. The
browser funnel uses the endpoint order as written: `portalA` is left and
`portalB` is right in the browser XY funnel frame.

## 5. Browser Planner

The active planner is in `static/js/explorer.js`. It follows the same high-level
sequence as `RecastPlannerRos::queryMultiLayer`:

1. layer-sensitive nearest-poly lookup for start;
2. layer-sensitive nearest-poly lookup for goal;
3. first A* over polygon-edge/yaw states;
4. corridor extraction and yaw-mask-aware compression;
5. Detour-style string pulling with all portal crossings;
6. crossing-map construction keyed by `fromRef:toRef`;
7. second A* constrained by the crossing map;
8. final SE(2) route output and display.

The planner keeps compact polygon ids for array indexing, but planning keys and
debug output use original Detour `dtPolyRef` values wherever comparison with ROS
matters.

### 5.1 Query Snapping

Start and goal layers are exact 1-based yaw layers. The browser does not snap a
pose to a different yaw layer.

`findNearestPolyMultiLayerWeb()` searches polygons whose bounds overlap the
configured search extent, rejects polygons that do not support the requested
layer, computes the closest point on each candidate polygon, and rejects nearest
points outside the search extent.

### 5.2 A* State

The A* state is:

```text
(polyA, polyB, yawLayer, position)
```

where:

- `polyA == polyB` means a same-polygon node;
- first-pass edge nodes are unordered and sorted by original `dtPolyRef`;
- second-pass edge nodes are ordered `from -> to`;
- same-polygon node keys include position;
- edge node keys do not include position, matching Detour node-pool identity more
  closely.

Yaw transitions check the circular lower/upper layers and cost
`singleYawLayerCost`. Spatial transitions use directional translation cost:

```text
dist * abs(cos(moveYaw - headingYaw)) * unitLonDistCost
+ dist * abs(sin(moveYaw - headingYaw)) * unitLatDistCost
```

The displayed `fwd`, `lat`, and `turn` stats are route summaries, not the raw A*
cost. They split each final route segment into forward/lateral distance relative
to the segment's starting yaw and accumulate circular yaw deltas.

### 5.3 Corridor and String Pulling

After the first A* succeeds, the browser:

1. extracts a polygon path from the `(polyA, polyB)` chain;
2. compresses `A, B, A` patterns only when polygon `A` is safe at every yaw bit;
3. runs a Detour-style funnel over the corridor;
4. appends all portal crossings, not just visible corners;
5. builds `crossingMap[fromRef:toRef] = crossingPosition`.

`segmentPortalIntersection2D()` computes the 2D intersection and interpolates the
final crossing point along the portal, mirroring Detour's `dtVlerp(left, right,
t)` behavior.

The second A* expands only along crossing-map edges and uses those string-pulled
crossing positions instead of raw portal midpoints.

### 5.4 Debug Output

Every plan stores debug output in:

- `window.__se2ExplorerLastPlanDebug`;
- `S.lastPlanDebug` inside the Explorer closure;
- hidden `<pre id="plan-debug-json">` in `tools/preview_harness.html`.

The debug object records snapped refs/positions/layers, first-stage ref/layer
nodes, compressed corridor refs, crossing-map entries, and second-stage ref/layer
nodes.

## 6. File Inventory

**Web app**

- `static/js/explorer.js`: viewer, traversability overlay, polygon graph display,
  browser ASA planner, path display, pose tools.
- `index.html`: main page and Explorer markup.
- `static/css/index.css`: Explorer layout and controls.
- `static/scenes/index.json`: scene list.
- `static/scenes/<dir>/...`: per-scene assets.

**Mesh pipeline**

- `tools/blender_probe_bounds.py`: inspect GLB bounds.
- `tools/blender_export_scene.py`: import GLB, align to OBJ/navmesh frame, export
  Z-up GLB.
- `tools/glb_shrink_textures.py`: downscale embedded textures.
- `tools/bin/gltfpack`: meshopt compressor/simplifier.
- `tools/build_gltfpack.sh`: build `gltfpack`.
- `tools/make_scene_glb.sh`: build one `scene.glb`.
- `tools/make_all_glbs.sh`: batch GLB generation.

**Field pipeline**

- `tools/web_export_field.launch`: build the SE(2) NavMesh with span data and
  area-marker publishing enabled.
- `tools/export_field.py`: write `field.bin` and `scene.json`.
- `tools/export_all_fields.sh`: batch field export.

**Polygon graph pipeline**

- `se2_navmesh_ros/launch/map_input_polygon_export.launch`: build a scene and
  export `polyfield.bin` / `polyfield.json`.
- `se2_navmesh_ros/utils/NavMeshPolygonExporter`: C++ Detour graph exporter.
- `se2_navmesh_msgs/ExportNavMeshPolygons.srv`: on-demand polygon export service.

**QA / preview**

- `tools/preview_harness.html`: minimal Explorer-only page.
- `tools/validate_explorer_paths.sh`: headless-Chrome browser ASA sweep.
- `tools/screenshot_scene.sh`: headless-Chrome screenshot of one scene.
- `tools/verify_scenes.py`: legacy scene/field QA. Treat planner-specific logic
  there as historical unless it is updated to the polygon ASA implementation.

## 7. Regenerate Assets

Regenerate every scene from scratch:

```bash
tools/build_gltfpack.sh
tools/make_all_glbs.sh
source <ws>/devel/setup.bash
tools/export_all_fields.sh
# Then regenerate polyfield.bin + polyfield.json for each scene with:
#   se2_navmesh_ros/launch/map_input_polygon_export.launch
# or the ~export_navmesh_polygons service.
```

For one polygon graph export, use the launch file directly:

```bash
roslaunch se2_navmesh_ros map_input_polygon_export.launch \
  mesh_file_path:=/path/to/<scene>.obj \
  recast_config:=/path/to/<scene>.yaml \
  agent_config:=/path/to/anymal_params.yaml \
  start_goal_config:=/path/to/<scene>_start_goal_hard.yaml \
  polygon_graph_output_dir:=/path/to/static/scenes/<dir> \
  polygon_graph_file_stem:=polyfield
```

After regenerating assets, validate the browser planner:

```bash
tools/validate_explorer_paths.sh
tools/validate_explorer_paths.sh 00403
```

In sandboxed environments, headless Chrome may need to run outside the sandbox
because crashpad/WebGL use sockets and helper processes.

## 8. Current Browser Validation

`tools/validate_explorer_paths.sh` starts a local HTTP server, opens the preview
harness in headless Chrome, reads `plan-debug-json`, and prints path statistics.

Current configured-query results:

```text
scene   | length  | startRef | goalRef | corridor | crossings | nodes
00114   | 10.76 m | 4456461  | 6291473 | 61       | 60        | 106
00403   | 7.48 m  | 4620293  | 4948015 | 91       | 90        | 130
00473   | 19.12 m | 5668900  | 5472272 | 151      | 150       | 205
00654   | 33.19 m | 5308429  | 4349959 | 142      | 141       | 190
00700   | 13.08 m | 4833298  | 5914633 | 76       | 75        | 101
00797   | 12.07 m | 5046274  | 6160388 | 51       | 50        | 68
```

For `00114`, the browser snap refs and path length match the ROS verbose export
baseline:

```text
startRef = 4456461
goalRef  = 6291473
Path length = 10.76
```

## 9. Coordinate Frames and Alignment

- Navmesh, polygon graph, field, poses, and browser path live in the OBJ/navmesh
  frame, Z-up.
- HM3D GLBs are rotated into that frame offline by `blender_export_scene.py`.
- Some scenes need an additional 180 degree yaw alignment. The Blender exporter
  uses OBJ bounds to choose the best alignment.
- Three.js renders with `camera.up = (0, 0, 1)`, so scene mesh, field, polygons,
  markers, and path overlay with no runtime frame transform.

## 10. Known Boundaries

- The browser graph is a compact Detour export, not a full Detour navmesh. Closest
  point and boundary operations are browser-side geometry approximations over the
  exported polygons.
- JavaScript bitwise mask handling assumes current `yawBits = 20`; masks with 32
  or more bits need a different representation.
- The Explorer validates configured scene queries. Interactive user-placed poses
  should be spot-checked after changing snapping, cost, or string-pulling logic.
- Robot footprint sweep/playback is still disabled.

## 11. Environment Notes

- Local preview:

  ```bash
  cd se2-navmesh.github.io
  python3 -m http.server 8020
  # open http://127.0.0.1:8020/tools/preview_harness.html?scene=00403
  ```

- `tools/bin/gltfpack` is built locally because upstream release binaries may
  require a newer glibc than Ubuntu 20.04.
- When running ROS export nodes as background tasks, tear down ROS children
  carefully; avoid broad `pkill -f` patterns that can match the shell command
  itself.
