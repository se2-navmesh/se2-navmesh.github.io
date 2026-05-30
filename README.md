# SE(2) NavMesh Project Page

Anonymous static project page prototype for `se2-navmesh.github.io`.

Everything needed for the website is inside this folder. It can be uploaded directly
to a GitHub Pages repository.

The first interactive scene uses:

- `mesh/00114-Coer9RdivP7.obj` as the raw source mesh.
- `se2_navmesh_ros_testing/config/00114-Coer9RdivP7_start_goal_hard.yaml` for start and goal.
- A local hard-query planning-surface crop at `static/scenes/00114-hard/local_planning_surface.json`.
- A ROS-exported SE(2) navmesh overlay at `static/scenes/00114-hard/navmesh_overlay.json`,
  with the full navmesh in RViz light blue and planner-used search polygons in yellow.
- A planner-exported ASA route in `paths.plannerQuery` in `static/scenes/00114-hard/scene.json`.

The assets can be regenerated from a running SE2 navmesh ROS node:

```bash
roslaunch tools/web_export_00114.launch
python3 tools/export_scene_assets.py
```

`tools/export_scene_assets.py` uses Python Open3D for OBJ loading when it is installed,
and otherwise falls back to a streaming OBJ parser.

## Interactive components

Static interactive widgets live in `static/js/interactives.js` (no build step, vanilla JS):

- **Video segmented control** — Overview / Real-World Navigation / Online Generation tabs.
- **NavMesh vs. SE(2) comparison slider** — wipes between `static/images/compare_navmesh.jpg`
  and `static/images/compare_se2.jpg` (aligned crops of the *Store* scene).
- **Yaw-feasibility demo** — rotate an ANYmal footprint in an adjustable-width passage to see
  which headings fit; the dial counts feasible yaw channels (safe / restricted / inaccessible).
- **ASA stepper** — steps/auto-plays through A* → string pulling → yaw refinement.
- **SPC benchmark chart** — Chart.js plot of ASA vs. sampling-based planners.

MathJax and Chart.js are loaded from CDN; everything else is local.

## Videos

The Videos section and the hero "Video" button expect three MP4s (posters are shown until
the files exist):

| Tab                  | Drop file at                          |
| -------------------- | ------------------------------------- |
| Overview             | `static/videos/overview.mp4`          |
| Real-World Navigation| `static/videos/real_world.mp4`        |
| Online Generation    | `static/videos/online_generation.mp4` |

After adding a file, optionally make it autoplay by adding `autoplay muted loop` to the
matching `<video>` tag in `index.html`.

Most result figures are derived from the paper sources in `../SE2/Images/` (JPGs copied and
PDFs rasterized with `pdftoppm` / `convert`).

## Local Preview

From this folder:

```bash
python3 -m http.server 8020
```

Then open `http://127.0.0.1:8020/`.
