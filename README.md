# SE(2) NavMesh Project Page

Anonymous static project page prototype for `se2-navmesh.github.io`.

Everything needed for the website is inside this folder. It can be uploaded directly
to a GitHub Pages repository.

The first interactive scene uses:

- `mesh/00114-Coer9RdivP7.obj` as the raw source mesh.
- `se2_navmesh_ros_testing/config/00114-Coer9RdivP7_start_goal_hard.yaml` for start and goal.
- A local hard-query planning-surface crop at `static/scenes/00114-hard/local_planning_surface.json`.
- A cropped ROS-exported SE(2) navmesh overlay at `static/scenes/00114-hard/navmesh_overlay.json`.
- A planner-exported ASA route in `paths.plannerQuery` in `static/scenes/00114-hard/scene.json`.

The assets can be regenerated from a running SE2 navmesh ROS node:

```bash
roslaunch tools/web_export_00114.launch
python3 tools/export_scene_assets.py
```

`tools/export_scene_assets.py` uses Python Open3D for OBJ loading when it is installed,
and otherwise falls back to a streaming OBJ parser.

## Local Preview

From this folder:

```bash
python3 -m http.server 8020
```

Then open `http://127.0.0.1:8020/`.
