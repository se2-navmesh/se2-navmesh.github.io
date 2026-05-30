# SE(2) NavMesh Project Page

Anonymous static project page prototype for `se2-navmesh.github.io`.

Everything needed for the website is inside this folder. It can be uploaded directly
to a GitHub Pages repository.

The first interactive scene uses:

- `mesh/00114-Coer9RdivP7.obj` as the raw source mesh.
- `se2_navmesh_ros_testing/config/00114-Coer9RdivP7_start_goal_hard.yaml` for start and goal.
- A local planning-surface preview at `static/scenes/00114-hard/local_planning_surface.json`.
- A ROS-style navmesh overlay scaffold at `static/scenes/00114-hard/navmesh_overlay.json`.

The displayed route is currently a visualization scaffold. Replace `paths.configuredQueryDraft`
in `static/scenes/00114-hard/scene.json` with planner-exported ASA/path data once available.

## Local Preview

From this folder:

```bash
python3 -m http.server 8020
```

Then open `http://127.0.0.1:8020/`.
