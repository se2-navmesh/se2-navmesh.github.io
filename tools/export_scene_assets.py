#!/usr/bin/env python3
"""Export web viewer assets from a running SE(2) NavMesh ROS node.

The exporter listens to the latched ROS topics published by
tools/web_export_00114.launch, converts the planner path and navmesh marker to
compact JSON, and builds a local OBJ crop for the browser preview. If Python
Open3D is installed it is used for OBJ loading; otherwise a streaming OBJ parser
keeps this script usable on the standard ROS workstation.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

import yaml


ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = ROOT.parent
SCENE_DIR = ROOT / "static" / "scenes" / "00114-hard"
OBJ_PATH = WORKSPACE_SRC / "mesh" / "00114-Coer9RdivP7.obj"
START_GOAL_YAML = (
    WORKSPACE_SRC
    / "novo_3d"
    / "deps"
    / "se2_navmesh"
    / "se2_navmesh_ros_testing"
    / "config"
    / "00114-Coer9RdivP7_start_goal_hard.yaml"
)

YAW_LAYERS = 40
YAW_STEP = math.pi / 20.0


Point = Tuple[float, float, float]
Triangle = Tuple[Point, Point, Point]


def rounded(value: float) -> float:
    return round(float(value), 6)


def point_dict(point: Point, z_offset: float = 0.0) -> dict:
    return {
        "x": rounded(point[0]),
        "y": rounded(point[1]),
        "z": rounded(point[2] + z_offset),
    }


def layer_from_yaw(yaw: float) -> int:
    layer = int(round((yaw % (2.0 * math.pi)) / YAW_STEP)) + 1
    return max(1, min(YAW_LAYERS, layer))


def bounds_for_points(points: Sequence[Point], margin_xy: float, margin_z: float) -> Tuple[Point, Point]:
    mins = [min(p[i] for p in points) for i in range(3)]
    maxs = [max(p[i] for p in points) for i in range(3)]
    return (
        (mins[0] - margin_xy, mins[1] - margin_xy, mins[2] - margin_z),
        (maxs[0] + margin_xy, maxs[1] + margin_xy, maxs[2] + margin_z),
    )


def in_bounds(point: Point, bounds: Tuple[Point, Point]) -> bool:
    lo, hi = bounds
    return all(lo[i] <= point[i] <= hi[i] for i in range(3))


def triangle_centroid(tri: Triangle) -> Point:
    return (
        (tri[0][0] + tri[1][0] + tri[2][0]) / 3.0,
        (tri[0][1] + tri[1][1] + tri[2][1]) / 3.0,
        (tri[0][2] + tri[1][2] + tri[2][2]) / 3.0,
    )


def triangle_area_xy(tri: Triangle) -> float:
    ax, ay, _ = tri[0]
    bx, by, _ = tri[1]
    cx, cy, _ = tri[2]
    return abs((bx - ax) * (cy - ay) - (cx - ax) * (by - ay)) * 0.5


def triangle_area_3d(tri: Triangle) -> float:
    ax, ay, az = tri[0]
    bx, by, bz = tri[1]
    cx, cy, cz = tri[2]
    ux, uy, uz = bx - ax, by - ay, bz - az
    vx, vy, vz = cx - ax, cy - ay, cz - az
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    return 0.5 * math.sqrt(nx * nx + ny * ny + nz * nz)


def triangle_abs_normal_z(tri: Triangle) -> float:
    ax, ay, az = tri[0]
    bx, by, bz = tri[1]
    cx, cy, cz = tri[2]
    ux, uy, uz = bx - ax, by - ay, bz - az
    vx, vy, vz = cx - ax, cy - ay, cz - az
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    length = math.sqrt(nx * nx + ny * ny + nz * nz)
    if length <= 1e-9:
        return 0.0
    return abs(nz) / length


def closest_segment_distance(point: Point, path: Sequence[Point]) -> Tuple[float, float]:
    best_xy = float("inf")
    best_z_delta = float("inf")
    px, py, pz = point
    for i in range(len(path) - 1):
        ax, ay, az = path[i]
        bx, by, bz = path[i + 1]
        dx = bx - ax
        dy = by - ay
        denom = dx * dx + dy * dy
        t = 0.0 if denom <= 1e-9 else ((px - ax) * dx + (py - ay) * dy) / denom
        t = max(0.0, min(1.0, t))
        qx = ax + dx * t
        qy = ay + dy * t
        qz = az + (bz - az) * t
        dist_xy = math.hypot(px - qx, py - qy)
        if dist_xy < best_xy:
            best_xy = dist_xy
            best_z_delta = abs(pz - qz)
    return best_xy, best_z_delta


def filter_planning_surface(
    triangles: Sequence[Triangle],
    path: Sequence[Point],
    max_area: float,
    min_abs_normal_z: float,
    corridor_xy: float,
    corridor_z: float,
) -> List[Triangle]:
    filtered = []
    for tri in triangles:
        if triangle_area_3d(tri) > max_area:
            continue
        if triangle_abs_normal_z(tri) < min_abs_normal_z:
            continue
        dist_xy, dist_z = closest_segment_distance(triangle_centroid(tri), path)
        if dist_xy > corridor_xy or dist_z > corridor_z:
            continue
        filtered.append(tri)
    return filtered


def triangle_bounds(triangles: Sequence[Triangle]) -> Tuple[Point, Point]:
    points = [point for tri in triangles for point in tri]
    return (
        tuple(min(point[i] for point in points) for i in range(3)),
        tuple(max(point[i] for point in points) for i in range(3)),
    )


def load_obj_triangles_with_open3d(obj_path: Path, bounds: Tuple[Point, Point]) -> List[Triangle] | None:
    try:
        import open3d as o3d  # type: ignore
    except ImportError:
        return None

    mesh = o3d.io.read_triangle_mesh(str(obj_path))
    vertices = [(float(v[0]), float(v[1]), float(v[2])) for v in mesh.vertices]
    triangles = []
    for face in mesh.triangles:
        tri = (vertices[int(face[0])], vertices[int(face[1])], vertices[int(face[2])])
        if in_bounds(triangle_centroid(tri), bounds):
            triangles.append(tri)
    return triangles


def parse_face_index(token: str, vertex_count: int) -> int:
    raw = token.split("/")[0]
    index = int(raw)
    if index < 0:
        return vertex_count + index
    return index - 1


def load_obj_triangles_streaming(obj_path: Path, bounds: Tuple[Point, Point]) -> List[Triangle]:
    vertices: List[Point] = []
    triangles: List[Triangle] = []

    with obj_path.open("r", encoding="utf-8", errors="ignore") as file:
        for line in file:
            if line.startswith("v "):
                parts = line.split()
                vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
            elif line.startswith("f "):
                refs = [parse_face_index(part, len(vertices)) for part in line.split()[1:]]
                if len(refs) < 3:
                    continue
                base = vertices[refs[0]]
                for i in range(1, len(refs) - 1):
                    tri = (base, vertices[refs[i]], vertices[refs[i + 1]])
                    if in_bounds(triangle_centroid(tri), bounds):
                        triangles.append(tri)

    return triangles


def decimate(triangles: Sequence[Triangle], max_count: int) -> List[Triangle]:
    if len(triangles) <= max_count:
        return list(triangles)
    stride = math.ceil(len(triangles) / max_count)
    return list(triangles[::stride])[:max_count]


def flat_positions(triangles: Iterable[Triangle]) -> List[float]:
    values: List[float] = []
    for tri in triangles:
        for point in tri:
            values.extend([rounded(point[0]), rounded(point[1]), rounded(point[2])])
    return values


def marker_triangles(
    marker,
    bounds: Optional[Tuple[Point, Point]] = None,
    z_offset: float = 0.035,
    kind: str = "full",
) -> List[dict]:
    polygons = []
    for i in range(0, len(marker.points) - 2, 3):
        tri = tuple((marker.points[i + j].x, marker.points[i + j].y, marker.points[i + j].z) for j in range(3))
        if bounds is not None and not in_bounds(triangle_centroid(tri), bounds):
            continue
        if triangle_area_xy(tri) < 1e-5:
            continue
        polygons.append(
            {
                "id": len(polygons) + 1,
                "kind": kind,
                "vertices": [point_dict(p, z_offset) for p in tri],
            }
        )
    return polygons


def trim_polygons(polygons: Sequence[dict], max_count: int) -> List[dict]:
    if len(polygons) <= max_count:
        trimmed = list(polygons)
    else:
        stride = math.ceil(len(polygons) / max_count)
        trimmed = list(polygons[::stride])[:max_count]
    for index, polygon in enumerate(trimmed, start=1):
        polygon["id"] = index
    return trimmed


def mission_points(message) -> List[dict]:
    points = []
    last = None
    for waypoint in message.points:
        current = (
            rounded(waypoint.x),
            rounded(waypoint.y),
            rounded(waypoint.z),
            layer_from_yaw(float(waypoint.yaw)),
        )
        if current == last:
            continue
        points.append({"x": current[0], "y": current[1], "z": current[2], "layer": current[3]})
        last = current
    return points


def write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def build_mesh_preview(args: argparse.Namespace, path_points: Sequence[dict]) -> Tuple[dict, int, str]:
    path_xyz = [(p["x"], p["y"], p["z"]) for p in path_points]
    mesh_crop_bounds = bounds_for_points(path_xyz, margin_xy=args.mesh_margin_xy, margin_z=args.mesh_margin_z)

    mesh_triangles = load_obj_triangles_with_open3d(OBJ_PATH, mesh_crop_bounds)
    mesh_loader = "open3d"
    if mesh_triangles is None:
        mesh_triangles = load_obj_triangles_streaming(OBJ_PATH, mesh_crop_bounds)
        mesh_loader = "streaming_obj_parser"

    mesh_triangles = filter_planning_surface(
        mesh_triangles,
        path_xyz,
        max_area=args.max_mesh_area,
        min_abs_normal_z=args.min_mesh_abs_normal_z,
        corridor_xy=args.mesh_corridor_xy,
        corridor_z=args.mesh_corridor_z,
    )
    mesh_triangles = decimate(mesh_triangles, args.max_mesh_triangles)
    if not mesh_triangles:
        raise RuntimeError("Mesh surface filtering produced no triangles.")

    mesh_bounds = triangle_bounds(mesh_triangles)
    data = {
        "source": str(OBJ_PATH.relative_to(WORKSPACE_SRC)),
        "loader": mesh_loader,
        "sourceFaceCount": 432822,
        "triangleCount": len(mesh_triangles),
        "filter": (
            "hard-query planning-surface crop: upward triangles near the "
            "exported SE2 path corridor"
        ),
        "bounds": {
            "min": [rounded(v) for v in mesh_bounds[0]],
            "max": [rounded(v) for v in mesh_bounds[1]],
        },
        "positions": flat_positions(mesh_triangles),
    }
    return data, len(mesh_triangles), mesh_loader


def export_mesh_from_existing_scene(args: argparse.Namespace) -> None:
    scene_path = SCENE_DIR / "scene.json"
    scene = json.loads(scene_path.read_text(encoding="utf-8"))
    path_points = scene["paths"].get("plannerQuery") or scene["paths"].get("configuredQueryDraft")
    if not path_points:
        raise RuntimeError("Existing scene has no plannerQuery path to build a mesh crop around.")

    mesh_data, triangle_count, mesh_loader = build_mesh_preview(args, path_points)
    write_json(SCENE_DIR / "local_planning_surface.json", mesh_data)

    notes = scene.get("notes", [])
    if notes:
        notes[-1] = (
            f"The mesh preview was generated with {mesh_loader} using an upward-surface path-corridor filter; "
            "Python Open3D will be used automatically when installed."
        )
        scene["notes"] = notes
        write_json(scene_path, scene)

    print(f"Wrote {triangle_count} mesh preview triangles")


def export_from_ros(args: argparse.Namespace) -> None:
    import rospy
    from se2_navmesh_msgs.msg import SE2Path
    from visualization_msgs.msg import Marker

    rospy.init_node("se2_web_asset_exporter", anonymous=True)
    mission = rospy.wait_for_message(args.mission_topic, SE2Path, timeout=args.timeout)
    navmesh_marker = rospy.wait_for_message(args.navmesh_topic, Marker, timeout=args.timeout)
    path_navmesh_marker = rospy.wait_for_message(args.path_navmesh_topic, Marker, timeout=args.timeout)

    path_points = mission_points(mission)
    if len(path_points) < 2:
        raise RuntimeError("Planner mission topic did not contain a usable path.")

    mesh_data, mesh_triangle_count, mesh_loader = build_mesh_preview(args, path_points)

    full_navmesh_polygons = trim_polygons(
        marker_triangles(navmesh_marker, kind="full"),
        args.max_full_navmesh_triangles,
    )
    path_navmesh_polygons = trim_polygons(
        marker_triangles(path_navmesh_marker, z_offset=0.065, kind="pathUsed"),
        args.max_path_navmesh_triangles,
    )
    navmesh_polygons = full_navmesh_polygons + path_navmesh_polygons

    starts = path_points[0]
    goals = path_points[-1]
    with START_GOAL_YAML.open("r", encoding="utf-8") as file:
        query_config = yaml.safe_load(file)

    write_json(SCENE_DIR / "local_planning_surface.json", mesh_data)

    write_json(
        SCENE_DIR / "navmesh_overlay.json",
        {
            "source": args.navmesh_topic,
            "sourceType": "visualization_msgs/Marker TRIANGLE_LIST from se2_navmesh_static",
            "pathSource": args.path_navmesh_topic,
            "pathSourceType": "visualization_msgs/Marker TRIANGLE_LIST from recast_path_polygons",
            "filter": "full navmesh marker plus planner searched/path polygons",
            "triangleCount": len(navmesh_polygons),
            "fullTriangleCount": len(full_navmesh_polygons),
            "pathTriangleCount": len(path_navmesh_polygons),
            "polygons": navmesh_polygons,
        },
    )

    write_json(
        SCENE_DIR / "scene.json",
        {
            "id": "00114-hard",
            "name": "00114-Coer9RdivP7, hard query",
            "meshPreview": "local_planning_surface.json",
            "navmeshOverlay": "navmesh_overlay.json",
            "sourceMesh": "../../../../../mesh/00114-Coer9RdivP7.obj",
            "yawLayers": YAW_LAYERS,
            "yawLayerStepRad": YAW_STEP,
            "agent": {
                "length": 0.93,
                "width": 0.53,
                "height": 0.89,
                "maxClimb": 0.25,
                "maxSlopeDeg": 30,
            },
            "start": starts,
            "goal": goals,
            "paths": {
                "plannerQuery": path_points,
            },
            "queryConfig": {
                "start": {
                    "x": query_config["start_x"],
                    "y": query_config["start_y"],
                    "z": query_config["start_z"],
                    "layer": query_config["start_layer"],
                },
                "goal": {
                    "x": query_config["goal_x"],
                    "y": query_config["goal_y"],
                    "z": query_config["goal_z"],
                    "layer": query_config["goal_layer"],
                },
            },
            "notes": [
                "Start and goal are loaded from se2_navmesh_ros_testing/config/00114-Coer9RdivP7_start_goal_hard.yaml.",
                "The visible route is exported from the SE2 NavMesh planner mission topic, not hand drawn.",
                "The navmesh overlay shows the full se2_navmesh_static TRIANGLE_LIST marker in RViz light blue and the planner-used recast_path_polygons marker in yellow.",
                f"The mesh preview was generated with {mesh_loader} using an upward-surface path-corridor filter; Python Open3D will be used automatically when installed.",
            ],
        },
    )

    print(f"Wrote {SCENE_DIR / 'scene.json'}")
    print(f"Wrote {len(path_points)} path waypoints")
    print(f"Wrote {mesh_triangle_count} mesh preview triangles")
    print(
        "Wrote "
        f"{len(full_navmesh_polygons)} full navmesh triangles and "
        f"{len(path_navmesh_polygons)} path navmesh triangles"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mission-topic", default="/se2navmesh_mission")
    parser.add_argument("--navmesh-topic", default="/se2_navmesh_static_web_export/navigation_mesh")
    parser.add_argument("--path-navmesh-topic", default="/se2_navmesh_static_web_export/recast_path_polygons")
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--mesh-margin-xy", type=float, default=1.3)
    parser.add_argument("--mesh-margin-z", type=float, default=0.7)
    parser.add_argument("--mesh-corridor-xy", type=float, default=0.95)
    parser.add_argument("--mesh-corridor-z", type=float, default=0.32)
    parser.add_argument("--min-mesh-abs-normal-z", type=float, default=0.58)
    parser.add_argument("--max-mesh-area", type=float, default=0.08)
    parser.add_argument("--max-mesh-triangles", type=int, default=22000)
    parser.add_argument("--max-full-navmesh-triangles", type=int, default=14000)
    parser.add_argument("--max-path-navmesh-triangles", type=int, default=10000)
    parser.add_argument("--mesh-only", action="store_true")
    args = parser.parse_args()
    if args.mesh_only:
        export_mesh_from_existing_scene(args)
    else:
        export_from_ros(args)


if __name__ == "__main__":
    main()
