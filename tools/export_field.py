#!/usr/bin/env python3
"""Scrape a running se2_navmesh_static node into web assets.

Subscribes to the latched topics published by tools/web_export_field.launch and
writes, into <scene_dir>:

  field.bin   little-endian records [float32 x, y, z, uint32 mask] per surface
              cell; `mask` low bits are the feasible yaw layers over [0, pi)
              (the footprint is 180-deg symmetric, so layer L and L+bits share
              feasibility).  This is the SE(2) traversability field used for
              both the yaw slider and live planning.
  scene.json  metadata: agent, yaw layers, bounds, configured start/goal, and a
              reference ASA path (continuous yaw) for sanity/seed.

Run (after the node is up; topics are latched so order does not matter):
  rosrun is not needed -- just: python3 tools/export_field.py --scene-dir <dir>
"""
import argparse
import json
import math
import struct
import sys

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from se2_navmesh_msgs.msg import SE2Path

NODE = "/se2_navmesh_static_web_export"


def get(name, default=None):
    return rospy.get_param(NODE + "/" + name, default)


def wait(topic, typ, timeout):
    rospy.loginfo("waiting for %s ...", topic)
    return rospy.wait_for_message(topic, typ, timeout=timeout)


def build_path(points, with_yaw=True, with_action=False):
    """Deduplicate consecutive waypoints into a list of plain dicts.

    `with_yaw` keeps the heading (initial A* / second A* stages); the string
    pulling stage has no orientation so it is dropped. `with_action` keeps the
    SE2 action-type code (only meaningful for the published mission path)."""
    path = []
    last = None
    for w in points:
        key = (round(w.x, 4), round(w.y, 4), round(w.z, 4))
        if with_yaw:
            key = key + (round(w.yaw, 5),)
        if key == last:
            continue
        last = key
        rec = {"x": key[0], "y": key[1], "z": key[2]}
        if with_yaw:
            rec["yaw"] = key[3]
        if with_action:
            rec["action"] = int(w.action_type)
        path.append(rec)
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene-dir", required=True)
    ap.add_argument("--timeout", type=float, default=600.0)
    args = ap.parse_args()

    rospy.init_node("se2_web_field_exporter", anonymous=True)

    span_area = wait(NODE + "/navigation_mesh_span_area", MarkerArray, args.timeout)
    mission = wait("/se2navmesh_mission", SE2Path, args.timeout)
    raw_mission = wait("/se2navmesh_raw_path", SE2Path, args.timeout)
    straight_mission = wait("/se2navmesh_straight_path", SE2Path, args.timeout)

    bits = int(get("num_yaw_layers_pi", 20))
    layer_mask = (1 << bits) - 1
    cell_size = float(get("cell_size", 0.1))
    cell_height = float(get("cell_height", 0.1))
    yaw_step = math.pi / bits

    # --- feasibility field cells ---
    cells = bytearray()
    count = 0
    lo = [1e9, 1e9, 1e9]
    hi = [-1e9, -1e9, -1e9]
    for m in span_area.markers:
        if m.type != Marker.TEXT_VIEW_FACING or not m.text:
            continue
        try:
            mask = int(m.text, 2) & layer_mask
        except ValueError:
            continue
        if mask == 0:
            continue
        x = m.pose.position.x
        y = m.pose.position.y
        z = m.pose.position.z - 0.5 * cell_height   # undo display offset -> span top
        cells += struct.pack("<fffI", x, y, z, mask)
        count += 1
        lo = [min(lo[0], x), min(lo[1], y), min(lo[2], z)]
        hi = [max(hi[0], x), max(hi[1], y), max(hi[2], z)]

    if count == 0:
        rospy.logerr("no feasible span cells scraped -- is display_span_area true?")
        sys.exit(1)

    with open(args.scene_dir + "/field.bin", "wb") as f:
        f.write(cells)

    # --- ASA path stages ---
    # Stage 3 (second A*, with yaw + action codes) is also the reference path.
    reference_path = build_path(mission.points, with_yaw=True, with_action=True)
    asa_stages = {
        "initialAstar": build_path(raw_mission.points, with_yaw=True),   # stage 1: with orientation
        "stringPull": build_path(straight_mission.points, with_yaw=False),  # stage 2: no orientation
        "secondAstar": reference_path,                                   # stage 3: with orientation
    }

    scene = {
        "id": get("__scene_id", "scene"),
        "field": {
            "file": "field.bin",
            "record": "f32 x,y,z; u32 mask",
            "cellCount": count,
            "cellSize": cell_size,
            "yawBits": bits,           # feasibility layers over [0, pi)
            "yawStepRad": yaw_step,
            "bounds": {"min": [round(v, 3) for v in lo],
                       "max": [round(v, 3) for v in hi]},
        },
        "agent": {
            "length": float(get("agent_length", 0.93)),
            "width": float(get("agent_width", 0.53)),
            "height": float(get("agent_height", 0.89)),
            "maxClimb": float(get("agent_max_climb", 0.25)),
            "maxSlopeDeg": float(get("agent_max_slope", 30)),
        },
        "start": {"x": float(get("start_x")), "y": float(get("start_y")),
                  "z": float(get("start_z")), "layer": int(get("start_layer", 1))},
        "goal": {"x": float(get("goal_x")), "y": float(get("goal_y")),
                 "z": float(get("goal_z")), "layer": int(get("goal_layer", 1))},
        "referencePath": reference_path,
        "asaStages": asa_stages,
    }
    with open(args.scene_dir + "/scene.json", "w") as f:
        json.dump(scene, f, indent=2)

    rospy.loginfo("exported %d cells; ASA stages: initial=%d, string=%d, second=%d -> %s",
                  count, len(asa_stages["initialAstar"]), len(asa_stages["stringPull"]),
                  len(asa_stages["secondAstar"]), args.scene_dir)
    print("FIELD_EXPORT_OK cells=%d initial=%d string=%d second=%d" % (
        count, len(asa_stages["initialAstar"]), len(asa_stages["stringPull"]),
        len(asa_stages["secondAstar"])))


if __name__ == "__main__":
    main()
