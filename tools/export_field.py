#!/usr/bin/env python3
"""Scrape a running se2_navmesh_static node into web assets.

Subscribes to the latched topics published by tools/web_export_field.launch and
writes, into <scene_dir>:

  field.bin   little-endian records [float32 x, y, z, uint32 mask] per surface
              cell; `mask` low bits are the feasible yaw layers over [0, pi)
              (the footprint is 180-deg symmetric, so layer L and L+bits share
              feasibility). This is a span-level SE(2) traversability field
              used by the Explorer as an explanatory safe/restricted overlay.
              It is not the final Detour polygon graph and is not the active
              route-planning substrate; browser planning should use
              polyfield.bin/polyfield.json.
  scene.json  metadata: agent, yaw layers, bounds, and configured start/goal.

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

NODE = "/se2_navmesh_static_web_export"


def get(name, default=None):
    return rospy.get_param(NODE + "/" + name, default)


def wait(topic, typ, timeout):
    rospy.loginfo("waiting for %s ...", topic)
    return rospy.wait_for_message(topic, typ, timeout=timeout)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene-dir", required=True)
    ap.add_argument("--timeout", type=float, default=600.0)
    args = ap.parse_args()

    rospy.init_node("se2_web_field_exporter", anonymous=True)

    span_area = wait(NODE + "/navigation_mesh_span_area", MarkerArray, args.timeout)
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
    }
    with open(args.scene_dir + "/scene.json", "w") as f:
        json.dump(scene, f, indent=2)

    rospy.loginfo("exported %d cells -> %s", count, args.scene_dir)
    print("FIELD_EXPORT_OK cells=%d" % count)


if __name__ == "__main__":
    main()
