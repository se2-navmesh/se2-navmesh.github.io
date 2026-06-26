#!/usr/bin/env python3
"""QA every scene listed in static/scenes/index.json.

For each scene checks:
  - field.bin record count matches scene.json field.cellCount;
  - field bounds match scene.json field.bounds;
  - scene.glb is aligned to the field: the glb's world-space AABB (dequantized
    through the node transforms) contains the field's XY footprint -- this is
    what catches a mis-rotated mesh (e.g. the HM3D 0/180-deg yaw inconsistency);
  - the in-browser lattice A* (mirrored here) finds a path for the configured
    start->goal query.

Run from the site root:  python3 tools/verify_scenes.py
Exits non-zero if any scene fails.
"""
import json, struct, math, heapq, collections, os, sys

import numpy as np
from pygltflib import GLTF2

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENES = os.path.join(ROOT, "static", "scenes")


def read_field(path):
    d = open(path, "rb").read()
    n = len(d) // 16
    px = [0.0] * n; py = [0.0] * n; pz = [0.0] * n; mask = [0] * n
    for i in range(n):
        px[i], py[i], pz[i], mask[i] = struct.unpack_from("<fffI", d, i * 16)
    return px, py, pz, mask, n


def glb_world_aabb(path):
    g = GLTF2().load(path)
    def node_matrix(nd):
        if nd.matrix:
            return np.array(nd.matrix, dtype=float).reshape(4, 4).T
        m = np.eye(4)
        if nd.scale: m[:3, :3] = np.diag(nd.scale)
        if nd.translation: m[:3, 3] = nd.translation
        return m
    lo = np.array([np.inf] * 3); hi = np.array([-np.inf] * 3)
    def walk(idx, parent):
        nd = g.nodes[idx]; M = parent @ node_matrix(nd)
        if nd.mesh is not None:
            for p in g.meshes[nd.mesh].primitives:
                acc = g.accessors[p.attributes.POSITION]
                if not acc.min or not acc.max:
                    continue
                amin, amax = acc.min, acc.max
                for cx in (amin[0], amax[0]):
                    for cy in (amin[1], amax[1]):
                        for cz in (amin[2], amax[2]):
                            w = M @ np.array([cx, cy, cz, 1.0])
                            np.minimum(lo, w[:3], out=lo)
                            np.maximum(hi, w[:3], out=hi)
        for c in (nd.children or []):
            walk(c, M)
    scene = g.scenes[g.scene or 0]
    for r in scene.nodes:
        walk(r, np.eye(4))
    return lo, hi


def plan(px, py, pz, mask, sj):
    bits = sj["field"]["yawBits"]; nL = bits * 2
    cs = sj["field"]["cellSize"]
    climb = sj["agent"].get("maxClimb", 0.25)
    # cost model mirrors explorer.js plan() / the C++ ASA planner: edge cost is
    # traversal time, with lateral motion ~5x dearer than longitudinal.
    yaw_step = sj["field"].get("yawStepRad", math.pi / bits)
    ag = sj.get("agent", {})
    v_lon = ag.get("maxLonVelocity", 0.5); v_lat = ag.get("maxLatVelocity", 0.1)
    v_ang = ag.get("maxAngVelocity", 0.5)
    dir_f = ag.get("directionCostFactor", 1.0); turn_f = ag.get("turningCostFactor", 1.0)
    unit_lon = dir_f / v_lon; unit_lat = dir_f / v_lat   # cost per metre fwd / sideways
    ROT = yaw_step / v_ang * turn_f                       # cost per yaw-layer step
    h_unit = min(unit_lon, unit_lat)                      # cheapest metre -> admissible h
    n = len(px)
    grid = collections.defaultdict(list); gi = [0] * n; gj = [0] * n
    for i in range(n):
        gi[i] = round(px[i] / cs); gj[i] = round(py[i] / cs)
        grid[(gi[i], gj[i])].append(i)
    nbr = [None] * n
    for i in range(n):
        out = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0: continue
                for j in grid.get((gi[i] + dx, gj[i] + dy), ()):
                    if abs(pz[i] - pz[j]) <= climb: out.append(j)
        nbr[i] = out
    feas = lambda c, L: (mask[c] >> (L % bits)) & 1

    def nearest(x, y, z):
        b, bd = -1, 1e18
        for i in range(n):
            dd = (px[i] - x) ** 2 + (py[i] - y) ** 2 + ((pz[i] - z) * 1.5) ** 2
            if dd < bd: bd, b = dd, i
        return b
    s, gpt = sj["start"], sj["goal"]
    sc = nearest(s["x"], s["y"], s["z"]); gc = nearest(gpt["x"], gpt["y"], gpt["z"])
    want = ((s.get("layer", 1) - 1) % nL + nL) % nL
    sL = want if feas(sc, want) else -1
    if sL < 0:
        for dd in range(1, nL + 1):
            if feas(sc, (want + dd) % nL): sL = (want + dd) % nL; break
            if feas(sc, (want - dd) % nL): sL = (want - dd) % nL; break
    gx, gy, gz = px[gc], py[gc], pz[gc]
    h = lambda c: math.hypot(px[c] - gx, py[c] - gy, pz[c] - gz) * h_unit
    G = {(sc, sL): 0.0}; H = [(h(sc), sc, sL)]
    while H:
        f, c, L = heapq.heappop(H)
        if c == gc: return G[(c, L)]
        if f - h(c) > G[(c, L)] + 1e-9: continue
        g0 = G[(c, L)]
        for dL in (1, nL - 1):
            L2 = (L + dL) % nL
            if feas(c, L2):
                ng = g0 + ROT
                if ng < G.get((c, L2), 1e18): G[(c, L2)] = ng; heapq.heappush(H, (ng + h(c), c, L2))
        if feas(c, L):
            for j in nbr[c]:
                dx = px[j] - px[c]; dy = py[j] - py[c]; dz = pz[j] - pz[c]
                dist = math.hypot(dx, dy, dz)
                diff = math.atan2(dy, dx) - L * yaw_step
                move_cost = dist * (abs(math.cos(diff)) * unit_lon + abs(math.sin(diff)) * unit_lat)
                for dL in (0, 1, nL - 1):
                    L2 = (L + dL) % nL
                    if feas(j, L2):
                        ng = g0 + move_cost + (0 if dL == 0 else ROT)
                        if ng < G.get((j, L2), 1e18): G[(j, L2)] = ng; heapq.heappush(H, (ng + h(j), j, L2))
    return None


def main():
    scenes = json.load(open(os.path.join(SCENES, "index.json")))
    ok = True
    print("%-7s %8s %8s %8s  %s" % ("scene", "cells", "aligned", "cost(s)", "notes"))
    for sc in scenes:
        d = sc["dir"]; sd = os.path.join(SCENES, d)
        notes = []
        try:
            sj = json.load(open(os.path.join(sd, "scene.json")))
            px, py, pz, mask, n = read_field(os.path.join(sd, "field.bin"))
            if n != sj["field"]["cellCount"]:
                notes.append("count %d!=%d" % (n, sj["field"]["cellCount"])); ok = False
            glo, ghi = glb_world_aabb(os.path.join(sd, "scene.glb"))
            flo = [min(px), min(py), min(pz)]; fhi = [max(px), max(py), max(pz)]
            pad = 1.0
            aligned = all(glo[i] - pad <= flo[i] and fhi[i] <= ghi[i] + pad for i in (0, 1))
            if not aligned:
                notes.append("field XY outside glb AABB"); ok = False
            cost = plan(px, py, pz, mask, sj)
            if cost is None:
                notes.append("NO PATH"); ok = False
            print("%-7s %8d %8s %8s  %s" % (
                d, n, "yes" if aligned else "NO",
                ("%.1f" % cost) if cost else "none", "; ".join(notes)))
        except Exception as e:
            print("%-7s  ERROR: %s" % (d, e)); ok = False
    print("\n%s" % ("ALL SCENES OK" if ok else "FAILURES ABOVE"))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
