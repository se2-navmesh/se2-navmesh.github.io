"""Blender headless: turn an HM3D .glb into a web-ready, navmesh-aligned .glb.

  blender -b --python tools/blender_export_scene.py -- \
      --in  <src.glb> --out <dst.glb> \
      [--tris 250000] [--tex 2048] [--draco 6] \
      [--crop xmin ymin zmin xmax ymax zmax]   # in navmesh (output) frame

Pipeline:
  1. import glTF (importer yields a Blender Z-up frame),
  2. apply the fixed HM3D->navmesh rotation  (x,y,z) -> (-x,-z,-y),
  3. optional crop to a navmesh-frame AABB (bisect + delete outside),
  4. decimate to a target triangle budget,
  5. downscale textures to a max edge,
  6. export GLB in navmesh Z-up frame (export_yup=False) with Draco.

The HM3D->navmesh rotation is constant across scenes (all OBJs in mesh/ were
produced from these GLBs the same way); it was derived by matching vertex
bounds and is asserted in tools/blender_probe_bounds.py.
"""
import sys
import bpy
import bmesh
from mathutils import Matrix, Vector


def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    a = {"tris": 250000, "tex": 2048, "draco": 6, "crop": None}
    i = 0
    while i < len(argv):
        k = argv[i]
        if k == "--in":
            a["in"] = argv[i + 1]; i += 2
        elif k == "--out":
            a["out"] = argv[i + 1]; i += 2
        elif k == "--tris":
            a["tris"] = int(argv[i + 1]); i += 2
        elif k == "--tex":
            a["tex"] = int(argv[i + 1]); i += 2
        elif k == "--draco":
            a["draco"] = int(argv[i + 1]); i += 2
        elif k == "--obj":
            a["obj"] = argv[i + 1]; i += 2
        elif k == "--crop":
            a["crop"] = [float(x) for x in argv[i + 1:i + 7]]; i += 7
        else:
            i += 1
    return a


def obj_bounds(path):
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    with open(path) as f:
        for line in f:
            if line.startswith("v "):
                t = line.split()
                for i in range(3):
                    v = float(t[i + 1]); lo[i] = min(lo[i], v); hi[i] = max(hi[i], v)
    return lo, hi


def aabb_after_rz(lo, hi, deg):
    import math
    c = round(math.cos(math.radians(deg))); s = round(math.sin(math.radians(deg)))
    nlo = [float("inf")] * 2; nhi = [float("-inf")] * 2
    for X in (lo[0], hi[0]):
        for Y in (lo[1], hi[1]):
            x = c * X - s * Y; y = s * X + c * Y
            nlo[0] = min(nlo[0], x); nhi[0] = max(nhi[0], x)
            nlo[1] = min(nlo[1], y); nhi[1] = max(nhi[1], y)
    return ([nlo[0], nlo[1], lo[2]], [nhi[0], nhi[1], hi[2]])


def align_yaw_to_obj(obj_path):
    """The HM3D->OBJ export adds an inconsistent 0/180-deg yaw across scenes;
    pick the Z-rotation whose AABB best matches the navmesh OBJ and apply it."""
    olo, ohi = obj_bounds(obj_path)
    lo, hi = world_bounds()
    best_deg, best_err = 0, float("inf")
    for deg in (0, 90, 180, 270):
        rlo, rhi = aabb_after_rz(lo, hi, deg)
        err = sum(abs(rlo[i] - olo[i]) + abs(rhi[i] - ohi[i]) for i in range(3))
        if err < best_err:
            best_err, best_deg = err, deg
    if best_deg:
        apply_to_roots(Matrix.Rotation(__import__("math").radians(best_deg), 4, "Z"))
    print("ALIGN_YAW deg=%d err=%.3f" % (best_deg, best_err))


# (x, y, z) -> (-x, -z, -y) ; a proper rotation (det = +1), no translation/scale.
HM3D_TO_NAVMESH = Matrix((
    (-1.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, -1.0, 0.0),
    (0.0, -1.0, 0.0, 0.0),
    (0.0, 0.0, 0.0, 1.0),
))


def mesh_objects():
    return [o for o in bpy.data.objects if o.type == "MESH"]


def world_bounds():
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    for o in mesh_objects():
        mw = o.matrix_world
        for v in o.data.vertices:
            w = mw @ v.co
            for i in range(3):
                lo[i] = min(lo[i], w[i]); hi[i] = max(hi[i], w[i])
    return lo, hi


def tri_count():
    return sum(sum(len(p.vertices) - 2 for p in o.data.polygons) for o in mesh_objects())


def apply_to_roots(matrix):
    for o in bpy.data.objects:
        if o.parent is None:
            o.matrix_world = matrix @ o.matrix_world


def crop_to_aabb(box):
    lo = Vector(box[:3]); hi = Vector(box[3:])
    for o in mesh_objects():
        mw = o.matrix_world
        bm = bmesh.new(); bm.from_mesh(o.data)
        kill = [v for v in bm.verts
                if not (lo.x <= (mw @ v.co).x <= hi.x
                        and lo.y <= (mw @ v.co).y <= hi.y
                        and lo.z <= (mw @ v.co).z <= hi.z)]
        bmesh.ops.delete(bm, geom=kill, context="VERTS")
        bm.to_mesh(o.data); bm.free()
        o.data.update()
    # drop now-empty objects
    for o in list(mesh_objects()):
        if len(o.data.vertices) == 0:
            bpy.data.objects.remove(o, do_unlink=True)


def decimate(target_tris):
    cur = tri_count()
    if cur <= target_tris:
        return cur
    ratio = max(0.02, min(1.0, target_tris / float(cur)))
    for o in mesh_objects():
        if len(o.data.polygons) <= 3:        # decimate needs >3 faces
            continue
        m = o.modifiers.new("dec", "DECIMATE")
        m.decimate_type = "COLLAPSE"
        m.ratio = ratio
    return ratio


def downscale_textures(max_edge):
    for img in bpy.data.images:
        if not img.has_data or img.size[0] == 0:
            continue
        w, h = img.size
        m = max(w, h)
        if m <= max_edge:
            continue
        s = max_edge / float(m)
        img.scale(max(1, int(w * s)), max(1, int(h * s)))
        img.pack()


def main():
    a = parse_args()
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.gltf(filepath=a["in"])

    apply_to_roots(HM3D_TO_NAVMESH)
    if a.get("obj"):
        align_yaw_to_obj(a["obj"])
    lo, hi = world_bounds()
    print("ALIGNED min=[%.3f, %.3f, %.3f] max=[%.3f, %.3f, %.3f]" % (*lo, *hi))

    if a["crop"]:
        crop_to_aabb(a["crop"])
        lo, hi = world_bounds()
        print("CROPPED min=[%.3f, %.3f, %.3f] max=[%.3f, %.3f, %.3f]" % (*lo, *hi))

    before = tri_count()
    dec = decimate(a["tris"])
    downscale_textures(a["tex"])

    bpy.ops.export_scene.gltf(
        filepath=a["out"],
        export_format="GLB",
        export_yup=False,                 # keep navmesh Z-up frame
        export_apply=True,                # bake the decimate modifier
        export_materials=True,
        export_image_format="JPEG",       # JPEG is ~10-20x smaller than PNG here
        export_draco_mesh_compression_enable=bool(a["draco"]),
        export_draco_mesh_compression_level=max(1, a["draco"]),
    )
    print("EXPORT tris_before=%d decimate_ratio=%s -> %s" % (before, dec, a["out"]))


main()
