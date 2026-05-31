"""Blender headless probe: import a glTF/GLB and print its world-space bounds.

Run:
  blender --background --python tools/blender_probe_bounds.py -- <file.glb>

The glTF importer converts the file's Y-up frame to Blender's Z-up frame, so the
printed bounds are directly comparable to the navmesh OBJ frame.
"""
import sys
import bpy

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
if not argv:
    raise SystemExit("usage: blender -b --python blender_probe_bounds.py -- <file.glb>")
path = argv[0]

# Start from an empty scene.
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=path)

lo = [float("inf")] * 3
hi = [float("-inf")] * 3
nverts = 0
nobj = 0
for obj in bpy.data.objects:
    if obj.type != "MESH":
        continue
    nobj += 1
    mw = obj.matrix_world
    for v in obj.data.vertices:
        w = mw @ v.co
        for i in range(3):
            lo[i] = min(lo[i], w[i])
            hi[i] = max(hi[i], w[i])
        nverts += 1

print("PROBE mesh_objects=%d verts=%d" % (nobj, nverts))
print("PROBE min=[%.3f, %.3f, %.3f]" % tuple(lo))
print("PROBE max=[%.3f, %.3f, %.3f]" % tuple(hi))
print("PROBE size=[%.3f, %.3f, %.3f]" % tuple(hi[i] - lo[i] for i in range(3)))
