"""Resize the embedded textures of a GLB in place (PIL), then repack the buffer.

  python3 tools/glb_shrink_textures.py <in.glb> <out.glb> [max_edge=1024] [quality=82]

Blender 2.82's glTF exporter re-emits original full-res image bytes even after
in-memory scaling, so we shrink textures here instead: decode each embedded
image, downscale to `max_edge`, re-encode as JPEG, and rebuild the single GLB
binary buffer (recomputing every bufferView offset/length with 4-byte
alignment). Accessors reference bufferViews by index, so mesh data stays valid.
"""
import io
import sys
from PIL import Image
from pygltflib import GLTF2


def main():
    src, dst = sys.argv[1], sys.argv[2]
    max_edge = int(sys.argv[3]) if len(sys.argv) > 3 else 1024
    quality = int(sys.argv[4]) if len(sys.argv) > 4 else 82

    g = GLTF2().load(src)
    blob = g.binary_blob()
    bv_bytes = [bytes(blob[bv.byteOffset:(bv.byteOffset or 0) + bv.byteLength]) for bv in g.bufferViews]

    resized = 0
    for img in g.images:
        if img.bufferView is None:
            continue
        im = Image.open(io.BytesIO(bv_bytes[img.bufferView]))
        if im.mode not in ("RGB", "L"):
            im = im.convert("RGB")
        w, h = im.size
        m = max(w, h)
        if m > max_edge:
            s = max_edge / float(m)
            im = im.resize((max(1, int(w * s)), max(1, int(h * s))), Image.LANCZOS)
        out = io.BytesIO()
        im.save(out, format="JPEG", quality=quality, optimize=True)
        bv_bytes[img.bufferView] = out.getvalue()
        img.mimeType = "image/jpeg"
        resized += 1

    # Rebuild the single binary buffer, 4-byte aligned per bufferView.
    new = bytearray()
    for bv, data in zip(g.bufferViews, bv_bytes):
        if len(new) % 4:
            new += b"\x00" * (4 - len(new) % 4)
        bv.byteOffset = len(new)
        bv.byteLength = len(data)
        bv.buffer = 0
        new += data

    g.buffers[0].byteLength = len(new)
    g.buffers[0].uri = None
    g.set_binary_blob(bytes(new))
    g.save(dst)
    print("SHRINK images=%d max_edge=%d quality=%d -> %s" % (resized, max_edge, quality, dst))


main()
