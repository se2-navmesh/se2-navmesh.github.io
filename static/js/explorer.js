/* Interactive SE(2) NavMesh Path-Planning Explorer.
 *
 * Loads, per scene:
 *   - scene.glb   textured HM3D environment, baked into the navmesh frame
 *                 (meshopt-compressed; needs MeshoptDecoder)
 *   - field.bin   SE(2) traversability field: one record per surface cell,
 *                 [f32 x, y, z, u32 mask]; mask's low `yawBits` bits are the
 *                 feasible headings over [0, pi) (footprint is 180-deg symmetric)
 *   - polyfield.bin/json  exported Detour polygon graph for displaying the
 *                 SE(2) NavMesh surface polygons and browser-side ASA planning
 *   - scene.json  agent, yaw layers, bounds, configured start/goal
 *
 * What it does that the old viewer didn't:
 *   - shows the real environment, not a sliver;
 *   - colors the field by traversability: safe (fits at every heading) vs
 *     restricted (fits at only some) -> makes yaw-dependent traversability
 *     tangible;
 *   - computes the planned path with polygon-level ASA over `polyfield.bin`.
 */
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { MeshoptDecoder } from "three/addons/libs/meshopt_decoder.module.js";

const $ = (id) => document.getElementById(id);
const SCENES_URL = "./static/scenes/index.json";

const COL = {
  bg: 0x0e1626,
  safe: new THREE.Color(0x32c771),       // fits at every heading
  restricted: new THREE.Color(0xffc23d), // fits at only some headings
  path: 0xff6a4d,
  initialAstar: 0xfacc15,
  initialAstarArrow: 0xca8a04,
  stringPull: 0x1d4ed8,
  secondAstar: 0xf59e0b,
  secondAstarArrow: 0xc2410c,
  ref: 0x8aa0c8,
  navmesh: new THREE.Color(0x6db8f2),
  navmeshEdge: 0xd7ebff,
  start: 0x00d85a,
  goal: 0xff1f2d,
  robot: 0x33d6c0,
};

const ASA_STAGE_META = {
  initialAstar: {
    caption: "Initial Search over polygon-edge/yaw states: a feasible path through portal midpoints, with a valid heading at every node.",
  },
  stringPull: {
    caption: "Path Straightening shortens the path inside the polygon corridor. Positions only - orientation is intentionally dropped at this stage.",
  },
  secondAstar: {
    caption: "Yaw Refinement re-optimizes heading along the straightened path, restoring yaw that matches the new direction of motion.",
  },
};

const ASA_DEFAULTS = {
  numYawLayersPi: 20,
  turningFactor: 1.0,
  directionFactor: 1.0,
  heuristicType: 0,
  searchNodeSize: 1048575,
  searchExtent: { x: 1.0, y: 1.0, z: 0.5 },
  maxLonVelocity: 0.5,
  maxLatVelocity: 0.1,
  maxAngVelocity: 0.5,
};

const canvas = $("viewer");
if (canvas) boot().catch(showError);

async function boot() {
  // ── renderer / scene / camera ───────────────────────────────────
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;

  const world = new THREE.Scene();
  world.background = new THREE.Color(COL.bg);

  const camera = new THREE.PerspectiveCamera(45, 1, 0.03, 800);
  camera.up.set(0, 0, 1); // data is z-up

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.rotateSpeed = 0.8;

  world.add(new THREE.HemisphereLight(0xdce6f5, 0x141b29, 1.35));
  const key = new THREE.DirectionalLight(0xffffff, 1.4);
  const fill = new THREE.DirectionalLight(0x9fb3d6, 0.5);
  world.add(key, fill);

  const gltfLoader = new GLTFLoader().setMeshoptDecoder(MeshoptDecoder);

  // ── per-scene state (rebuilt on scene change) ───────────────────
  let S = null; // active scene bundle
  let armedPoseTool = null;    // one-shot target for the next left-drag pose edit
  let startPt = null, goalPt = null;
  let poseDrag = null;

  // ── scene selector ──────────────────────────────────────────────
  const scenes = await fetch(SCENES_URL).then((r) => r.json());
  const sel = $("scene-select");
  scenes.forEach((sc, i) => {
    const o = document.createElement("option");
    o.value = String(i);
    o.textContent = sc.name;
    sel.appendChild(o);
  });
  sel.addEventListener("change", () => loadScene(scenes[Number(sel.value)]));

  // optional ?scene=<dir> deep link (also used by tools/screenshot_scene.sh)
  const want = new URLSearchParams(location.search).get("scene");
  let initial = scenes.findIndex((s) => s.dir === want || s.id === want);
  if (initial < 0) initial = 0;
  sel.value = String(initial);
  await loadScene(scenes[initial]);

  // ── load + build one scene ──────────────────────────────────────
  async function loadScene(meta) {
    const progress = { json: 0, glb: 0, field: 0, polyJson: 0, polyBin: 0 };
    const updateProgress = () => {
      const pct = Math.min(99, Math.round(progress.json + progress.glb + progress.field +
        progress.polyJson + progress.polyBin));
      setLoading("Loading " + meta.name + " … " + pct + "%");
    };
    updateProgress();
    if (S) { world.remove(S.group); disposeGroup(S.group); }

    const base = "./static/scenes/" + meta.dir + "/";
    const sceneJson = fetch(base + "scene.json")
      .then((r) => r.json())
      .then((v) => { progress.json = 5; updateProgress(); return v; });
    const sceneGltf = gltfLoader.loadAsync(base + "scene.glb", (xhr) => {
      if (xhr.lengthComputable && xhr.total > 0) {
        progress.glb = Math.min(70, (xhr.loaded / xhr.total) * 70);
      } else {
        progress.glb = Math.max(progress.glb, 8);
      }
      updateProgress();
    }).then((v) => { progress.glb = 70; updateProgress(); return v; });
    const fieldBin = fetch(base + "field.bin")
      .then((r) => r.arrayBuffer())
      .then((v) => { progress.field = 12; updateProgress(); return v; });
    const polyJson = fetch(base + "polyfield.json")
      .then((r) => r.ok ? r.json() : null)
      .then((v) => { progress.polyJson = 3; updateProgress(); return v; });
    const polyBin = polyJson.then((m) => {
      if (!m) { progress.polyBin = 10; updateProgress(); return null; }
      return fetch(base + (m.binary || "polyfield.bin"))
        .then((r) => r.ok ? r.arrayBuffer() : null)
        .then((v) => { progress.polyBin = 10; updateProgress(); return v; });
    });
    const [scene, gltf, fieldBuf, polyMeta, polyBuf] = await Promise.all([
      sceneJson,
      sceneGltf,
      fieldBin,
      polyJson,
      polyBin,
    ]);
    setLoading("Preparing " + meta.name + " …");

    const group = new THREE.Group();
    world.add(group);

    // environment mesh
    const env = gltf.scene;
    const rayTargets = [];
    env.traverse((o) => { if (o.isMesh) { o.castShadow = false; rayTargets.push(o); } });
    group.add(env);

    // field
    const field = parseField(fieldBuf, scene.field);
    const cells = buildCells(field, scene.field);
    group.add(cells.mesh);

    const polyGraph = polyMeta && polyBuf ? buildPolyOverlay(parsePolyField(polyBuf, polyMeta)) : null;
    if (polyGraph) group.add(polyGraph.group);

    const planner = polyGraph ? buildAsaPlanner(polyGraph.poly, scene) : null;

    const pathGroup = new THREE.Group();
    const asaStageGroup = new THREE.Group();
    const markers = new THREE.Group();
    const robot = buildRobot(scene.agent);
    group.add(pathGroup, asaStageGroup, markers, robot);

    S = { meta, scene, field, cells, polyGraph, planner, group, env, rayTargets,
          pathGroup, asaStageGroup, markers, route: null, asaStages: null,
          asaStage: 2, t: 0, routeTimeline: null, playRate: 1, playLastMs: null, playing: false, robot };

    // camera framing
    const b = scene.field.bounds;
    S.center = new THREE.Vector3((b.min[0]+b.max[0])/2, (b.min[1]+b.max[1])/2, (b.min[2]+b.max[2])/2);
    S.radius = Math.hypot(b.max[0]-b.min[0], b.max[1]-b.min[1], b.max[2]-b.min[2]) / 2;
    key.position.set(S.center.x + S.radius, S.center.y - S.radius, S.center.z + S.radius * 2.2);
    fill.position.set(S.center.x - S.radius, S.center.y + S.radius, S.center.z + S.radius);
    frameView();

    // initial query = configured start/goal
    setQuery(scene.start, scene.goal);
    applyToggles();
    colorCells();
    setLoading(null);
  }

  // ── feasibility field parsing ───────────────────────────────────
  function parseField(buf, fmeta) {
    const dv = new DataView(buf);
    const n = (buf.byteLength / 16) | 0;
    const px = new Float32Array(n), py = new Float32Array(n), pz = new Float32Array(n);
    const mask = new Uint32Array(n);
    for (let i = 0; i < n; i++) {
      const o = i * 16;
      px[i] = dv.getFloat32(o, true);
      py[i] = dv.getFloat32(o + 4, true);
      pz[i] = dv.getFloat32(o + 8, true);
      mask[i] = dv.getUint32(o + 12, true);
    }
    const bits = fmeta.yawBits;
    const full = bits >= 32 ? 0xffffffff : (1 << bits) - 1;
    const safe = new Uint8Array(n);
    for (let i = 0; i < n; i++) safe[i] = (mask[i] & full) === full ? 1 : 0;
    return { n, px, py, pz, mask, safe, bits, full, nLayers: bits * 2,
             yawStep: fmeta.yawStepRad, cellSize: fmeta.cellSize };
  }

  function feasible(field, c, L) {
    return (field.mask[c] >> (L % field.bits)) & 1;
  }

  // ── polygon graph parsing + display ─────────────────────────────
  function parsePolyField(buf, pmeta) {
    const dv = new DataView(buf);
    let o = 0;
    const magic = String.fromCharCode(...new Uint8Array(buf, 0, 8));
    if (magic !== "SE2POLY1") throw new Error("Unsupported polygon graph format: " + magic);
    o += 8;
    const version = dv.getUint32(o, true); o += 4;
    const polyRefBytes = dv.getUint32(o, true); o += 4;
    const yawBits = dv.getUint32(o, true); o += 4;
    const vertexCount = dv.getUint32(o, true); o += 4;
    const polygonCount = dv.getUint32(o, true); o += 4;
    const indexCount = dv.getUint32(o, true); o += 4;
    const neighborCount = dv.getUint32(o, true); o += 4;
    const yawStep = dv.getFloat32(o, true); o += 4;
    const cellSize = dv.getFloat32(o, true); o += 4;

    const vx = new Float32Array(vertexCount);
    const vy = new Float32Array(vertexCount);
    const vz = new Float32Array(vertexCount);
    for (let i = 0; i < vertexCount; i++) {
      vx[i] = dv.getFloat32(o, true); o += 4;
      vy[i] = dv.getFloat32(o, true); o += 4;
      vz[i] = dv.getFloat32(o, true); o += 4;
    }

    const firstVert = new Uint32Array(polygonCount);
    const vertCount = new Uint32Array(polygonCount);
    const firstNeighbor = new Uint32Array(polygonCount);
    const polyNeighborCount = new Uint32Array(polygonCount);
    const mask = new Uint32Array(polygonCount);
    const ref = new Array(polygonCount);
    const area = new Uint16Array(polygonCount);
    const flags = new Uint16Array(polygonCount);
    const tile = new Int32Array(polygonCount);
    const tilePoly = new Int32Array(polygonCount);
    const cx = new Float32Array(polygonCount);
    const cy = new Float32Array(polygonCount);
    const cz = new Float32Array(polygonCount);
    for (let i = 0; i < polygonCount; i++) {
      ref[i] = Number(dv.getBigUint64(o, true)); o += 8;
      firstVert[i] = dv.getUint32(o, true); o += 4;
      vertCount[i] = dv.getUint32(o, true); o += 4;
      firstNeighbor[i] = dv.getUint32(o, true); o += 4;
      polyNeighborCount[i] = dv.getUint32(o, true); o += 4;
      mask[i] = dv.getUint32(o, true); o += 4;
      area[i] = dv.getUint16(o, true); o += 2;
      flags[i] = dv.getUint16(o, true); o += 2;
      tile[i] = dv.getInt32(o, true); o += 4;
      tilePoly[i] = dv.getInt32(o, true); o += 4;
      cx[i] = dv.getFloat32(o, true); o += 4;
      cy[i] = dv.getFloat32(o, true); o += 4;
      cz[i] = dv.getFloat32(o, true); o += 4;
    }

    const indices = new Uint32Array(indexCount);
    for (let i = 0; i < indexCount; i++) { indices[i] = dv.getUint32(o, true); o += 4; }

    const neighborPoly = new Uint32Array(neighborCount);
    const neighborRef = new Array(neighborCount);
    const neighborEdge = new Uint32Array(neighborCount);
    const portal = new Float32Array(neighborCount * 6);
    for (let i = 0; i < neighborCount; i++) {
      neighborPoly[i] = dv.getUint32(o, true); o += 4;
      neighborRef[i] = Number(dv.getBigUint64(o, true)); o += 8;
      neighborEdge[i] = dv.getUint32(o, true); o += 4;
      for (let j = 0; j < 6; j++) { portal[i * 6 + j] = dv.getFloat32(o, true); o += 4; }
    }

    const full = yawBits >= 32 ? 0xffffffff : (Math.pow(2, yawBits) - 1) >>> 0;
    return { version, polyRefBytes, yawBits, yawStep, cellSize, vertexCount,
             polygonCount, indexCount, neighborCount, vx, vy, vz, ref, firstVert,
             vertCount, firstNeighbor, polyNeighborCount, mask, area, flags, tile,
             tilePoly, cx, cy, cz, indices, neighborPoly, neighborRef, neighborEdge,
             portal, full, meta: pmeta };
  }

  function buildPolyOverlay(poly) {
    let triCount = 0, edgeCount = 0;
    for (let i = 0; i < poly.polygonCount; i++) {
      const n = poly.vertCount[i];
      if (n >= 3) triCount += n - 2;
      edgeCount += n;
    }
    const pos = new Float32Array(triCount * 9);
    const col = new Float32Array(triCount * 9);
    const edgePos = new Float32Array(edgeCount * 6);
    const zFace = 0.075, zEdge = 0.09;
    let p = 0, c = 0, e = 0;
    for (let i = 0; i < poly.polygonCount; i++) {
      const start = poly.firstVert[i], n = poly.vertCount[i];
      for (let j = 1; j + 1 < n; j++) {
        p = putVertex(poly, poly.indices[start], zFace, pos, p);
        p = putVertex(poly, poly.indices[start + j], zFace, pos, p);
        p = putVertex(poly, poly.indices[start + j + 1], zFace, pos, p);
        for (let k = 0; k < 3; k++) {
          col[c++] = COL.navmesh.r; col[c++] = COL.navmesh.g; col[c++] = COL.navmesh.b;
        }
      }
      for (let j = 0; j < n; j++) {
        const a = poly.indices[start + j];
        const b = poly.indices[start + ((j + 1) % n)];
        e = putVertex(poly, a, zEdge, edgePos, e);
        e = putVertex(poly, b, zEdge, edgePos, e);
      }
    }

    const faceGeo = new THREE.BufferGeometry();
    faceGeo.setAttribute("position", new THREE.BufferAttribute(pos, 3));
    faceGeo.setAttribute("color", new THREE.BufferAttribute(col, 3));
    faceGeo.computeBoundingSphere();
    const faceMat = new THREE.MeshBasicMaterial({
      vertexColors: true, transparent: true, opacity: 0.34, side: THREE.DoubleSide,
      depthWrite: false, polygonOffset: true, polygonOffsetFactor: -6, polygonOffsetUnits: -6,
    });
    const faces = new THREE.Mesh(faceGeo, faceMat);
    faces.renderOrder = 3;

    const edgeGeo = new THREE.BufferGeometry();
    edgeGeo.setAttribute("position", new THREE.BufferAttribute(edgePos, 3));
    edgeGeo.computeBoundingSphere();
    const edges = new THREE.LineSegments(edgeGeo, new THREE.LineBasicMaterial({
      color: COL.navmeshEdge, transparent: true, opacity: 0.72, depthWrite: false,
    }));
    edges.renderOrder = 4;

    const group = new THREE.Group();
    group.add(faces, edges);
    group.visible = false;
    return { group, poly, faces, edges };
  }

  function putVertex(poly, idx, dz, out, o) {
    out[o++] = poly.vx[idx];
    out[o++] = poly.vy[idx];
    out[o++] = poly.vz[idx] + dz;
    return o;
  }

  // ── traversability cells (instanced, recolored by heading) ──────
  function buildCells(field, fmeta) {
    const geo = new THREE.PlaneGeometry(field.cellSize * 0.96, field.cellSize * 0.96);
    const mat = new THREE.MeshBasicMaterial({
      vertexColors: false, transparent: true, opacity: 0.62, side: THREE.DoubleSide,
      depthWrite: false, polygonOffset: true, polygonOffsetFactor: -3, polygonOffsetUnits: -3,
    });
    const mesh = new THREE.InstancedMesh(geo, mat, field.n);
    mesh.renderOrder = 2;
    const m = new THREE.Matrix4();
    for (let i = 0; i < field.n; i++) {
      m.makeTranslation(field.px[i], field.py[i], field.pz[i] + 0.03);
      mesh.setMatrixAt(i, m);
    }
    mesh.instanceMatrix.needsUpdate = true;
    return { mesh };
  }

  // Binary traversability colouring: a cell is `safe` when it fits at every
  // heading (mask all-ones) and `restricted` otherwise. The exporter drops
  // cells feasible at no heading, so there is no third "blocked" state.
  function colorCells() {
    if (!S) return;
    const f = S.field, mesh = S.cells.mesh;
    let nRestricted = 0;
    for (let i = 0; i < f.n; i++) {
      if (f.safe[i]) mesh.setColorAt(i, COL.safe);
      else { mesh.setColorAt(i, COL.restricted); nRestricted++; }
    }
    mesh.instanceColor.needsUpdate = true;
    $("stat-cells").textContent = f.n.toLocaleString();
    $("stat-restricted").textContent = nRestricted.toLocaleString();
    if (S.robot && !S.route) orientRobot();
  }

  // ── planner: polygon-level ASA over exported Detour graph ───────
  function buildAsaPlanner(poly, scene) {
    const ag = scene.agent || {};
    const params = {
      ...ASA_DEFAULTS,
      numYawLayersPi: poly.yawBits || ASA_DEFAULTS.numYawLayersPi,
      maxLonVelocity: ag.maxLonVelocity || ASA_DEFAULTS.maxLonVelocity,
      maxLatVelocity: ag.maxLatVelocity || ASA_DEFAULTS.maxLatVelocity,
      maxAngVelocity: ag.maxAngVelocity || ASA_DEFAULTS.maxAngVelocity,
    };
    params.nLayers = params.numYawLayersPi * 2;
    params.yawStepRad = poly.yawStep || Math.PI / params.numYawLayersPi;
    params.singleYawLayerCost = params.yawStepRad / params.maxAngVelocity * params.turningFactor;
    params.unitLonDistCost = params.directionFactor / params.maxLonVelocity;
    params.unitLatDistCost = params.directionFactor / params.maxLatVelocity;
    params.unitDistMinCost = params.directionFactor / Math.max(params.maxLonVelocity, params.maxLatVelocity);

    const polygons = new Array(poly.polygonCount);
    const idByRef = new Map();
    for (let i = 0; i < poly.polygonCount; i++) {
      const verts = [];
      const bounds = { min: [Infinity, Infinity, Infinity], max: [-Infinity, -Infinity, -Infinity] };
      const start = poly.firstVert[i];
      for (let j = 0; j < poly.vertCount[i]; j++) {
        const vi = poly.indices[start + j];
        const v = { x: poly.vx[vi], y: poly.vy[vi], z: poly.vz[vi] };
        verts.push(v);
        bounds.min[0] = Math.min(bounds.min[0], v.x); bounds.max[0] = Math.max(bounds.max[0], v.x);
        bounds.min[1] = Math.min(bounds.min[1], v.y); bounds.max[1] = Math.max(bounds.max[1], v.y);
        bounds.min[2] = Math.min(bounds.min[2], v.z); bounds.max[2] = Math.max(bounds.max[2], v.z);
      }
      const p = {
        id: i,
        ref: poly.ref[i],
        verts,
        bounds,
        mask: poly.mask[i],
        area: poly.area[i],
        flags: poly.flags[i],
        center: { x: poly.cx[i], y: poly.cy[i], z: poly.cz[i] },
        neighbors: [],
      };
      polygons[i] = p;
      idByRef.set(p.ref, i);
    }
    for (let i = 0; i < poly.polygonCount; i++) {
      for (let j = 0; j < poly.polyNeighborCount[i]; j++) {
        const ni = poly.firstNeighbor[i] + j;
        const a = {
          x: poly.portal[ni * 6],
          y: poly.portal[ni * 6 + 1],
          z: poly.portal[ni * 6 + 2],
        };
        const b = {
          x: poly.portal[ni * 6 + 3],
          y: poly.portal[ni * 6 + 4],
          z: poly.portal[ni * 6 + 5],
        };
        polygons[i].neighbors.push({
          poly: poly.neighborPoly[ni],
          ref: poly.neighborRef[ni],
          edge: poly.neighborEdge[ni],
          portalA: a,
          portalB: b,
          midpoint: midpoint(a, b),
        });
      }
    }
    return { poly, polygons, idByRef, params };
  }

  function planAsa(start, goal) {
    if (!S || !S.planner) return { route: null, message: "Polygon graph unavailable." };
    const P = S.planner;
    const startLayer = normalizeLayer(start.layer != null ? start.layer : yawToLayer(start.yaw), P.params.nLayers);
    const goalLayer = normalizeLayer(goal.layer != null ? goal.layer : yawToLayer(goal.yaw), P.params.nLayers);
    const debug = { startLayer, goalLayer };
    const s = findNearestPolyMultiLayerWeb(P, start, startLayer);
    if (!s) return { route: null, message: "No start polygon within search extent for the selected yaw." };
    const g = findNearestPolyMultiLayerWeb(P, goal, goalLayer);
    if (!g) return { route: null, message: "No goal polygon within search extent for the selected yaw." };
    debug.start = debugSnap(P, s, start);
    debug.goal = debugSnap(P, g, goal);

    const first = findPathMultiLayerWeb(P, s, g);
    if (!first) return { route: null, message: "ASA first A* could not find a polygon corridor." };
    const initialRoute = routeFromNodes(first.nodes, P);
    const corridor = extractCorridor(P, first.nodes);
    if (corridor.length < 1) return { route: null, message: "ASA corridor extraction failed." };
    debug.firstNodeRefs = first.nodes.map((n) => [P.polygons[n.a].ref, P.polygons[n.b].ref, n.layer]);
    debug.corridorRefs = corridor.map((id) => P.polygons[id].ref);
    const crossings = stringPullCrossings(P, corridor, s.pos, g.pos);
    if (!crossings) return { route: null, message: "ASA string-pulling failed." };
    debug.crossings = crossings.debug;
    const second = findPathMultiLayerFilteredWeb(P, s, g, crossings.map);
    if (!second) return { route: null, message: "ASA second A* could not refine the corridor." };
    const secondRoute = routeFromNodes(second.nodes, P);
    debug.secondNodeRefs = second.nodes.map((n) => [P.polygons[n.a].ref, P.polygons[n.b].ref, n.layer]);
    debug.stageSummary = {
      initialAstar: { points: initialRoute.length },
      stringPull: { points: crossings.corners.length, crossings: crossings.debug.length },
      secondAstar: { points: secondRoute.length },
    };
    return {
      route: secondRoute,
      cost: second.cost,
      corridor,
      debug,
      message: "ASA path computed.",
      stages: {
        initialAstar: {
          label: "Initial Search",
          color: COL.initialAstar,
          arrowColor: COL.initialAstarArrow,
          hasYaw: true,
          caption: ASA_STAGE_META.initialAstar.caption,
          route: initialRoute,
          nodeRefs: debug.firstNodeRefs,
        },
        stringPull: {
          label: "Path Straightening",
          color: COL.stringPull,
          hasYaw: false,
          caption: ASA_STAGE_META.stringPull.caption,
          route: crossings.corners.map((p) => ({ x: p.x, y: p.y, z: p.z })),
          corridorRefs: debug.corridorRefs,
          refs: crossings.refs,
          crossings: crossings.debug,
        },
        secondAstar: {
          label: "Yaw Refinement",
          color: COL.secondAstar,
          arrowColor: COL.secondAstarArrow,
          hasYaw: true,
          caption: ASA_STAGE_META.secondAstar.caption,
          route: secondRoute,
          nodeRefs: debug.secondNodeRefs,
        },
      },
    };
  }

  function findNearestPolyMultiLayerWeb(P, pose, layer) {
    const ex = P.params.searchExtent;
    let best = null, bestDist = Infinity;
    for (const poly of P.polygons) {
      if (!supportsLayer(poly, layer, P.params)) continue;
      if (poly.bounds.max[0] < pose.x - ex.x || poly.bounds.min[0] > pose.x + ex.x ||
          poly.bounds.max[1] < pose.y - ex.y || poly.bounds.min[1] > pose.y + ex.y ||
          poly.bounds.max[2] < pose.z - ex.z || poly.bounds.min[2] > pose.z + ex.z) {
        continue;
      }
      const cp = closestPointOnPoly(poly, pose);
      if (Math.abs(pose.x - cp.x) > ex.x || Math.abs(pose.y - cp.y) > ex.y ||
          Math.abs(pose.z - cp.z) > ex.z) {
        continue;
      }
      const dz = pose.z - cp.z;
      const d = cp.over ? Math.max(0, Math.abs(dz) - (S.scene.agent.maxClimb || 0.25)) ** 2
        : distSq(pose, cp);
      if (d < bestDist) {
        bestDist = d;
        best = { poly: poly.id, ref: poly.ref, pos: { x: cp.x, y: cp.y, z: cp.z }, layer };
      }
    }
    return best;
  }

  function findPathMultiLayerWeb(P, s, g) {
    return asaSearch(P, s, g, null, false);
  }

  function findPathMultiLayerFilteredWeb(P, s, g, crossingMap) {
    return asaSearch(P, s, g, crossingMap, true);
  }

  function asaSearch(P, s, g, crossingMap, ordered) {
    const params = P.params;
    const open = new MinHeap();
    const nodes = new Map();
    const closed = new Set();
    let created = 0;

    const start = makeNode(P, s.poly, s.poly, s.layer, s.pos, "start", null, 0, "start", "start", ordered);
    start.total = heuristic(P, start, g);
    nodes.set(start.key, start);
    open.push(start.key, start.total);
    created++;

    while (open.size) {
      const key = open.pop();
      const cur = nodes.get(key);
      if (!cur || closed.has(key)) continue;
      closed.add(key);
      if (isGoalNode(cur, g)) return reconstructResult(nodes, cur);

      const parent = cur.parent ? nodes.get(cur.parent) : null;
      const yawChecks = [lowerLayer(cur.layer, params), upperLayer(cur.layer, params)];
      for (let i = 0; i < yawChecks.length; i++) {
        if (params.nLayers === 2 && i === 1) continue;
        const layer = yawChecks[i];
        if (parent && cur.layer !== parent.layer && layer === parent.layer) continue;
        const polyA = P.polygons[cur.a], polyB = P.polygons[cur.b];
        if (!supportsCombined(polyA, polyB, layer, params)) continue;
        const cand = makeNode(P, cur.a, cur.b, layer, cur.pos, cur.posKey, null,
          cur.cost + params.singleYawLayerCost, cur.stepFrom, cur.stepTo, ordered);
        if (relax(cand, cur, g)) created++;
      }

      if (ordered) {
        expandOrderedSpatial(cur, parent);
      } else {
        expandFirstStageSpatial(cur, parent);
      }
      connectGoal(cur);
    }
    return null;

    function expandFirstStageSpatial(cur, parent) {
      const bases = cur.a === cur.b ? [cur.a] : [cur.a, cur.b];
      for (const baseId of bases) {
        const otherId = baseId === cur.a ? cur.b : cur.a;
        const base = P.polygons[baseId];
        for (const nb of base.neighbors) {
          const nextId = nb.poly;
          if (nextId === otherId) continue;
          const pair = unorderedPairByRef(P, baseId, nextId);
          if (parent && pair.a === parent.a && pair.b === parent.b) continue;
          if (!passDouble(P, baseId, nextId, cur.layer)) continue;
          const pos = nb.midpoint;
          const cost = cur.cost + translationCost(P, cur.pos, pos, cur.layer);
          const cand = makeNode(P, pair.a, pair.b, cur.layer, pos, pointKey(pos), null, cost, baseId, nextId, false);
          if (relax(cand, cur, g)) created++;
        }
      }
    }

    function expandOrderedSpatial(cur, parent) {
      const fromId = cur.b;
      const from = P.polygons[fromId];
      for (const nb of from.neighbors) {
        const nextId = nb.poly;
        if (parent && fromId === parent.a && nextId === parent.b) continue;
        const mapKey = crossingKey(P, fromId, nextId);
        const pos = crossingMap.get(mapKey);
        if (!pos) continue;
        if (!passDouble(P, fromId, nextId, cur.layer)) continue;
        const cost = cur.cost + (dist(cur.pos, pos) < 1e-4 ? 0 : translationCost(P, cur.pos, pos, cur.layer));
        const cand = makeNode(P, fromId, nextId, cur.layer, pos, pointKey(pos), null, cost, fromId, nextId, true);
        if (relax(cand, cur, g)) created++;
      }
    }

    function connectGoal(cur) {
      const touchesGoal = ordered ? cur.b === g.poly : (cur.a === g.poly || cur.b === g.poly);
      if (!touchesGoal) return;
      const refsDiffer = cur.a !== cur.b;
      const posDiffer = distSq(cur.pos, g.pos) > 1e-10;
      if (!refsDiffer && !posDiffer) return;
      if (!supportsLayer(P.polygons[g.poly], cur.layer, params)) return;
      const cost = cur.cost + translationCost(P, cur.pos, g.pos, cur.layer);
      const cand = makeNode(P, g.poly, g.poly, cur.layer, g.pos, "goal", null, cost, g.poly, g.poly, ordered);
      if (relax(cand, cur, g)) created++;
    }

    function relax(cand, parent, goal) {
      if (created > params.searchNodeSize) return false;
      cand.parent = parent.key;
      cand.total = cand.cost + heuristic(P, cand, goal);
      const old = nodes.get(cand.key);
      if (old && old.cost <= cand.cost) return false;
      nodes.set(cand.key, cand);
      open.push(cand.key, cand.total);
      return !old;
    }
  }

  function makeNode(P, a, b, layer, pos, posKeyValue, parent, cost, stepFrom, stepTo, ordered) {
    const pk = posKeyValue || pointKey(pos);
    const ar = P.polygons[a].ref, br = P.polygons[b].ref;
    const needsPosition = a === b;
    const key = (ordered ? "o:" : "u:") + ar + ":" + br + ":" + layer + ":0" +
      (needsPosition ? ":" + pk : "");
    return { a, b, layer, pos: { x: pos.x, y: pos.y, z: pos.z }, posKey: pk,
             parent, cost, total: cost, key, stepFrom, stepTo };
  }

  function reconstructResult(nodes, goalNode) {
    const seq = [];
    for (let n = goalNode; n; n = n.parent ? nodes.get(n.parent) : null) seq.push(n);
    seq.reverse();
    return { nodes: seq, cost: goalNode.cost };
  }

  function isGoalNode(n, g) {
    return n.a === g.poly && n.b === g.poly && n.layer === g.layer && distSq(n.pos, g.pos) < 1e-10;
  }

  function extractCorridor(P, nodes) {
    const pathA = [], pathB = [];
    for (const n of nodes) {
      const last = pathA.length - 1;
      if (last < 0 || pathA[last] !== n.a || pathB[last] !== n.b) {
        pathA.push(n.a);
        pathB.push(n.b);
      }
    }
    return compressPolyPathMultiLayer(P, extractPolyPath(pathA, pathB));
  }

  function extractPolyPath(pathA, pathB) {
    const polyPath = [];
    if (!pathA.length || pathA.length !== pathB.length || pathA[0] !== pathB[0]) return polyPath;
    polyPath.push(pathA[0]);
    let lastA = pathA[0], lastB = pathB[0];
    for (let i = 1; i < pathA.length; i++) {
      const prev = polyPath[polyPath.length - 1];
      if (pathA[i] === prev && pathB[i] === prev) {
        lastA = pathA[i]; lastB = pathB[i];
        continue;
      }
      if (pathA[i] === prev || pathB[i] === prev) {
        polyPath.push(pathA[i] === prev ? pathB[i] : pathA[i]);
      } else if (lastA === pathA[i]) {
        polyPath.push(lastA, pathB[i]);
      } else if (lastA === pathB[i]) {
        polyPath.push(lastA, pathA[i]);
      } else if (lastB === pathA[i]) {
        polyPath.push(lastB, pathB[i]);
      } else if (lastB === pathB[i]) {
        polyPath.push(lastB, pathA[i]);
      } else {
        return [];
      }
      lastA = pathA[i];
      lastB = pathB[i];
    }
    return polyPath;
  }

  function compressPolyPathMultiLayer(P, path) {
    const out = [];
    const full = P.params.numYawLayersPi >= 32 ? 0xffffffff :
      ((1 << P.params.numYawLayersPi) - 1) >>> 0;
    let i = 0;
    while (i < path.length) {
      if (i + 2 < path.length && path[i] === path[i + 2] &&
          (P.polygons[path[i]].mask & full) === full) {
        out.push(path[i]);
        i += 3;
      } else {
        out.push(path[i]);
        i++;
      }
    }
    return out;
  }

  function stringPullCrossings(P, corridor, start, goal) {
    const straight = findStraightPathAllCrossings(P, corridor, start, goal);
    if (!straight) return null;
    if (straight.refs.length) straight.refs[straight.refs.length - 1] = P.polygons[corridor[corridor.length - 1]].ref;
    const map = new Map();
    let idx = 0;
    const filteredPos = new Array(corridor.length);
    for (let i = 0; i < corridor.length; i++) {
      const ref = P.polygons[corridor[i]].ref;
      filteredPos[i] = straight.points[Math.min(idx, straight.points.length - 1)];
      if (straight.refs[idx] === ref && idx + 1 < straight.refs.length) idx++;
    }
    for (let i = 1; i < corridor.length; i++) {
      map.set(crossingKey(P, corridor[i - 1], corridor[i]), filteredPos[i]);
    }
    return {
      corners: straight.points,
      refs: straight.refs,
      map,
      debug: Array.from(map.entries()).map(([key, pos]) => ({ key, pos: { ...pos } })),
    };
  }

  function findStraightPathAllCrossings(P, corridor, start, goal) {
    if (!corridor.length) return null;
    const closestStart = closestPointOnPolyBoundary(P.polygons[corridor[0]], start);
    const closestGoal = closestPointOnPolyBoundary(P.polygons[corridor[corridor.length - 1]], goal);
    const points = [], refs = [];
    appendStraightPoint(points, refs, closestStart, P.polygons[corridor[0]].ref);
    if (corridor.length === 1) {
      appendStraightPoint(points, refs, closestGoal, 0);
      return { points, refs };
    }

    let apex = closestStart;
    let left = closestStart;
    let right = closestStart;
    let apexIndex = 0;
    let leftIndex = 0, rightIndex = 0;
    let leftRef = P.polygons[corridor[0]].ref, rightRef = P.polygons[corridor[0]].ref;

    for (let i = 0; i < corridor.length; i++) {
      let newLeft, newRight;
      if (i + 1 < corridor.length) {
        const portal = portalForPath(P, corridor[i], corridor[i + 1]);
        if (!portal) return null;
        newLeft = portal.left;
        newRight = portal.right;
        if (i === 0 && distPtSegSq2D(apex, newLeft, newRight) < 0.001 * 0.001) continue;
      } else {
        newLeft = closestGoal;
        newRight = closestGoal;
      }

      if (triarea2(apex, right, newRight) <= 0) {
        if (samePoint(apex, right) || triarea2(apex, left, newRight) > 0) {
          right = newRight;
          rightRef = i + 1 < corridor.length ? P.polygons[corridor[i + 1]].ref : 0;
          rightIndex = i;
        } else {
          appendPortalsAllCrossings(P, points, refs, corridor, apex, left, apexIndex, leftIndex);
          appendStraightPoint(points, refs, left, leftRef);
          apex = left;
          apexIndex = leftIndex;
          left = apex; right = apex;
          leftIndex = apexIndex; rightIndex = apexIndex;
          i = apexIndex;
          continue;
        }
      }

      if (triarea2(apex, left, newLeft) >= 0) {
        if (samePoint(apex, left) || triarea2(apex, right, newLeft) < 0) {
          left = newLeft;
          leftRef = i + 1 < corridor.length ? P.polygons[corridor[i + 1]].ref : 0;
          leftIndex = i;
        } else {
          appendPortalsAllCrossings(P, points, refs, corridor, apex, right, apexIndex, rightIndex);
          appendStraightPoint(points, refs, right, rightRef);
          apex = right;
          apexIndex = rightIndex;
          left = apex; right = apex;
          leftIndex = apexIndex; rightIndex = apexIndex;
          i = apexIndex;
          continue;
        }
      }
    }

    appendPortalsAllCrossings(P, points, refs, corridor, apex, closestGoal, apexIndex, corridor.length - 1);
    appendStraightPoint(points, refs, closestGoal, 0);
    return { points, refs };
  }

  function appendPortalsAllCrossings(P, points, refs, corridor, startPos, endPos, startIdx, endIdx) {
    for (let i = startIdx; i < endIdx; i++) {
      const edge = findNeighbor(P, corridor[i], corridor[i + 1]);
      if (!edge) continue;
      const hit = segmentPortalIntersection2D(startPos, endPos, edge.portalA, edge.portalB);
      if (hit) {
        appendStraightPoint(points, refs, hit, P.polygons[corridor[i + 1]].ref);
      }
    }
  }

  function routeFromNodes(nodes, P) {
    const out = [];
    for (const n of nodes) {
      const p = { x: n.pos.x, y: n.pos.y, z: n.pos.z, layer: n.layer,
                  yaw: (n.layer - 1) * P.params.yawStepRad };
      const last = out[out.length - 1];
      if (!last || distSq(last, p) > 1e-10 || last.layer !== p.layer) out.push(p);
    }
    return out;
  }

  function passDouble(P, currentId, neighborId, layer) {
    const params = P.params, cur = P.polygons[currentId], nb = P.polygons[neighborId];
    if (!supportsLayer(cur, layer, params)) return false;
    return supportsLayer(nb, layer, params) ||
      supportsLayer(nb, lowerLayer(layer, params), params) ||
      supportsLayer(nb, upperLayer(layer, params), params);
  }

  function supportsCombined(a, b, layer, params) {
    const bit = layerBit(layer, params);
    return (((a.mask | b.mask) >>> bit) & 1) !== 0;
  }

  function supportsLayer(poly, layer, params) {
    const bit = layerBit(layer, params);
    return ((poly.mask >>> bit) & 1) !== 0;
  }

  function layerBit(layer, params) {
    return (layer > params.numYawLayersPi ? layer - params.numYawLayersPi : layer) - 1;
  }

  function lowerLayer(layer, params) { return layer > 1 ? layer - 1 : params.nLayers; }
  function upperLayer(layer, params) { return layer < params.nLayers ? layer + 1 : 1; }
  function normalizeLayer(layer, nLayers) { return ((layer - 1) % nLayers + nLayers) % nLayers + 1; }

  function translationCost(P, a, b, layer) {
    const dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
    const d = Math.hypot(dx, dy, dz);
    if (d < 1e-9) return 0;
    const diff = Math.atan2(dy, dx) - (layer - 1) * P.params.yawStepRad;
    return d * (Math.abs(Math.cos(diff)) * P.params.unitLonDistCost +
                Math.abs(Math.sin(diff)) * P.params.unitLatDistCost);
  }

  function heuristic(P, n, goal) {
    const d = P.params.heuristicType === 1
      ? Math.abs(n.pos.x - goal.pos.x) + Math.abs(n.pos.y - goal.pos.y) + Math.abs(n.pos.z - goal.pos.z)
      : dist(n.pos, goal.pos);
    return d * P.params.unitDistMinCost +
      layerDistance(n.layer, goal.layer, P.params.nLayers) * P.params.singleYawLayerCost;
  }

  function layerDistance(a, b, n) {
    const d = Math.abs(a - b);
    return Math.min(d, n - d);
  }

  function findNeighbor(P, from, to) {
    return P.polygons[from].neighbors.find((n) => n.poly === to) || null;
  }

  function portalForPath(P, from, to) {
    const edge = findNeighbor(P, from, to);
    if (!edge) return null;
    // Exported portal endpoints already preserve Detour's left/right order in
    // the browser XY plane used by the funnel.
    return { left: edge.portalA, right: edge.portalB };
  }

  function unorderedPairByRef(P, a, b) {
    const ar = P.polygons[a].ref, br = P.polygons[b].ref;
    return ar <= br ? { a, b } : { a: b, b: a };
  }

  function crossingKey(P, from, to) {
    return P.polygons[from].ref + ":" + P.polygons[to].ref;
  }

  function debugSnap(P, snapped, requested) {
    return {
      ref: snapped.ref,
      poly: snapped.poly,
      layer: snapped.layer,
      requested: { x: requested.x, y: requested.y, z: requested.z, yaw: requested.yaw },
      nearest: { ...snapped.pos },
      offset: {
        x: snapped.pos.x - requested.x,
        y: snapped.pos.y - requested.y,
        z: snapped.pos.z - requested.z,
      },
      mask: P.polygons[snapped.poly].mask,
    };
  }

  function appendStraightPoint(points, refs, pos, ref) {
    const last = points[points.length - 1];
    if (last && samePoint3D(last, pos)) {
      refs[refs.length - 1] = ref;
      return;
    }
    points.push({ x: pos.x, y: pos.y, z: pos.z });
    refs.push(ref);
  }

  function pointKey(p) { return p.x.toFixed(4) + "," + p.y.toFixed(4) + "," + p.z.toFixed(4); }
  function midpoint(a, b) { return { x: (a.x + b.x) * 0.5, y: (a.y + b.y) * 0.5, z: (a.z + b.z) * 0.5 }; }
  function dist(a, b) { return Math.sqrt(distSq(a, b)); }
  function distSq(a, b) { const dx = a.x-b.x, dy = a.y-b.y, dz = a.z-b.z; return dx*dx + dy*dy + dz*dz; }
  function samePoint(a, b) { return Math.abs(a.x-b.x) < 1e-6 && Math.abs(a.y-b.y) < 1e-6; }
  function samePoint3D(a, b) { return distSq(a, b) < 1e-12; }
  function cross2(ax, ay, bx, by) { return ax * by - ay * bx; }
  function triarea2(a, b, c) { return cross2(b.x - a.x, b.y - a.y, c.x - a.x, c.y - a.y); }

  function closestPointOnPoly(poly, p) {
    if (pointInPolyXY(poly.verts, p)) {
      return { ...projectToPolyPlane(poly, p), over: true };
    }
    let best = null, bd = Infinity;
    for (let i = 0; i < poly.verts.length; i++) {
      const q = closestPointOnSegment3D(p, poly.verts[i], poly.verts[(i + 1) % poly.verts.length]);
      const d = distSq(p, q);
      if (d < bd) { bd = d; best = q; }
    }
    return { ...best, over: false };
  }

  function closestPointOnPolyBoundary(poly, p) {
    if (pointInPolyXY(poly.verts, p)) return projectToPolyPlane(poly, p);
    let best = null, bd = Infinity;
    for (let i = 0; i < poly.verts.length; i++) {
      const q = closestPointOnSegment2D(p, poly.verts[i], poly.verts[(i + 1) % poly.verts.length]);
      const d = distSq(p, q);
      if (d < bd) { bd = d; best = q; }
    }
    return best || { x: p.x, y: p.y, z: p.z };
  }

  function pointInPolyXY(verts, p) {
    let inside = false;
    for (let i = 0, j = verts.length - 1; i < verts.length; j = i++) {
      const a = verts[i], b = verts[j];
      if (((a.y > p.y) !== (b.y > p.y)) &&
          (p.x < (b.x - a.x) * (p.y - a.y) / ((b.y - a.y) || 1e-12) + a.x)) {
        inside = !inside;
      }
    }
    return inside;
  }

  function projectToPolyPlane(poly, p) {
    const a = poly.verts[0], b = poly.verts[1], c = poly.verts[2];
    const ux = b.x-a.x, uy = b.y-a.y, uz = b.z-a.z;
    const vx = c.x-a.x, vy = c.y-a.y, vz = c.z-a.z;
    const nx = uy*vz - uz*vy, ny = uz*vx - ux*vz, nz = ux*vy - uy*vx;
    const z = Math.abs(nz) < 1e-8 ? poly.center.z : a.z - (nx*(p.x-a.x) + ny*(p.y-a.y)) / nz;
    return { x: p.x, y: p.y, z };
  }

  function closestPointOnSegment3D(p, a, b) {
    const dx = b.x-a.x, dy = b.y-a.y, dz = b.z-a.z;
    const l2 = dx*dx + dy*dy + dz*dz;
    const t = l2 > 0 ? Math.max(0, Math.min(1, ((p.x-a.x)*dx + (p.y-a.y)*dy + (p.z-a.z)*dz) / l2)) : 0;
    return { x: a.x + dx*t, y: a.y + dy*t, z: a.z + dz*t };
  }

  function closestPointOnSegment2D(p, a, b) {
    const dx = b.x-a.x, dy = b.y-a.y;
    const l2 = dx*dx + dy*dy;
    const t = l2 > 0 ? Math.max(0, Math.min(1, ((p.x-a.x)*dx + (p.y-a.y)*dy) / l2)) : 0;
    return { x: a.x + dx*t, y: a.y + dy*t, z: a.z + (b.z-a.z)*t };
  }

  function distPtSegSq2D(p, a, b) {
    const q = closestPointOnSegment2D(p, a, b);
    const dx = p.x - q.x, dy = p.y - q.y;
    return dx*dx + dy*dy;
  }

  function segmentPortalIntersection2D(a, b, c, d) {
    const r = { x: b.x - a.x, y: b.y - a.y };
    const s = { x: d.x - c.x, y: d.y - c.y };
    const den = cross2(r.x, r.y, s.x, s.y);
    if (Math.abs(den) < 1e-9) return null;
    const qp = { x: c.x - a.x, y: c.y - a.y };
    const t = cross2(qp.x, qp.y, s.x, s.y) / den;
    const u = cross2(qp.x, qp.y, r.x, r.y) / den;
    if (t < -1e-6 || t > 1 + 1e-6 || u < -1e-6 || u > 1 + 1e-6) return null;
    return { x: c.x + (d.x - c.x)*u, y: c.y + (d.y - c.y)*u, z: c.z + (d.z - c.z)*u };
  }

  // ── query + polygon-level ASA path ──────────────────────────────
  function setQuery(start, goal) {
    startPt = poseFromScenePoint(start, 0);
    goalPt = poseFromScenePoint(goal, 0);
    disarmPoseTool("Showing the configured query. Click Set Start or Set Goal, then left-drag on the scene to set a pose.");
    replan();
  }

  function replan() {
    S.markers.clear();
    if (startPt) S.markers.add(poseArrow(startPt, COL.start));
    if (goalPt) S.markers.add(poseArrow(goalPt, COL.goal));

    if (!startPt || !goalPt) {
      S.lastPlanDebug = null;
      S.asaStages = null;
      S.route = null;
      S.routeTimeline = null;
      S.playing = false;
      S.playLastMs = null;
      updateRouteControls();
      window.__se2ExplorerLastPlanDebug = null;
      writePlanDebug(null);
      showAsaStage(-1);
      drawRoute(null, "Set both start and goal poses.");
      return;
    }
    const result = planAsa(startPt, goalPt);
    S.lastPlanDebug = result.debug || null;
    S.asaStages = result.stages || null;
    window.__se2ExplorerLastPlanDebug = S.lastPlanDebug;
    writePlanDebug(S.lastPlanDebug);
    if (!result.route) {
      S.asaStages = null;
      S.playing = false;
    }
    drawRoute(result.route, result.message);
    S.routeTimeline = buildRouteTimeline(S.route);
    S.playLastMs = null;
    if (result.route && S.asaStage < 0) S.asaStage = 2;
    showAsaStage(S.asaStage);
    updateRouteControls();
    setRobotAt(S.t);
  }

  function drawRoute(route, message) {
    S.pathGroup.clear();
    S.route = route;
    if (!route || route.length < 2) {
      $("stat-len").textContent = "—";
      $("stat-cost").textContent = route ? "—" : "no path";
      if (message) setHint(message);
      return;
    }
    const stats = routeStats(route, true);
    const len = stats.len, fwd = stats.fwd, lat = stats.lat, turn = stats.turn;
    $("stat-len").textContent = len.toFixed(2) + " m";
    $("stat-cost").textContent = fwd.toFixed(2) + " m fwd · " + lat.toFixed(2) +
      " m lat · " + (turn*180/Math.PI).toFixed(0) + "° turn";
    if (message) setHint(message);
  }

  function showAsaStage(stageIndex) {
    if (!S || !S.asaStageGroup) return;
    S.asaStageGroup.clear();
    const stages = asaStageList();
    const active = stages[stageIndex] ? stageIndex : -1;
    S.asaStage = active;
    updateAsaStageUi(active, stages);
    if (active < 0) return;
    if (!asaPipelineVisible()) return;
    drawAsaStageRoute(stages[active]);
  }

  function asaStageList() {
    const stages = S && S.asaStages;
    return stages ? [stages.initialAstar, stages.stringPull, stages.secondAstar] : [];
  }

  function drawAsaStageRoute(stage) {
    const pts = routePolylinePoints(stage.route);
    if (pts.length > 1) {
      S.asaStageGroup.add(polyline(pts, stage.color, 0.0225, 1.0, true));
    }
    if (stage.hasYaw) S.asaStageGroup.add(stageHeadingArrows(stage.route, stage.arrowColor || stage.color));
  }

  function stageHeadingArrows(route, color) {
    const group = new THREE.Group();
    const maxArrows = 24;
    const step = Math.max(1, Math.ceil(route.length / maxArrows));
    const headLen = Math.max(0.14, Math.min(0.22, (S.scene.agent.length || 0.8) * 0.22));
    const headRadius = 0.028;
    const zOffset = pathHeightOffset();
    const mat = new THREE.MeshStandardMaterial({
      color,
      emissive: color,
      emissiveIntensity: 0.35,
      roughness: 0.38,
      metalness: 0.0,
      depthWrite: false,
    });
    const geo = new THREE.ConeGeometry(headRadius, headLen, 18);
    for (let i = 0; i < route.length; i += step) {
      const p = route[i];
      if (!Number.isFinite(p.yaw)) continue;
      const head = new THREE.Mesh(geo, mat);
      head.position.set(
        p.x + Math.cos(p.yaw) * headLen * 0.5,
        p.y + Math.sin(p.yaw) * headLen * 0.5,
        p.z + zOffset);
      head.rotation.z = p.yaw - Math.PI / 2;
      head.renderOrder = 6;
      group.add(head);
    }
    return group;
  }

  function updateAsaStageUi(active, stages) {
    document.querySelectorAll("#asa-stage-controls [data-asa-stage]").forEach((btn) => {
      btn.classList.toggle("is-active", Number(btn.dataset.asaStage) === active);
    });
    const caption = $("asa-caption");
    const statsEl = $("asa-stats");
    if (!caption || !statsEl) return;
    if (!stages.length) {
      caption.textContent = "ASA stages unavailable for this query.";
      statsEl.innerHTML = "";
      return;
    }
    if (active < 0) {
      caption.textContent = "Select a stage to overlay the browser planner's A* -> string-pull -> A* path for the active query.";
      statsEl.innerHTML = "";
      return;
    }
    const stage = stages[active];
    const stats = routeStats(stage.route, stage.hasYaw);
    caption.textContent = stage.caption || stage.label;
    statsEl.innerHTML = "<div><span>Length</span><strong>" + stats.len.toFixed(2) + " m</strong></div>" +
      (stage.hasYaw
        ? "<div><span>Fwd · lat · turn</span><strong>" + stats.fwd.toFixed(2) + " · " +
          stats.lat.toFixed(2) + " m · " + (stats.turn*180/Math.PI).toFixed(0) + "°</strong></div>"
        : "");
  }

  function routePolylinePoints(route) {
    const pts = [];
    const zOffset = pathHeightOffset();
    for (const p of route || []) {
      const v = [p.x, p.y, p.z + zOffset], last = pts[pts.length - 1];
      if (!last || Math.hypot(v[0] - last[0], v[1] - last[1], v[2] - last[2]) > 1e-4) pts.push(v);
    }
    return pts;
  }

  function routeStats(route, hasYaw) {
    let len = 0, fwd = 0, lat = 0, turn = 0;
    for (let i = 1; route && i < route.length; i++) {
      const dx = route[i].x - route[i-1].x, dy = route[i].y - route[i-1].y, dz = route[i].z - route[i-1].z;
      const d = Math.hypot(dx, dy, dz);
      len += d;
      if (!hasYaw) continue;
      if (d > 1e-9) {
        const diff = Math.atan2(dy, dx) - route[i-1].yaw;
        fwd += d * Math.abs(Math.cos(diff));
        lat += d * Math.abs(Math.sin(diff));
      }
      let a = Math.abs(route[i].yaw - route[i-1].yaw) % (2*Math.PI);
      if (a > Math.PI) a = 2*Math.PI - a;
      turn += a;
    }
    return { len, fwd, lat, turn };
  }

  // ── robot footprint along route ─────────────────────────────────
  function setRobotAt(t) {
    if (!S || !S.robot) return;
    const route = S.route;
    if (!route || route.length < 2) { orientRobot(); return; }
    S.t = Math.max(0, Math.min(1, t));
    const timed = routePoseAt(S.t);
    const i = timed.index, k = timed.u, a = route[i], b = route[i + 1];
    const zOffset = pathHeightOffset();
    S.robot.position.set(a.x + (b.x-a.x)*k, a.y + (b.y-a.y)*k, a.z + (b.z-a.z)*k + zOffset);
    let yaw = a.yaw + shortestAngle(a.yaw, b.yaw) * k;
    S.robot.rotation.set(0, 0, yaw);
  }
  function routePoseAt(t) {
    const route = S.route;
    const timeline = S.routeTimeline;
    if (!timeline || !timeline.total || timeline.cumulative.length !== route.length) {
      const f = t * (route.length - 1);
      const index = Math.min(route.length - 2, Math.floor(f));
      return { index, u: f - index };
    }
    const target = Math.max(0, Math.min(1, t)) * timeline.total;
    let lo = 0, hi = timeline.cumulative.length - 1;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (timeline.cumulative[mid] < target) lo = mid + 1;
      else hi = mid;
    }
    const index = Math.max(0, Math.min(route.length - 2, lo - 1));
    const t0 = timeline.cumulative[index], t1 = timeline.cumulative[index + 1];
    const u = t1 > t0 ? (target - t0) / (t1 - t0) : 0;
    return { index, u: Math.max(0, Math.min(1, u)) };
  }
  function buildRouteTimeline(route) {
    const cumulative = [0];
    if (!route || route.length < 2) return { cumulative, total: 0 };
    const p = S && S.planner ? S.planner.params : ASA_DEFAULTS;
    const lonV = Math.max(1e-6, p.maxLonVelocity || ASA_DEFAULTS.maxLonVelocity);
    const latV = Math.max(1e-6, p.maxLatVelocity || ASA_DEFAULTS.maxLatVelocity);
    const angV = Math.max(1e-6, p.maxAngVelocity || ASA_DEFAULTS.maxAngVelocity);
    let total = 0;
    for (let i = 1; i < route.length; i++) {
      const a = route[i - 1], b = route[i];
      const dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
      const dist = Math.hypot(dx, dy, dz);
      let moveTime = 0;
      if (dist > 1e-9) {
        const diff = Math.atan2(dy, dx) - a.yaw;
        moveTime = dist * Math.abs(Math.cos(diff)) / lonV +
          dist * Math.abs(Math.sin(diff)) / latV;
      }
      const yawTime = Math.abs(shortestAngle(a.yaw || 0, b.yaw || 0)) / angV;
      total += moveTime + yawTime;
      cumulative.push(total);
    }
    return { cumulative, total };
  }
  function orientRobot() {
    if (!S || !S.robot || !startPt) return;
    S.robot.position.set(startPt.x, startPt.y, startPt.z + pathHeightOffset());
    S.robot.rotation.set(0, 0, startPt.yaw || 0);
  }
  function updateRouteControls() {
    const slider = $("route"), play = $("play"), rate = $("play-rate");
    const enabled = !!(S && S.route && S.route.length > 1);
    if (slider) {
      slider.disabled = !enabled;
      slider.value = String(Math.round((S && S.t || 0) * 1000));
    }
    if (play) {
      play.disabled = !enabled;
      play.textContent = S && S.playing ? "Pause" : "Play route";
    }
    if (rate) rate.textContent = (S ? S.playRate : 1) + "x";
  }

  // ── interaction: RViz-style pose tool for start / goal ──────────
  const ray = new THREE.Raycaster();
  const ndc = new THREE.Vector2();
  const dragPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
  const dragHit = new THREE.Vector3();
  renderer.domElement.addEventListener("pointerdown", (e) => {
    if (e.button !== 0 || !S || !armedPoseTool) return;
    const hit = pickScenePoint(e);
    if (!hit) return;
    e.preventDefault();
    renderer.domElement.setPointerCapture(e.pointerId);
    controls.enabled = false;
    dragPlane.set(new THREE.Vector3(0, 0, 1), -hit.point.z);
    const yaw = currentToolPoseYaw();
    poseDrag = { pointerId: e.pointerId, target: armedPoseTool, point: hit.point.clone(), yaw };
    setDraftPose(yaw);
  });
  renderer.domElement.addEventListener("pointermove", (e) => {
    if (!poseDrag || e.pointerId !== poseDrag.pointerId) return;
    e.preventDefault();
    const p = pointOnDragPlane(e);
    if (!p) return;
    const dx = p.x - poseDrag.point.x, dy = p.y - poseDrag.point.y;
    if (Math.hypot(dx, dy) > 1e-4) poseDrag.yaw = Math.atan2(dy, dx);
    setDraftPose(poseDrag.yaw);
  });
  renderer.domElement.addEventListener("pointerup", (e) => {
    if (!poseDrag || e.pointerId !== poseDrag.pointerId) return;
    e.preventDefault();
    const p = pointOnDragPlane(e);
    if (p) {
      const dx = p.x - poseDrag.point.x, dy = p.y - poseDrag.point.y;
      if (Math.hypot(dx, dy) > 1e-4) poseDrag.yaw = Math.atan2(dy, dx);
    }
    const pose = {
      x: poseDrag.point.x,
      y: poseDrag.point.y,
      z: poseDrag.point.z,
      yaw: normalizeYaw(poseDrag.yaw),
    };
    pose.layer = yawToLayer(pose.yaw);
    const target = poseDrag.target;
    if (target === "start") startPt = pose;
    else goalPt = pose;
    poseDrag = null;
    controls.enabled = true;
    renderer.domElement.releasePointerCapture(e.pointerId);
    replan();
    disarmPoseTool((target === "start" ? "Start" : "Goal") + " pose set. Click Set Start or Set Goal to edit another pose.");
  });
  renderer.domElement.addEventListener("pointercancel", (e) => {
    if (!poseDrag || e.pointerId !== poseDrag.pointerId) return;
    poseDrag = null;
    controls.enabled = true;
    if (renderer.domElement.hasPointerCapture(e.pointerId)) {
      renderer.domElement.releasePointerCapture(e.pointerId);
    }
    disarmPoseTool("Pose edit cancelled. Click Set Start or Set Goal to try again.");
  });

  function pickScenePoint(e) {
    const r = renderer.domElement.getBoundingClientRect();
    ndc.x = ((e.clientX - r.left) / r.width) * 2 - 1;
    ndc.y = -((e.clientY - r.top) / r.height) * 2 + 1;
    ray.setFromCamera(ndc, camera);
    return ray.intersectObjects(S.rayTargets, true)[0] || null;
  }

  function pointOnDragPlane(e) {
    const r = renderer.domElement.getBoundingClientRect();
    ndc.x = ((e.clientX - r.left) / r.width) * 2 - 1;
    ndc.y = -((e.clientY - r.top) / r.height) * 2 + 1;
    ray.setFromCamera(ndc, camera);
    return ray.ray.intersectPlane(dragPlane, dragHit) ? dragHit : null;
  }

  function setDraftPose(yaw) {
    const pose = {
      x: poseDrag.point.x,
      y: poseDrag.point.y,
      z: poseDrag.point.z,
      yaw: normalizeYaw(yaw),
    };
    pose.layer = yawToLayer(pose.yaw);
    if (poseDrag.target === "start") startPt = pose;
    else goalPt = pose;
    replan();
  }

  // ── UI wiring ───────────────────────────────────────────────────
  $("route").addEventListener("input", (e) => {
    if (!S) return;
    S.playing = false;
    S.t = Number(e.target.value) / 1000;
    setRobotAt(S.t);
    updateRouteControls();
  });
  $("play").addEventListener("click", () => {
    if (!S || !S.route) return;
    S.playing = !S.playing;
    S.playLastMs = null;
    updateRouteControls();
  });
  $("play-rate").addEventListener("click", () => {
    if (!S) return;
    const rates = [1, 2, 4, 8, 16];
    const i = rates.indexOf(S.playRate);
    S.playRate = rates[(i + 1) % rates.length];
    S.playLastMs = null;
    updateRouteControls();
  });
  $("reset-view").addEventListener("click", frameView);
  $("reset-query").addEventListener("click", () => { setQuery(S.scene.start, S.scene.goal); });
  $("pose-tool-start").addEventListener("click", () => armPoseTool("start"));
  $("pose-tool-goal").addEventListener("click", () => armPoseTool("goal"));
  document.querySelectorAll("#asa-stage-controls [data-asa-stage]").forEach((btn) => {
    btn.addEventListener("click", () => {
      if (!S) return;
      const idx = Number(btn.dataset.asaStage);
      S.asaStage = idx;
      setAsaPipelineVisible(true);
      showAsaStage(S.asaStage);
    });
  });

  bindToggle("toggle-mesh", () => S.env);
  bindToggle("toggle-cells", () => S.cells.mesh);
  bindToggle("toggle-polys", () => S.polyGraph ? S.polyGraph.group : null);
  bindToggle("toggle-path", () => null, () => showAsaStage(S.asaStage));
  bindToggle("toggle-robot", () => S.robot);
  function applyToggles() {
    setToggleVisible("toggle-mesh", () => S.env);
    setToggleVisible("toggle-cells", () => S.cells.mesh);
    setToggleVisible("toggle-polys", () => S.polyGraph ? S.polyGraph.group : null);
    showAsaStage(S.asaStage);
    setToggleVisible("toggle-robot", () => S.robot);
  }
  function bindToggle(id, get, afterChange) {
    const cb = $(id); if (!cb) return;
    cb.addEventListener("change", () => {
      setToggleVisible(id, get);
      if (afterChange) afterChange();
    });
  }
  function setToggleVisible(id, get) {
    const cb = $(id), o = get();
    if (cb && o) o.visible = cb.checked;
  }
  function asaPipelineVisible() {
    const cb = $("toggle-path");
    return !cb || cb.checked;
  }
  function setAsaPipelineVisible(visible) {
    const cb = $("toggle-path");
    if (cb) cb.checked = visible;
  }

  function writePlanDebug(debug) {
    const el = $("plan-debug-json");
    if (el) el.textContent = debug ? JSON.stringify(debug) : "";
  }

  function pathHeightOffset() {
    const h = S && S.scene && S.scene.agent ? Number(S.scene.agent.height) : NaN;
    return Number.isFinite(h) ? h * 0.5 : 0.07;
  }

  function armPoseTool(target) {
    if (armedPoseTool === target) {
      disarmPoseTool((target === "start" ? "Set Start" : "Set Goal") + " cancelled. Click Set Start or Set Goal to arm a pose edit.");
      return;
    }

    armedPoseTool = target;
    const start = $("pose-tool-start"), goal = $("pose-tool-goal");
    if (start) {
      start.classList.toggle("is-armed", target === "start");
      start.textContent = target === "start" ? "Setting Start..." : "Set Start";
    }
    if (goal) {
      goal.classList.toggle("is-armed", target === "goal");
      goal.textContent = target === "goal" ? "Setting Goal..." : "Set Goal";
    }
    setHint((target === "start" ? "Set Start" : "Set Goal") + " armed. Left-drag once on the scene to set position and yaw, or click the button again to cancel.");
  }

  function disarmPoseTool(message) {
    armedPoseTool = null;
    const start = $("pose-tool-start"), goal = $("pose-tool-goal");
    if (start) {
      start.classList.remove("is-armed");
      start.textContent = "Set Start";
    }
    if (goal) {
      goal.classList.remove("is-armed");
      goal.textContent = "Set Goal";
    }
    if (message) setHint(message);
  }

  function frameView() {
    const d = S.radius * 2.0;
    camera.position.set(S.center.x + d*0.65, S.center.y - d*0.8, S.center.z + d*0.8);
    controls.target.copy(S.center); controls.update();
  }

  // ── resize + animation loop ─────────────────────────────────────
  function resize() {
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (!w || !h) return;
    renderer.setSize(w, h, false);
    camera.aspect = w / h; camera.updateProjectionMatrix();
  }
  new ResizeObserver(resize).observe(canvas);
  window.addEventListener("resize", resize); resize();

  function tick() {
    if (S && S.playing && S.route) {
      const now = performance.now();
      const dt = S.playLastMs == null ? 0 : (now - S.playLastMs) / 1000;
      S.playLastMs = now;
      const duration = S.routeTimeline && S.routeTimeline.total ? S.routeTimeline.total : 4.0;
      S.t += dt * (S.playRate || 1) / duration; if (S.t > 1) S.t = 0;
      const slider = $("route");
      if (slider) slider.value = String(Math.round(S.t * 1000));
      setRobotAt(S.t);
    }
    controls.update();
    renderer.render(world, camera);
    requestAnimationFrame(tick);
  }
  tick();

  // ── small helpers ───────────────────────────────────────────────
  function poseArrow(p, color) {
    const len = Math.max(0.55, Math.min(1.0, (S.scene.agent.length || 0.8) * 0.9));
    const yaw = p.yaw || 0;
    const zOffset = (S.scene.agent.height || 0.32) * 0.5;
    const base = new THREE.Vector3(p.x, p.y, p.z + zOffset);
    const headLen = len * 0.3;
    const shaftLen = len - headLen;
    const shaftRadius = Math.max(0.035, len * 0.045);
    const headRadius = Math.max(0.095, len * 0.12);
    const mat = new THREE.MeshStandardMaterial({
      color,
      emissive: color,
      emissiveIntensity: 0.18,
      roughness: 0.38,
      metalness: 0.0,
      depthWrite: false,
    });

    const g = new THREE.Group();
    const shaft = new THREE.Mesh(new THREE.CylinderGeometry(shaftRadius, shaftRadius, shaftLen, 18), mat);
    shaft.rotation.z = -Math.PI / 2;
    shaft.position.set(shaftLen * 0.5, 0, 0);

    const head = new THREE.Mesh(new THREE.ConeGeometry(headRadius, headLen, 24), mat);
    head.rotation.z = -Math.PI / 2;
    head.position.set(shaftLen + headLen * 0.5, 0, 0);

    g.position.copy(base);
    g.rotation.z = yaw;
    g.add(shaft, head);
    g.renderOrder = 5;
    return g;
  }

  function poseFromScenePoint(p, fallbackYaw) {
    const yaw = p.yaw !== undefined ? p.yaw :
      (p.layer !== undefined ? (p.layer - 1) * S.field.yawStep : fallbackYaw);
    return { x: p.x, y: p.y, z: p.z, yaw: normalizeYaw(yaw), layer: p.layer || yawToLayer(yaw) };
  }

  function currentToolPoseYaw() {
    const p = armedPoseTool === "start" ? startPt : goalPt;
    return p && p.yaw !== undefined ? p.yaw : 0;
  }

  function normalizeYaw(yaw) {
    const tau = 2 * Math.PI;
    return ((yaw % tau) + tau) % tau;
  }

  function yawToLayer(yaw) {
    if (!S || !S.field || !S.field.yawStep) return 1;
    return Math.round(normalizeYaw(yaw) / S.field.yawStep) % S.field.nLayers + 1;
  }
  function buildRobot(agent) {
    const length = Number(agent && agent.length) || 0.8;
    const width = Number(agent && agent.width) || 0.5;
    const height = Number(agent && agent.height) || 0.3;
    const g = new THREE.Group();
    const box = new THREE.BoxGeometry(length, width, height);
    g.add(new THREE.Mesh(box, new THREE.MeshStandardMaterial({
      color: COL.robot, transparent: true, opacity: 0.16, roughness: 0.5, depthWrite: false })));
    g.add(new THREE.LineSegments(new THREE.EdgesGeometry(box),
      new THREE.LineBasicMaterial({ color: COL.robot })));
    const cone = new THREE.Mesh(new THREE.ConeGeometry(Math.max(0.06, width * 0.12), Math.max(0.18, length * 0.24), 14),
      new THREE.MeshStandardMaterial({ color: COL.path, roughness: 0.4 }));
    cone.rotation.z = -Math.PI / 2; cone.position.set(length / 2 + Math.max(0.1, length * 0.14), 0, 0);
    g.add(cone); g.renderOrder = 5;
    return g;
  }
  function polyline(pts, color, radius, opacity, emissive) {
    const v = pts.map((p) => new THREE.Vector3(p[0], p[1], p[2]));
    const curve = new THREE.CatmullRomCurve3(v, false, "catmullrom", 0.2);
    const tube = new THREE.TubeGeometry(curve, Math.max(24, v.length * 2), radius, 10, false);
    return new THREE.Mesh(tube, new THREE.MeshStandardMaterial({
      color, roughness: 0.4, transparent: opacity < 1, opacity,
      emissive: emissive ? color : 0x000000, emissiveIntensity: emissive ? 0.5 : 0 }));
  }
  function setHint(t) { const e = $("query-hint"); if (e) e.textContent = t; }
  function setLoading(t) {
    const e = $("viewer-loading"); if (!e) return;
    if (t) { e.style.display = "block"; e.textContent = t; } else e.style.display = "none";
  }
}

function shortestAngle(a, b) {
  let d = (b - a) % (2 * Math.PI);
  if (d > Math.PI) d -= 2 * Math.PI;
  if (d < -Math.PI) d += 2 * Math.PI;
  return d;
}

function disposeGroup(g) {
  g.traverse((o) => {
    if (o.geometry) o.geometry.dispose();
    if (o.material) (Array.isArray(o.material) ? o.material : [o.material]).forEach((m) => {
      for (const k in m) { const v = m[k]; if (v && v.isTexture) v.dispose(); }
      m.dispose();
    });
  });
}

// binary min-heap keyed by float priority, storing integer node ids
class MinHeap {
  constructor() { this.k = []; this.p = []; }
  get size() { return this.k.length; }
  push(key, pri) {
    const k = this.k, p = this.p; let i = k.length; k.push(key); p.push(pri);
    while (i > 0) { const par = (i - 1) >> 1; if (p[par] <= p[i]) break;
      swap(k, i, par); swap(p, i, par); i = par; }
  }
  pop() {
    const k = this.k, p = this.p, top = k[0], n = k.length - 1;
    k[0] = k[n]; p[0] = p[n]; k.pop(); p.pop();
    let i = 0; const len = k.length;
    while (true) {
      let l = 2*i+1, r = 2*i+2, m = i;
      if (l < len && p[l] < p[m]) m = l;
      if (r < len && p[r] < p[m]) m = r;
      if (m === i) break; swap(k, i, m); swap(p, i, m); i = m;
    }
    return top;
  }
}
function swap(a, i, j) { const t = a[i]; a[i] = a[j]; a[j] = t; }

function showError(err) {
  console.error(err);
  const detail = err && (err.stack || err.message || String(err)) || "Unknown explorer error.";
  const e = document.getElementById("viewer-loading");
  if (e) {
    e.style.display = "block";
    e.style.color = "#fca5a5";
    e.textContent = "Could not load the explorer.";
    e.title = detail;
  }
  const debug = document.getElementById("plan-debug-json");
  if (debug) debug.textContent = JSON.stringify({ error: detail });
}
