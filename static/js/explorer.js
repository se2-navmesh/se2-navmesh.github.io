/* Interactive SE(2) NavMesh Path-Planning Explorer.
 *
 * Loads, per scene:
 *   - scene.glb   textured HM3D environment, baked into the navmesh frame
 *                 (meshopt-compressed; needs MeshoptDecoder)
 *   - field.bin   SE(2) traversability field: one record per surface cell,
 *                 [f32 x, y, z, u32 mask]; mask's low `yawBits` bits are the
 *                 feasible headings over [0, pi) (footprint is 180-deg symmetric)
 *   - scene.json  agent, yaw layers, bounds, configured start/goal, ASA reference
 *
 * What it does that the old viewer didn't:
 *   - shows the real environment, not a sliver;
 *   - colors the field by traversability: safe (fits at every heading) vs
 *     restricted (fits at only some) -> makes yaw-dependent traversability
 *     tangible;
 *   - click a start and a goal and a yaw-aware path is planned in-browser by
 *     lattice A* over (cell, heading), with a directional (lateral > forward)
 *     cost so the robot prefers to face the way it travels.
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
  ref: 0x8aa0c8,
  start: 0x2bb673,
  goal: 0xff5b45,
  robot: 0x33d6c0,
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
  let pick = "start";          // which endpoint the next canvas click sets
  let startPt = null, goalPt = null;

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
    const progress = { json: 0, glb: 0, field: 0 };
    const updateProgress = () => {
      const pct = Math.min(99, Math.round(progress.json + progress.glb + progress.field));
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
        progress.glb = Math.min(80, (xhr.loaded / xhr.total) * 80);
      } else {
        progress.glb = Math.max(progress.glb, 8);
      }
      updateProgress();
    }).then((v) => { progress.glb = 80; updateProgress(); return v; });
    const fieldBin = fetch(base + "field.bin")
      .then((r) => r.arrayBuffer())
      .then((v) => { progress.field = 15; updateProgress(); return v; });
    const [scene, gltf, fieldBuf] = await Promise.all([
      sceneJson,
      sceneGltf,
      fieldBin,
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

    // planning graph over the field
    const planner = buildPlanner(field, scene);

    // path + robot + markers containers
    const pathGroup = new THREE.Group();
    const refGroup = new THREE.Group();
    const markers = new THREE.Group();
    const robot = buildRobot(scene.agent);
    group.add(pathGroup, refGroup, markers, robot);

    // reference (ROS ASA) path, faint
    if (scene.referencePath && scene.referencePath.length > 1) {
      refGroup.add(polyline(scene.referencePath.map((p) => [p.x, p.y, p.z + 0.05]),
        COL.ref, 0.025, 0.5));
    }

    S = { meta, scene, field, cells, planner, group, env, rayTargets,
          pathGroup, refGroup, markers, robot, route: null, t: 0, playing: false };

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

  // ── planner: lattice A* over (cell, heading) ────────────────────
  function buildPlanner(field, scene) {
    const cs = field.cellSize, climb = scene.agent.maxClimb || 0.25;
    const K = 100003;
    const ckey = (ix, iy) => ix * K + iy;
    const grid = new Map();
    const gix = new Int32Array(field.n), giy = new Int32Array(field.n);
    for (let i = 0; i < field.n; i++) {
      const ix = Math.round(field.px[i] / cs), iy = Math.round(field.py[i] / cs);
      gix[i] = ix; giy[i] = iy;
      const k = ckey(ix, iy);
      let a = grid.get(k); if (!a) grid.set(k, a = []); a.push(i);
    }
    // spatial neighbours (xy-adjacent, climbable z step)
    const nbr = new Array(field.n);
    for (let i = 0; i < field.n; i++) {
      const list = [];
      for (let dx = -1; dx <= 1; dx++)
        for (let dy = -1; dy <= 1; dy++) {
          if (dx === 0 && dy === 0) continue;
          const a = grid.get(ckey(gix[i] + dx, giy[i] + dy));
          if (!a) continue;
          for (const j of a) if (Math.abs(field.pz[i] - field.pz[j]) <= climb) list.push(j);
        }
      nbr[i] = list;
    }
    return { grid, ckey, gix, giy, nbr, cs, climb };
  }

  function nearestCell(field, x, y, z) {
    let best = -1, bd = Infinity;
    for (let i = 0; i < field.n; i++) {
      const dx = field.px[i] - x, dy = field.py[i] - y, dz = (field.pz[i] - z) * 1.5;
      const d = dx * dx + dy * dy + dz * dz;
      if (d < bd) { bd = d; best = i; }
    }
    return best;
  }

  function nearestFeasibleLayer(field, c, preferLayer) {
    if (feasible(field, c, preferLayer)) return preferLayer;
    for (let d = 1; d <= field.nLayers; d++) {
      const a = (preferLayer + d) % field.nLayers, b = (preferLayer - d + field.nLayers) % field.nLayers;
      if (feasible(field, c, a)) return a;
      if (feasible(field, c, b)) return b;
    }
    return -1;
  }

  // Returns array of {x,y,z,yaw} or null.
  //
  // Cost model mirrors the C++ ASA planner (se2_navmesh, testing_utils.cpp) /
  // paper (Method.tex eq. 6-7): edge cost = traversal *time*. A translation
  // holding heading psi splits the move into longitudinal / lateral components
  // and charges 1/v_long and 1/v_lat per metre; an in-place yaw step costs
  // yawStep/omega. With the default anymal params v_lat (0.1) << v_lon (0.5),
  // so sideways motion is ~5x more expensive -> the path prefers to face the
  // way it travels instead of strafing.
  function plan(startCell, startLayer, goalCell) {
    const f = S.field, P = S.planner, a = S.scene.agent || {};
    if (startCell < 0 || goalCell < 0 || startLayer < 0) return null;
    const nL = f.nLayers;
    const vLon = a.maxLonVelocity || 0.5, vLat = a.maxLatVelocity || 0.1;
    const vAng = a.maxAngVelocity || 0.5;
    const dirF = a.directionCostFactor || 1.0, turnF = a.turningCostFactor || 1.0;
    const unitLon = dirF / vLon, unitLat = dirF / vLat;  // cost per metre fwd / sideways
    const ROT = f.yawStep / vAng * turnF;                // cost per yaw-layer step
    const hUnit = Math.min(unitLon, unitLat);            // cheapest metre -> admissible h
    const id = (c, L) => c * nL + L;
    const gx = f.px[goalCell], gy = f.py[goalCell], gz = f.pz[goalCell];
    const h = (c) => Math.hypot(f.px[c] - gx, f.py[c] - gy, f.pz[c] - gz) * hUnit;

    const gScore = new Map(), came = new Map();
    const heap = new MinHeap();
    const s = id(startCell, startLayer);
    gScore.set(s, 0);
    heap.push(s, h(startCell));
    let goalNode = -1;

    while (heap.size) {
      const cur = heap.pop();
      const c = (cur / nL) | 0, L = cur % nL;
      if (c === goalCell) { goalNode = cur; break; }
      const g = gScore.get(cur);
      // rotational neighbours
      for (const dL of [1, nL - 1]) {
        const L2 = (L + dL) % nL;
        if (!feasible(f, c, L2)) continue;
        relax(cur, id(c, L2), g + ROT, c);
      }
      // translational neighbours, optionally turning by one layer while
      // stepping (so the robot can turn through a patchwork of restricted
      // cells whose feasible headings only overlap at adjacent layers)
      if (feasible(f, c, L)) {
        const list = P.nbr[c];
        for (let k = 0; k < list.length; k++) {
          const j = list[k];
          const dx = f.px[j] - f.px[c], dy = f.py[j] - f.py[c], dz = f.pz[j] - f.pz[c];
          const dist = Math.hypot(dx, dy, dz);
          // anisotropic cost of the move while holding the current heading L
          const diff = Math.atan2(dy, dx) - L * f.yawStep;
          const moveCost = dist * (Math.abs(Math.cos(diff)) * unitLon +
                                   Math.abs(Math.sin(diff)) * unitLat);
          for (const dL of [0, 1, nL - 1]) {
            const L2 = (L + dL) % nL;
            if (!feasible(f, j, L2)) continue;
            relax(cur, id(j, L2), g + moveCost + (dL === 0 ? 0 : ROT), j);
          }
        }
      }
    }
    function relax(from, to, ng, toCell) {
      const old = gScore.get(to);
      if (old !== undefined && old <= ng) return;
      gScore.set(to, ng);
      came.set(to, from);
      heap.push(to, ng + h(toCell));
    }
    if (goalNode < 0) return null;

    const seq = [];
    for (let node = goalNode; node !== undefined; node = came.get(node)) {
      const c = (node / nL) | 0, L = node % nL;
      seq.push({ x: f.px[c], y: f.py[c], z: f.pz[c], yaw: L * f.yawStep });
      if (node === s) break;
    }
    seq.reverse();
    return seq;
  }

  // ── query (start/goal) + planning ───────────────────────────────
  function setQuery(start, goal) {
    startPt = { x: start.x, y: start.y, z: start.z, layer: start.layer };
    goalPt = { x: goal.x, y: goal.y, z: goal.z };
    pick = "start";
    replan();
  }

  function replan() {
    const f = S.field;
    S.markers.clear();
    if (startPt) S.markers.add(marker(startPt, COL.start));
    if (goalPt) S.markers.add(marker(goalPt, COL.goal));

    if (!startPt || !goalPt) { drawRoute(null); return; }
    const sc = nearestCell(f, startPt.x, startPt.y, startPt.z);
    const gc = nearestCell(f, goalPt.x, goalPt.y, goalPt.z);
    // start heading: configured layer maps onto [0,2pi); snap to feasible
    const wantL = ((startPt.layer ? startPt.layer - 1 : 0) % f.nLayers + f.nLayers) % f.nLayers;
    const sL = nearestFeasibleLayer(f, sc, wantL);
    const route = plan(sc, sL, gc);
    drawRoute(route);
  }

  function drawRoute(route) {
    S.pathGroup.clear();
    S.route = route;
    if (!route || route.length < 2) {
      $("stat-len").textContent = "—";
      $("stat-cost").textContent = route ? "—" : "no path";
      setRobotAt(0);
      return;
    }
    // dedupe consecutive positions (in-place rotations repeat a point and
    // would make the Catmull-Rom tube degenerate)
    const pts = [];
    for (const p of route) {
      const v = [p.x, p.y, p.z + 0.07], last = pts[pts.length - 1];
      if (!last || Math.hypot(v[0] - last[0], v[1] - last[1], v[2] - last[2]) > 1e-4) pts.push(v);
    }
    if (pts.length > 1) S.pathGroup.add(polyline(pts, COL.path, 0.05, 1.0, true));
    // stats: split travel into forward/lateral relative to the heading held on
    // each segment, so the directional cost's effect is visible at a glance.
    let len = 0, fwd = 0, lat = 0, turn = 0;
    for (let i = 1; i < route.length; i++) {
      const dx = route[i].x - route[i-1].x, dy = route[i].y - route[i-1].y, dz = route[i].z - route[i-1].z;
      const d = Math.hypot(dx, dy, dz);
      len += d;
      if (d > 1e-9) {
        const diff = Math.atan2(dy, dx) - route[i-1].yaw;
        fwd += d * Math.abs(Math.cos(diff));
        lat += d * Math.abs(Math.sin(diff));
      }
      let a = Math.abs(route[i].yaw - route[i-1].yaw) % (2*Math.PI);
      if (a > Math.PI) a = 2*Math.PI - a;
      turn += a;
    }
    $("stat-len").textContent = len.toFixed(2) + " m";
    $("stat-cost").textContent = fwd.toFixed(2) + " m fwd · " + lat.toFixed(2) +
      " m lat · " + (turn*180/Math.PI).toFixed(0) + "° turn";
    S.t = 0; $("route").value = "0"; setRobotAt(0);
  }

  // ── robot footprint along route ─────────────────────────────────
  function setRobotAt(t) {
    if (!S || !S.robot) return;
    const route = S.route;
    if (!route || route.length < 2) { orientRobot(); return; }
    const f = t * (route.length - 1);
    const i = Math.min(route.length - 2, Math.floor(f));
    const k = f - i, a = route[i], b = route[i + 1];
    S.robot.position.set(a.x + (b.x-a.x)*k, a.y + (b.y-a.y)*k, a.z + (b.z-a.z)*k + 0.04);
    let yaw = a.yaw + shortestAngle(a.yaw, b.yaw) * k;
    S.robot.rotation.set(0, 0, yaw);
  }
  function orientRobot() {
    if (!S || !S.robot || !startPt) return;
    const f = S.field, c = nearestCell(f, startPt.x, startPt.y, startPt.z);
    const wantL = ((startPt.layer ? startPt.layer - 1 : 0) % f.nLayers + f.nLayers) % f.nLayers;
    const L = nearestFeasibleLayer(f, c, wantL);
    S.robot.position.set(f.px[c], f.py[c], f.pz[c] + 0.04);
    S.robot.rotation.set(0, 0, (L < 0 ? wantL : L) * f.yawStep);
  }

  // ── interaction: click to set start / goal ──────────────────────
  const ray = new THREE.Raycaster();
  const ndc = new THREE.Vector2();
  let dragged = false;
  renderer.domElement.addEventListener("pointerdown", () => { dragged = false; });
  renderer.domElement.addEventListener("pointermove", () => { dragged = true; });
  renderer.domElement.addEventListener("pointerup", (e) => {
    if (dragged || !S) return;
    const r = renderer.domElement.getBoundingClientRect();
    ndc.x = ((e.clientX - r.left) / r.width) * 2 - 1;
    ndc.y = -((e.clientY - r.top) / r.height) * 2 + 1;
    ray.setFromCamera(ndc, camera);
    const hit = ray.intersectObjects(S.rayTargets, true)[0];
    if (!hit) return;
    const p = hit.point;
    if (pick === "start") { startPt = { x: p.x, y: p.y, z: p.z, layer: 1 }; pick = "goal"; setHint("Click to place the goal."); }
    else { goalPt = { x: p.x, y: p.y, z: p.z }; pick = "start"; setHint("Click to set a new start."); }
    replan();
  });

  // ── UI wiring ───────────────────────────────────────────────────
  $("route").addEventListener("input", (e) => {
    S.playing = false; $("play").textContent = "Play route";
    S.t = Number(e.target.value) / 1000; setRobotAt(S.t);
  });
  $("play").addEventListener("click", () => {
    if (!S.route) return;
    S.playing = !S.playing; $("play").textContent = S.playing ? "Pause" : "Play route";
  });
  $("reset-view").addEventListener("click", frameView);
  $("reset-query").addEventListener("click", () => { setQuery(S.scene.start, S.scene.goal); setHint("Showing the configured query. Click the scene to set your own."); });

  bindToggle("toggle-mesh", () => S.env);
  bindToggle("toggle-cells", () => S.cells.mesh);
  bindToggle("toggle-path", () => S.pathGroup);
  bindToggle("toggle-robot", () => S.robot);
  bindToggle("toggle-ref", () => S.refGroup);
  function applyToggles() {
    for (const id of ["toggle-mesh","toggle-cells","toggle-path","toggle-robot","toggle-ref"]) {
      const cb = $(id); if (cb) cb.dispatchEvent(new Event("change"));
    }
  }
  function bindToggle(id, get) {
    const cb = $(id); if (!cb) return;
    cb.addEventListener("change", () => { const o = get(); if (o) o.visible = cb.checked; });
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
      S.t += 0.004; if (S.t > 1) S.t = 0;
      $("route").value = String(Math.round(S.t * 1000)); setRobotAt(S.t);
    }
    controls.update();
    renderer.render(world, camera);
    requestAnimationFrame(tick);
  }
  tick();

  // ── small helpers ───────────────────────────────────────────────
  function marker(p, color) {
    const m = new THREE.Mesh(new THREE.SphereGeometry(0.13, 18, 14),
      new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.35, roughness: 0.4 }));
    m.position.set(p.x, p.y, p.z + 0.13); m.renderOrder = 4;
    return m;
  }
  function buildRobot(agent) {
    const g = new THREE.Group();
    const box = new THREE.BoxGeometry(agent.length, agent.width, 0.14);
    g.add(new THREE.Mesh(box, new THREE.MeshStandardMaterial({
      color: COL.robot, transparent: true, opacity: 0.42, roughness: 0.5 })));
    g.add(new THREE.LineSegments(new THREE.EdgesGeometry(box),
      new THREE.LineBasicMaterial({ color: COL.robot })));
    const cone = new THREE.Mesh(new THREE.ConeGeometry(0.08, 0.22, 14),
      new THREE.MeshStandardMaterial({ color: COL.path, roughness: 0.4 }));
    cone.rotation.z = -Math.PI / 2; cone.position.set(agent.length / 2 + 0.13, 0, 0);
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
  const e = document.getElementById("viewer-loading");
  if (e) { e.style.display = "block"; e.style.color = "#fca5a5"; e.textContent = "Could not load the explorer."; }
}
