/* Interactive SE(2) NavMesh explorer — Three.js WebGL viewer.
 *
 * Renders the real ROS-exported assets for the hard staircase query 00114:
 *   - planning-surface mesh crop  (local_planning_surface.json)
 *   - SE(2) NavMesh polygons       (navmesh_overlay.json: full + pathUsed)
 *   - ASA route                    (scene.json paths.plannerQuery)
 * with an animated robot footprint whose heading follows the planned yaw.
 */
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const BASE = "./static/scenes/00114-hard/";
const $ = (id) => document.getElementById(id);

const COL = {
  bg: 0x0e1626,
  surfaceLow: 0x3a4963,
  surfaceHigh: 0xa6b8d6,
  navmesh: 0x6db8f2,
  search: 0xffce4d,
  path: 0xff6a4d,
  start: 0x2bb673,
  goal: 0xff5b45,
  robot: 0x2bb673
};

const canvas = $("viewer");
if (canvas) boot().catch(showError);

async function boot() {
  const [scene, surf, overlay] = await Promise.all([
    fetch(BASE + "scene.json").then((r) => r.json()),
    fetch(BASE + "local_planning_surface.json").then((r) => r.json()),
    fetch(BASE + "navmesh_overlay.json").then((r) => r.json())
  ]);

  const route = scene.paths.plannerQuery || [];
  const agent = scene.agent;
  const step = scene.yawLayerStepRad;

  // ── renderer / scene / camera ───────────────────────────────────
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));

  const world = new THREE.Scene();
  world.background = new THREE.Color(COL.bg);

  const camera = new THREE.PerspectiveCamera(45, 1, 0.05, 500);
  camera.up.set(0, 0, 1); // data is z-up

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.rotateSpeed = 0.85;

  // ── lights ──────────────────────────────────────────────────────
  world.add(new THREE.HemisphereLight(0xd4e0f3, 0x141b29, 1.05));
  const key = new THREE.DirectionalLight(0xffffff, 1.5);
  const fill = new THREE.DirectionalLight(0x9fb3d6, 0.5);
  world.add(key, fill);

  // ── bounds / center ─────────────────────────────────────────────
  const bmin = surf.bounds.min.slice();
  const bmax = surf.bounds.max.slice();
  for (const p of overlay.polygons) {
    for (const v of p.vertices) {
      bmin[0] = Math.min(bmin[0], v.x); bmax[0] = Math.max(bmax[0], v.x);
      bmin[1] = Math.min(bmin[1], v.y); bmax[1] = Math.max(bmax[1], v.y);
      bmin[2] = Math.min(bmin[2], v.z); bmax[2] = Math.max(bmax[2], v.z);
    }
  }
  const center = new THREE.Vector3(
    (bmin[0] + bmax[0]) / 2, (bmin[1] + bmax[1]) / 2, (bmin[2] + bmax[2]) / 2
  );
  const radius = Math.hypot(bmax[0] - bmin[0], bmax[1] - bmin[1], bmax[2] - bmin[2]) / 2;

  key.position.set(center.x + radius, center.y - radius * 0.5, center.z + radius * 2);
  fill.position.set(center.x - radius, center.y + radius, center.z + radius);

  // ── surface mesh (height-shaded) ────────────────────────────────
  const surfMesh = buildSurface(surf);
  world.add(surfMesh);

  // ── navmesh overlays ────────────────────────────────────────────
  const navFull = buildNavmesh(overlay.polygons, "full", COL.navmesh, 0.20, 0.015);
  const navUsed = buildNavmesh(overlay.polygons, "pathUsed", COL.search, 0.5, 0.03);
  world.add(navFull, navUsed);

  // ── path tube ───────────────────────────────────────────────────
  const pathGroup = new THREE.Group();
  if (route.length > 1) {
    const pts = route.map((p) => new THREE.Vector3(p.x, p.y, p.z + 0.1));
    const curve = new THREE.CatmullRomCurve3(pts, false, "catmullrom", 0.25);
    const tube = new THREE.TubeGeometry(curve, Math.max(80, route.length * 2), 0.07, 12, false);
    const pathMesh = new THREE.Mesh(tube, new THREE.MeshStandardMaterial({
      color: COL.path, roughness: 0.4, metalness: 0.0, emissive: 0xc23414, emissiveIntensity: 0.55
    }));
    pathMesh.renderOrder = 3;
    pathGroup.add(pathMesh);
  }
  world.add(pathGroup);

  // ── start / goal markers ────────────────────────────────────────
  const markers = new THREE.Group();
  markers.add(marker(scene.start, COL.start));
  markers.add(marker(scene.goal, COL.goal));
  world.add(markers);

  // ── ghost footprints + live robot ───────────────────────────────
  const ghosts = new THREE.Group();
  for (let i = 0; i < route.length; i += 6) ghosts.add(footprintOutline(route[i]));
  world.add(ghosts);

  const robot = buildRobot();
  world.add(robot);

  // ── stats ───────────────────────────────────────────────────────
  $("stat-tris").textContent = (surf.triangleCount || 0).toLocaleString();
  $("stat-polys").textContent = overlay.polygons.length.toLocaleString();
  $("stat-states").textContent = route.length.toLocaleString();

  // ── camera framing ──────────────────────────────────────────────
  function frame() {
    const d = radius * 2.1;
    camera.position.set(center.x + d * 0.7, center.y - d * 0.75, center.z + d * 0.7);
    controls.target.copy(center);
    controls.update();
  }
  frame();

  // ── robot placement along route ─────────────────────────────────
  function setRobot(t) {
    if (route.length < 2) return;
    const f = t * (route.length - 1);
    const i = Math.min(route.length - 2, Math.floor(f));
    const k = f - i;
    const a = route[i], b = route[i + 1];
    const layer = k < 0.5 ? a.layer : b.layer;
    const yaw = (layer - 1) * step;
    robot.position.set(
      a.x + (b.x - a.x) * k, a.y + (b.y - a.y) * k, a.z + (b.z - a.z) * k + 0.08
    );
    robot.rotation.set(0, 0, yaw);
    let deg = ((yaw * 180 / Math.PI) % 360 + 360) % 360;
    $("stat-yaw").textContent = yaw.toFixed(2) + " rad (" + Math.round(deg) + "°)";
  }
  setRobot(0);

  // ── controls / UI ───────────────────────────────────────────────
  const routeSlider = $("route");
  const playBtn = $("play");
  let playing = false, param = 0;

  routeSlider.addEventListener("input", () => {
    playing = false; playBtn.textContent = "Play route";
    param = Number(routeSlider.value) / 1000;
    setRobot(param);
  });
  playBtn.addEventListener("click", () => {
    playing = !playing;
    playBtn.textContent = playing ? "Pause" : "Play route";
  });
  $("reset").addEventListener("click", frame);

  bindToggle("toggle-surface", surfMesh);
  bindToggle("toggle-navmesh", navFull);
  bindToggle("toggle-search", navUsed);
  bindToggle("toggle-path", pathGroup);
  bindToggle("toggle-robot", robot, ghosts);

  // ── resize ──────────────────────────────────────────────────────
  function resize() {
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (!w || !h) return;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
  new ResizeObserver(resize).observe(canvas);
  window.addEventListener("resize", resize);
  resize();

  const loading = $("viewer-loading");
  if (loading) loading.style.display = "none";

  // ── animation loop ──────────────────────────────────────────────
  function tick() {
    if (playing) {
      param += 0.0035;
      if (param > 1) param = 0;
      routeSlider.value = String(Math.round(param * 1000));
      setRobot(param);
    }
    controls.update();
    renderer.render(world, camera);
    requestAnimationFrame(tick);
  }
  tick();

  // ── helpers (closures over THREE) ───────────────────────────────
  function buildSurface(s) {
    const pos = new Float32Array(s.positions);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.BufferAttribute(pos, 3));
    geo.computeVertexNormals();
    const zmin = s.bounds.min[2], zr = Math.max(1e-3, s.bounds.max[2] - s.bounds.min[2]);
    const colors = new Float32Array(pos.length);
    const lo = new THREE.Color(COL.surfaceLow), hi = new THREE.Color(COL.surfaceHigh), c = new THREE.Color();
    for (let i = 0; i < pos.length; i += 3) {
      c.copy(lo).lerp(hi, Math.min(1, Math.max(0, (pos[i + 2] - zmin) / zr)));
      colors[i] = c.r; colors[i + 1] = c.g; colors[i + 2] = c.b;
    }
    geo.setAttribute("color", new THREE.BufferAttribute(colors, 3));
    return new THREE.Mesh(geo, new THREE.MeshStandardMaterial({
      vertexColors: true, roughness: 0.97, metalness: 0.0, side: THREE.DoubleSide
    }));
  }

  function buildNavmesh(polys, kind, color, opacity, lift) {
    const verts = [];
    for (const p of polys) {
      if (p.kind !== kind) continue;
      const v = p.vertices;
      for (let i = 1; i < v.length - 1; i++) {
        verts.push(v[0].x, v[0].y, v[0].z + lift,
                   v[i].x, v[i].y, v[i].z + lift,
                   v[i + 1].x, v[i + 1].y, v[i + 1].z + lift);
      }
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute(verts, 3));
    return new THREE.Mesh(geo, new THREE.MeshBasicMaterial({
      color, transparent: true, opacity, side: THREE.DoubleSide,
      depthWrite: false, polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2
    }));
  }

  function marker(p, color) {
    const m = new THREE.Mesh(
      new THREE.SphereGeometry(0.16, 20, 16),
      new THREE.MeshStandardMaterial({ color, roughness: 0.4, emissive: color, emissiveIntensity: 0.3 })
    );
    m.position.set(p.x, p.y, p.z + 0.16);
    m.renderOrder = 4;
    return m;
  }

  function footprintOutline(state) {
    const yaw = (state.layer - 1) * step;
    const l = agent.length / 2, w = agent.width / 2;
    const c = Math.cos(yaw), s = Math.sin(yaw);
    const pts = [[l, w], [l, -w], [-l, -w], [-l, w]].map(([x, y]) =>
      new THREE.Vector3(state.x + x * c - y * s, state.y + x * s + y * c, state.z + 0.05));
    return new THREE.LineLoop(
      new THREE.BufferGeometry().setFromPoints(pts),
      new THREE.LineBasicMaterial({ color: COL.start, transparent: true, opacity: 0.32 })
    );
  }

  function buildRobot() {
    const g = new THREE.Group();
    const box = new THREE.BoxGeometry(agent.length, agent.width, 0.16);
    g.add(new THREE.Mesh(box, new THREE.MeshStandardMaterial({
      color: COL.robot, transparent: true, opacity: 0.34, roughness: 0.55
    })));
    g.add(new THREE.LineSegments(new THREE.EdgesGeometry(box),
      new THREE.LineBasicMaterial({ color: COL.robot })));
    const cone = new THREE.Mesh(new THREE.ConeGeometry(0.09, 0.24, 14),
      new THREE.MeshStandardMaterial({ color: COL.path, roughness: 0.4 }));
    cone.rotation.z = -Math.PI / 2;            // point along +x (heading)
    cone.position.set(agent.length / 2 + 0.14, 0, 0);
    g.add(cone);
    return g;
  }

  function bindToggle(id, ...objs) {
    const cb = $(id);
    if (!cb) return;
    cb.addEventListener("change", () => objs.forEach((o) => { o.visible = cb.checked; }));
  }
}

function showError(err) {
  console.error(err);
  const loading = $("viewer-loading");
  if (loading) { loading.textContent = "Could not load the 3D scene."; loading.style.color = "#fca5a5"; }
}
