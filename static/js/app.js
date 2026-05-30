const SCENE_URL = "./static/scenes/00114-hard/scene.json";

const state = {
  scene: null,
  mesh: null,
  navmesh: null,
  triangles: [],
  showMesh: true,
  showNavmesh: true,
  showPath: true,
  showFootprints: true,
  showSearch: false,
  layer: 31,
  playing: false,
  playback: 0,
  yaw: -0.72,
  pitch: 0.82,
  scale: 1,
  panX: 0,
  panY: 0,
  center: [0, 0, 0],
  dragging: false,
  panning: false,
  dragLast: null,
  dpr: 1,
};

function byId(id) {
  return document.getElementById(id);
}

function layerToYaw(layer) {
  return state.scene ? (layer - 1) * state.scene.yawLayerStepRad : 0;
}

function formatYaw(layer) {
  return `${layerToYaw(layer).toFixed(2)} rad`;
}

function initNavbar() {
  document.querySelectorAll(".navbar-burger").forEach((burger) => {
    burger.addEventListener("click", () => {
      const target = byId(burger.dataset.target);
      burger.classList.toggle("is-active");
      if (target) target.classList.toggle("is-active");
    });
  });
}

function resizeCanvas(canvas, ctx) {
  const rect = canvas.getBoundingClientRect();
  state.dpr = Math.min(window.devicePixelRatio || 1, 2);
  canvas.width = Math.max(1, Math.floor(rect.width * state.dpr));
  canvas.height = Math.max(1, Math.floor(rect.height * state.dpr));
  ctx.setTransform(state.dpr, 0, 0, state.dpr, 0, 0);
}

function computeSceneCenter(mesh) {
  const min = mesh.bounds.min;
  const max = mesh.bounds.max;
  state.center = [
    (min[0] + max[0]) * 0.5,
    (min[1] + max[1]) * 0.5,
    (min[2] + max[2]) * 0.5,
  ];
}

function fitView(canvas) {
  if (!state.mesh) return;
  const min = state.mesh.bounds.min;
  const max = state.mesh.bounds.max;
  const sx = Math.max(1, max[0] - min[0]);
  const sy = Math.max(1, max[1] - min[1]);
  const sz = Math.max(1, max[2] - min[2]);
  const span = Math.max(sx, sy, sz * 2.5);
  state.scale = Math.min(canvas.clientWidth, canvas.clientHeight) / span * 0.82;
  state.panX = canvas.clientWidth * 0.5;
  state.panY = canvas.clientHeight * 0.54;
}

function makeTriangles(mesh) {
  const pos = mesh.positions;
  const tris = [];
  for (let i = 0; i < pos.length; i += 9) {
    tris.push([
      [pos[i], pos[i + 1], pos[i + 2]],
      [pos[i + 3], pos[i + 4], pos[i + 5]],
      [pos[i + 6], pos[i + 7], pos[i + 8]],
    ]);
  }
  return tris;
}

function project(point) {
  const dx = point[0] - state.center[0];
  const dy = point[1] - state.center[1];
  const dz = point[2] - state.center[2];
  const cy = Math.cos(state.yaw);
  const sy = Math.sin(state.yaw);
  const cp = Math.cos(state.pitch);
  const sp = Math.sin(state.pitch);

  const rx = dx * cy - dy * sy;
  const ry = dx * sy + dy * cy;
  const rz = dz;
  const py = ry * cp - rz * sp;
  const pz = ry * sp + rz * cp;

  return {
    x: rx * state.scale + state.panX,
    y: -pz * state.scale + state.panY,
    depth: py,
    height: point[2],
  };
}

function drawBackground(ctx, canvas) {
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  const gradient = ctx.createLinearGradient(0, 0, 0, h);
  gradient.addColorStop(0, "#0f172a");
  gradient.addColorStop(1, "#111827");
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, w, h);

  ctx.strokeStyle = "rgba(255,255,255,0.055)";
  ctx.lineWidth = 1;
  const step = 48;
  for (let x = 0; x < w; x += step) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, h);
    ctx.stroke();
  }
  for (let y = 0; y < h; y += step) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
  }
}

function drawMesh(ctx) {
  if (!state.showMesh || !state.mesh) return;
  const zMin = state.mesh.bounds.min[2];
  const zMax = state.mesh.bounds.max[2];
  const zRange = Math.max(0.001, zMax - zMin);

  const projected = state.triangles.map((tri) => {
    const p0 = project(tri[0]);
    const p1 = project(tri[1]);
    const p2 = project(tri[2]);
    return {
      p0,
      p1,
      p2,
      depth: (p0.depth + p1.depth + p2.depth) / 3,
      height: (p0.height + p1.height + p2.height) / 3,
    };
  });

  projected.sort((a, b) => a.depth - b.depth);
  ctx.lineWidth = 0.35;
  for (const tri of projected) {
    const t = Math.max(0, Math.min(1, (tri.height - zMin) / zRange));
    const r = Math.round(72 + t * 92);
    const g = Math.round(91 + t * 112);
    const b = Math.round(112 + t * 126);
    ctx.beginPath();
    ctx.moveTo(tri.p0.x, tri.p0.y);
    ctx.lineTo(tri.p1.x, tri.p1.y);
    ctx.lineTo(tri.p2.x, tri.p2.y);
    ctx.closePath();
    ctx.fillStyle = `rgba(${r}, ${g}, ${b}, 0.18)`;
    ctx.fill();
  }
}

function navmeshColor(kind) {
  if (kind === "updated") return {fill: "rgba(182, 255, 116, 0.78)", stroke: "rgba(221, 255, 184, 0.96)"};
  if (kind === "restricted") return {fill: "rgba(255, 214, 92, 0.68)", stroke: "rgba(255, 235, 164, 0.98)"};
  return {fill: "rgba(166, 220, 253, 0.70)", stroke: "rgba(232, 248, 255, 0.96)"};
}

function drawNavmesh(ctx) {
  if (!state.showNavmesh || !state.navmesh) return;
  const projected = state.navmesh.polygons.map((poly) => {
    const verts = poly.vertices.map((v) => project([v.x, v.y, v.z]));
    const depth = verts.reduce((sum, v) => sum + v.depth, 0) / Math.max(1, verts.length);
    return {poly, verts, depth};
  }).sort((a, b) => a.depth - b.depth);

  ctx.save();
  ctx.lineJoin = "round";
  for (const item of projected) {
    if (item.verts.length < 3) continue;
    const color = navmeshColor(item.poly.kind);
    ctx.beginPath();
    item.verts.forEach((p, index) => {
      if (index === 0) ctx.moveTo(p.x, p.y);
      else ctx.lineTo(p.x, p.y);
    });
    ctx.closePath();
    ctx.fillStyle = color.fill;
    ctx.strokeStyle = color.stroke;
    ctx.lineWidth = 2.1;
    ctx.fill();
    ctx.stroke();
  }
  ctx.restore();
}

function pathPoints() {
  return state.scene.paths.configuredQueryDraft;
}

function interpolatePath(points, t) {
  const clamped = Math.max(0, Math.min(1, t));
  const scaled = clamped * (points.length - 1);
  const idx = Math.min(points.length - 2, Math.floor(scaled));
  const local = scaled - idx;
  const a = points[idx];
  const b = points[idx + 1];
  return {
    x: a.x + (b.x - a.x) * local,
    y: a.y + (b.y - a.y) * local,
    z: a.z + (b.z - a.z) * local,
    layer: Math.round(a.layer + (b.layer - a.layer) * local),
  };
}

function drawPolyline(ctx, points, color, width, alpha) {
  if (!points.length) return;
  ctx.save();
  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  ctx.strokeStyle = color;
  ctx.globalAlpha = alpha;
  ctx.lineWidth = width;
  ctx.beginPath();
  points.forEach((point, index) => {
    const p = project([point.x, point.y, point.z + 0.18]);
    if (index === 0) ctx.moveTo(p.x, p.y);
    else ctx.lineTo(p.x, p.y);
  });
  ctx.stroke();
  ctx.restore();
}

function drawMarker(ctx, point, color, label) {
  const p = project([point.x, point.y, point.z + 0.25]);
  ctx.save();
  ctx.fillStyle = color;
  ctx.strokeStyle = "#fff";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(p.x, p.y, 7, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  ctx.font = "700 12px 'Noto Sans', sans-serif";
  ctx.fillStyle = "rgba(255,255,255,0.94)";
  ctx.fillText(label, p.x + 11, p.y - 9);
  ctx.restore();
}

function drawFootprint(ctx, pose, layer, alpha) {
  const agent = state.scene.agent;
  const yaw = layerToYaw(layer);
  const l = agent.length * 0.5;
  const w = agent.width * 0.5;
  const corners = [
    [l, w],
    [l, -w],
    [-l, -w],
    [-l, w],
  ].map(([x, y]) => {
    const c = Math.cos(yaw);
    const s = Math.sin(yaw);
    return [
      pose.x + x * c - y * s,
      pose.y + x * s + y * c,
      pose.z + 0.12,
    ];
  }).map(project);

  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.fillStyle = "rgba(34, 160, 107, 0.18)";
  ctx.strokeStyle = "#22a06b";
  ctx.lineWidth = 2;
  ctx.beginPath();
  corners.forEach((p, index) => {
    if (index === 0) ctx.moveTo(p.x, p.y);
    else ctx.lineTo(p.x, p.y);
  });
  ctx.closePath();
  ctx.fill();
  ctx.stroke();

  const nose = project([
    pose.x + l * Math.cos(yaw),
    pose.y + l * Math.sin(yaw),
    pose.z + 0.16,
  ]);
  const center = project([pose.x, pose.y, pose.z + 0.16]);
  ctx.strokeStyle = "#ef5b45";
  ctx.beginPath();
  ctx.moveTo(center.x, center.y);
  ctx.lineTo(nose.x, nose.y);
  ctx.stroke();
  ctx.restore();
}

function drawSearchPreview(ctx, points) {
  if (!state.showSearch) return;
  ctx.save();
  ctx.fillStyle = "rgba(47, 111, 237, 0.11)";
  ctx.strokeStyle = "rgba(47, 111, 237, 0.24)";
  ctx.lineWidth = 1;
  points.forEach((point, index) => {
    if (index % 2 === 1) return;
    const p = project([point.x, point.y, point.z + 0.1]);
    const r = 13 + index * 1.2;
    ctx.beginPath();
    ctx.ellipse(p.x, p.y, r * 1.35, r * 0.62, -0.2, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
  });
  ctx.restore();
}

function drawPath(ctx) {
  if (!state.scene) return;
  const points = pathPoints();
  drawSearchPreview(ctx, points);

  if (state.showPath) {
    drawPolyline(ctx, points, "rgba(255,255,255,0.72)", 8, 0.8);
    drawPolyline(ctx, points, "#ef5b45", 3, 1);
  }

  if (state.showFootprints) {
    for (let i = 0; i < points.length; i += 2) {
      drawFootprint(ctx, points[i], points[i].layer, 0.38);
    }
    const live = interpolatePath(points, state.playback);
    drawFootprint(ctx, live, state.playing ? live.layer : state.layer, 0.95);
  }

  drawMarker(ctx, state.scene.start, "#22a06b", "start");
  drawMarker(ctx, state.scene.goal, "#ef5b45", "goal");
}

function draw(ctx, canvas) {
  drawBackground(ctx, canvas);
  drawMesh(ctx);
  drawNavmesh(ctx);
  drawPath(ctx);
}

function updateLabels() {
  byId("layer-value").textContent = String(state.layer);
  byId("yaw-value").textContent = formatYaw(state.layer);
}

function setupControls(canvas, ctx) {
  const layer = byId("layer");
  const play = byId("play");
  const reset = byId("reset");

  layer.addEventListener("input", () => {
    state.layer = Number(layer.value);
    state.playing = false;
    play.textContent = "Play";
    updateLabels();
    draw(ctx, canvas);
  });

  play.addEventListener("click", () => {
    state.playing = !state.playing;
    play.textContent = state.playing ? "Pause" : "Play";
  });

  reset.addEventListener("click", () => {
    state.playing = false;
    state.playback = 0;
    play.textContent = "Play";
    state.yaw = -0.72;
    state.pitch = 0.82;
    fitView(canvas);
    draw(ctx, canvas);
  });

  [
    ["toggle-mesh", "showMesh"],
    ["toggle-navmesh", "showNavmesh"],
    ["toggle-path", "showPath"],
    ["toggle-footprints", "showFootprints"],
    ["toggle-search", "showSearch"],
  ].forEach(([id, key]) => {
    byId(id).addEventListener("change", (event) => {
      state[key] = event.target.checked;
      draw(ctx, canvas);
    });
  });
}

function setupPointer(canvas, ctx) {
  canvas.addEventListener("mousedown", (event) => {
    state.dragging = true;
    state.panning = event.shiftKey || event.button === 2;
    state.dragLast = {x: event.clientX, y: event.clientY};
  });

  canvas.addEventListener("contextmenu", (event) => event.preventDefault());

  window.addEventListener("mouseup", () => {
    state.dragging = false;
  });

  window.addEventListener("mousemove", (event) => {
    if (!state.dragging || !state.dragLast) return;
    const dx = event.clientX - state.dragLast.x;
    const dy = event.clientY - state.dragLast.y;
    state.dragLast = {x: event.clientX, y: event.clientY};

    if (state.panning) {
      state.panX += dx;
      state.panY += dy;
    } else {
      state.yaw += dx * 0.008;
      state.pitch = Math.max(0.2, Math.min(1.28, state.pitch + dy * 0.006));
    }
    draw(ctx, canvas);
  });

  canvas.addEventListener("wheel", (event) => {
    event.preventDefault();
    state.scale = Math.max(8, Math.min(460, state.scale * (event.deltaY < 0 ? 1.12 : 0.89)));
    draw(ctx, canvas);
  }, {passive: false});
}

async function loadScene(canvas, ctx) {
  const scene = await fetch(SCENE_URL).then((response) => response.json());
  const mesh = await fetch(`./static/scenes/00114-hard/${scene.meshPreview}`).then((response) => response.json());
  const navmesh = await fetch(`./static/scenes/00114-hard/${scene.navmeshOverlay}`).then((response) => response.json());
  state.scene = scene;
  state.mesh = mesh;
  state.navmesh = navmesh;
  state.triangles = makeTriangles(mesh);
  state.layer = scene.start.layer;
  computeSceneCenter(mesh);
  fitView(canvas);

  byId("layer").max = String(scene.yawLayers);
  byId("layer").value = String(state.layer);
  byId("tri-count").textContent = mesh.triangleCount.toLocaleString();
  byId("path-count").textContent = scene.paths.configuredQueryDraft.length.toLocaleString();
  byId("case-name").textContent = scene.name;
  byId("path-note").textContent = `${scene.notes[1]} ${scene.notes[2] || ""} ${scene.notes[3] || ""}`.trim();
  updateLabels();
  draw(ctx, canvas);
}

function animationLoop(ctx, canvas) {
  if (state.playing) {
    state.playback += 0.004;
    if (state.playback > 1) state.playback = 0;
    draw(ctx, canvas);
  }
  requestAnimationFrame(() => animationLoop(ctx, canvas));
}

document.addEventListener("DOMContentLoaded", () => {
  initNavbar();
  const canvas = byId("viewer");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");

  const resize = () => {
    resizeCanvas(canvas, ctx);
    fitView(canvas);
    draw(ctx, canvas);
  };

  window.addEventListener("resize", resize);
  resizeCanvas(canvas, ctx);
  setupControls(canvas, ctx);
  setupPointer(canvas, ctx);

  loadScene(canvas, ctx)
    .then(() => animationLoop(ctx, canvas))
    .catch((error) => {
      console.error(error);
      drawBackground(ctx, canvas);
      ctx.fillStyle = "white";
      ctx.font = "16px 'Noto Sans', sans-serif";
      ctx.fillText("Could not load scene assets.", 24, 34);
    });
});
