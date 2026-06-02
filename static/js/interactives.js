/* Interactive components for the SE(2) NavMesh project page.
 * - Video segmented control
 * - NavMesh vs SE(2) before/after comparison slider
 * - Yaw-feasibility demo (rotate a robot footprint in a narrow passage)
 * - ASA pathfinding stepper (A* -> string pulling -> yaw refinement)
 * - Chart.js SPC benchmark plot
 */
(function () {
  "use strict";

  const SVGNS = "http://www.w3.org/2000/svg";
  const TWO_PI = Math.PI * 2;

  function el(id) { return document.getElementById(id); }

  /* ── Mobile navbar burger ─────────────────────────────────────── */
  function setupNavbar() {
    document.querySelectorAll(".navbar-burger").forEach((burger) => {
      burger.addEventListener("click", () => {
        const target = document.getElementById(burger.dataset.target);
        burger.classList.toggle("is-active");
        if (target) target.classList.toggle("is-active");
      });
    });
  }
  function svg(tag, attrs) {
    const node = document.createElementNS(SVGNS, tag);
    for (const k in attrs) node.setAttribute(k, attrs[k]);
    return node;
  }

  /* ── Video segmented control ──────────────────────────────────── */
  function setupVideoTabs() {
    const btns = document.querySelectorAll(".seg-btn[data-video-tab]");
    const panels = document.querySelectorAll("[data-video-panel]");
    const slider = el("seg-slider");
    if (!btns.length) return;

    function positionSlider(btn) {
      if (!slider || !btn) return;
      slider.style.width = btn.offsetWidth + "px";
      slider.style.transform = "translateX(" + (btn.offsetLeft - 3) + "px)";
    }

    function show(name) {
      btns.forEach((b) => {
        const active = b.dataset.videoTab === name;
        b.classList.toggle("is-active", active);
        b.setAttribute("aria-selected", active ? "true" : "false");
        if (active) positionSlider(b);
      });
      panels.forEach((p) => {
        const active = p.dataset.videoPanel === name;
        p.classList.toggle("is-hidden", !active);
        if (!active) p.querySelectorAll("video").forEach((v) => v.pause());
      });
    }

    btns.forEach((b) => b.addEventListener("click", (e) => {
      e.preventDefault();
      show(b.dataset.videoTab);
    }));

    show("overview");
    window.addEventListener("resize", () => {
      const active = document.querySelector(".seg-btn.is-active");
      if (active) positionSlider(active);
    });
    window.addEventListener("load", () => {
      const active = document.querySelector(".seg-btn.is-active");
      if (active) positionSlider(active);
    });
  }

  /* ── Before/after comparison slider ───────────────────────────── */
  // Scenes with aligned NavMesh / SE(2) NavMesh render pairs.
  const COMPARE_SCENES = {
    garage:    { label: "Garage",    stem: "Garage",    ar: "3198 / 1965" },
    store:     { label: "Store",     stem: "Store",     ar: "3198 / 1965" },
    gym:       { label: "Gym",       stem: "Gym",       ar: "3198 / 1965" },
    forum:     { label: "Forum",     stem: "Forum",     ar: "3198 / 1965" },
    studio:    { label: "Studio",    stem: "Studio",    ar: "3198 / 1965" },
    apartment: { label: "Apartment", stem: "Apartment", ar: "3198 / 1965" },
  };

  function setupCompareSlider() {
    const range = el("compare-range");
    const top = el("compare-top");
    const handle = el("compare-handle");
    if (!range || !top || !handle) return;

    function applyWipe(v) {
      top.style.clipPath = "inset(0 " + (100 - v) + "% 0 0)";
      handle.style.left = v + "%";
    }
    range.addEventListener("input", () => applyWipe(Number(range.value)));
    applyWipe(Number(range.value));

    // Scene selector: swap the aligned NavMesh / SE(2) pair under the wipe.
    const wrap = el("compare-slider");
    const baseImg = el("compare-base-img");
    const topImg = el("compare-top-img");
    const sceneName = el("compare-scene-name");
    const sceneBtns = document.querySelectorAll(".seg-btn[data-scene-tab]");
    const sceneSlider = el("scene-seg-slider");
    if (!sceneBtns.length || !baseImg || !topImg) return;

    function positionSceneSlider(btn) {
      if (!sceneSlider || !btn) return;
      sceneSlider.style.width = btn.offsetWidth + "px";
      sceneSlider.style.transform = "translateX(" + (btn.offsetLeft - 3) + "px)";
    }

    function selectScene(id) {
      const info = COMPARE_SCENES[id];
      if (!info) return;
      baseImg.src = "./static/images/" + info.stem + "_NavMesh.png";
      baseImg.alt = info.label + " classical NavMesh traversable regions";
      topImg.src = "./static/images/" + info.stem + "_SE2NavMesh.png";
      topImg.alt = info.label + " SE(2) NavMesh traversable regions";
      if (wrap) wrap.style.aspectRatio = info.ar;
      if (sceneName) sceneName.textContent = info.label;
      sceneBtns.forEach((b) => {
        const active = b.dataset.sceneTab === id;
        b.classList.toggle("is-active", active);
        b.setAttribute("aria-selected", active ? "true" : "false");
        if (active) positionSceneSlider(b);
      });
      range.value = 50;
      applyWipe(50);
    }

    sceneBtns.forEach((b) => b.addEventListener("click", (e) => {
      e.preventDefault();
      selectScene(b.dataset.sceneTab);
    }));

    selectScene("garage");
    const repositionActive = () => {
      const active = document.querySelector(".seg-btn[data-scene-tab].is-active");
      if (active) positionSceneSlider(active);
    };
    window.addEventListener("resize", repositionActive);
    window.addEventListener("load", repositionActive);
  }

  /* ── Yaw-feasibility demo ─────────────────────────────────────── */
  const ROBOT_L = 0.93;   // ANYmal length (m)
  const ROBOT_W = 0.53;   // ANYmal width (m)
  const N_YAW = 40;       // yaw channels
  const PPM = 150;        // pixels per meter (yaw stage)

  function yawHalfExtent(theta) {
    // vertical half-extent of the rotated footprint, passage runs along x
    return (ROBOT_L / 2) * Math.abs(Math.sin(theta)) +
           (ROBOT_W / 2) * Math.abs(Math.cos(theta));
  }

  function setupYawDemo() {
    const stage = el("yaw-svg");
    const dial = el("yaw-dial");
    const yawSlider = el("yaw-slider");
    const passSlider = el("passage-slider");
    if (!stage || !yawSlider || !passSlider) return;

    const CX = 230, CY = 160;

    function passageWidth() { return Number(passSlider.value) / 100; } // m
    function channelTheta(i) { return (i / N_YAW) * TWO_PI; }

    function feasibleCount(P) {
      let n = 0;
      for (let i = 0; i < N_YAW; i++) if (yawHalfExtent(channelTheta(i)) <= P / 2) n++;
      return n;
    }

    function renderStage() {
      const P = passageWidth();
      const idx = Number(yawSlider.value);
      const theta = channelTheta(idx);
      const fits = yawHalfExtent(theta) <= P / 2;
      const gapPx = P * PPM;
      const wallTop = CY - gapPx / 2;
      const wallBot = CY + gapPx / 2;

      stage.innerHTML = "";
      // free corridor
      stage.appendChild(svg("rect", { x: 0, y: wallTop, width: 460, height: gapPx, fill: "#eef5ff" }));
      // walls
      const wallStyle = { fill: "#cbd5e1", stroke: "#94a3b8", "stroke-width": 1 };
      stage.appendChild(svg("rect", Object.assign({ x: 0, y: 0, width: 460, height: Math.max(0, wallTop) }, wallStyle)));
      stage.appendChild(svg("rect", Object.assign({ x: 0, y: wallBot, width: 460, height: Math.max(0, 320 - wallBot) }, wallStyle)));
      // hatch hint on walls
      [["Wall", 18], ["Wall", 304]].forEach(([t, y]) => {
        const txt = svg("text", { x: 12, y: y, "font-size": 11, fill: "#64748b", "font-style": "italic" });
        txt.textContent = t;
        if (y < wallTop + 4 || y > wallBot - 4) stage.appendChild(txt);
      });
      // passage width dimension
      const dim = svg("line", { x1: 40, y1: wallTop, x2: 40, y2: wallBot, stroke: "#475569", "stroke-width": 1, "stroke-dasharray": "3,3" });
      stage.appendChild(dim);
      const dimT = svg("text", { x: 46, y: CY, "font-size": 11, fill: "#475569" });
      dimT.textContent = P.toFixed(2) + " m";
      stage.appendChild(dimT);

      // robot footprint (rotated)
      const hl = (ROBOT_L / 2) * PPM, hw = (ROBOT_W / 2) * PPM;
      const c = Math.cos(theta), s = Math.sin(theta);
      const corners = [[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]]
        .map(([x, y]) => (CX + x * c - y * s) + "," + (CY + x * s + y * c)).join(" ");
      const color = fits ? "#22a06b" : "#ef4444";
      stage.appendChild(svg("polygon", {
        points: corners, fill: color, "fill-opacity": 0.22, stroke: color, "stroke-width": 2.5, "stroke-linejoin": "round"
      }));
      // heading arrow
      const nx = CX + hl * c, ny = CY + hl * s;
      stage.appendChild(svg("line", { x1: CX, y1: CY, x2: nx, y2: ny, stroke: color, "stroke-width": 2.5 }));
      stage.appendChild(svg("circle", { cx: nx, cy: ny, r: 4, fill: color }));

      // labels
      el("yaw-deg").textContent = Math.round(idx * 360 / N_YAW) + "°";
      const badge = el("yaw-fit-badge");
      badge.textContent = fits ? "FITS" : "BLOCKED";
      badge.className = "fit-badge " + (fits ? "fit-yes" : "fit-no");
    }

    function renderDial() {
      const P = passageWidth();
      const idx = Number(yawSlider.value);
      dial.innerHTML = "";
      const cx = 60, cy = 60, r = 46;
      dial.appendChild(svg("circle", { cx, cy, r: r + 7, fill: "#f8fafc", stroke: "#e2e8f0", "stroke-width": 1 }));
      for (let i = 0; i < N_YAW; i++) {
        const th = channelTheta(i) - Math.PI / 2; // 0 at top
        const feas = yawHalfExtent(channelTheta(i)) <= P / 2;
        const cur = i === idx;
        const x1 = cx + Math.cos(th) * (r - 7), y1 = cy + Math.sin(th) * (r - 7);
        const x2 = cx + Math.cos(th) * (r + 4), y2 = cy + Math.sin(th) * (r + 4);
        dial.appendChild(svg("line", {
          x1, y1, x2, y2,
          stroke: cur ? "#1d4ed8" : (feas ? "#22a06b" : "#cbd5e1"),
          "stroke-width": cur ? 4 : 2.5, "stroke-linecap": "round"
        }));
      }
      const count = feasibleCount(P);
      el("yaw-feasible-count").textContent = count;
      const tag = el("yaw-region-tag");
      if (count >= N_YAW) { tag.textContent = "safe region"; tag.className = "yaw-region-tag tag-safe"; }
      else if (count > 0) { tag.textContent = "restricted region"; tag.className = "yaw-region-tag tag-restricted"; }
      else { tag.textContent = "inaccessible"; tag.className = "yaw-region-tag tag-blocked"; }
    }

    function render() { renderStage(); renderDial(); }

    yawSlider.addEventListener("input", render);
    passSlider.addEventListener("input", () => {
      el("passage-val").textContent = passageWidth().toFixed(2) + " m";
      render();
    });
    el("passage-val").textContent = passageWidth().toFixed(2) + " m";
    render();
  }

  /* ── Chart.js SPC benchmark plot ──────────────────────────────── */
  function setupSpcChart() {
    const canvas = el("spc-canvas");
    if (!canvas || typeof Chart === "undefined") return;

    const planners = ["ASA", "RRT-SE2NM", "RRT*-SE2NM", "PRM-SE2NM", "RRT", "RRT*", "PRM"];
    const gym = [0.41, 0.27, 0.15, 0.18, 0.13, 0.13, 0.16];
    const studio = [0.98, 0.63, 0.42, 0.26, 0.29, 0.29, 0.25];
    const highlight = (base, accent) => (c) => planners[c.dataIndex] === "ASA" ? accent : base;
    const legendBaseColors = ["#9ec5d8", "#7fb27e"];
    const isMobile = window.matchMedia("(max-width: 768px)").matches;

    new Chart(canvas, {
      type: "bar",
      data: {
        labels: planners,
        datasets: [
          { label: "Gym SPC", data: gym, backgroundColor: highlight("#9ec5d8", "#3f8fb5"), borderRadius: 3 },
          { label: "Studio SPC", data: studio, backgroundColor: highlight("#7fb27e", "#13866f"), borderRadius: 3 }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        plugins: {
          legend: {
            position: "bottom",
            labels: {
              boxWidth: isMobile ? 10 : 13,
              padding: isMobile ? 10 : 16,
              font: { size: isMobile ? 10 : 12 },
              generateLabels: (chart) => Chart.defaults.plugins.legend.labels.generateLabels(chart).map((label) => ({
                ...label,
                fillStyle: legendBaseColors[label.datasetIndex] || label.fillStyle,
                strokeStyle: legendBaseColors[label.datasetIndex] || label.strokeStyle,
              })),
            }
          },
          tooltip: { callbacks: { label: (c) => c.dataset.label + ": " + c.parsed.y.toFixed(2) } }
        },
        scales: {
          x: { grid: { display: false }, ticks: { maxRotation: isMobile ? 45 : 0, font: { size: isMobile ? 10 : 12 } } },
          y: { beginAtZero: true, max: 1.0, ticks: { stepSize: 0.2, font: { size: isMobile ? 10 : 12 } }, title: { display: true, text: "SPC (↑ better)", font: { size: isMobile ? 10 : 12 } }, grid: { color: "rgba(0,0,0,0.06)" } }
        }
      }
    });
  }

  document.addEventListener("DOMContentLoaded", function () {
    setupNavbar();
    setupVideoTabs();
    setupCompareSlider();
    setupYawDemo();
    setupSpcChart();
  });
})();
