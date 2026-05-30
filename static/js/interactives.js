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
  function setupCompareSlider() {
    const range = el("compare-range");
    const top = el("compare-top");
    const handle = el("compare-handle");
    if (!range || !top || !handle) return;

    function apply(v) {
      top.style.clipPath = "inset(0 " + (100 - v) + "% 0 0)";
      handle.style.left = v + "%";
    }
    range.addEventListener("input", () => apply(Number(range.value)));
    apply(Number(range.value));
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

  /* ── ASA pathfinding stepper ──────────────────────────────────── */
  function setupAsaDemo() {
    const stage = el("asa-svg");
    const info = el("asa-info");
    if (!stage || !info) return;

    // corridor polygons (left room, narrow passage, right room)
    const POLYS = [
      { pts: [[40, 60], [190, 60], [190, 220], [40, 220]], fill: "#dbeafe" },
      { pts: [[190, 118], [300, 118], [300, 182], [190, 182]], fill: "#bfdbfe" },
      { pts: [[300, 60], [445, 60], [445, 220], [300, 220]], fill: "#dbeafe" }
    ];
    const START = [72, 188], GOAL = [415, 92];

    // stage 0: A* zigzag through edge midpoints
    const P0 = [START, [190, 150], [245, 150], [300, 150], GOAL];
    // stage 1 & 2: string-pulled (straightened) path hugging passage corners
    const P12 = [START, [190, 175], [300, 128], GOAL];

    function segAngle(a, b) { return Math.atan2(b[1] - a[1], b[0] - a[0]); }
    function headingsAlong(path) {
      const h = [];
      for (let i = 0; i < path.length; i++) {
        const a = path[Math.max(0, i - 1)], b = path[Math.min(path.length - 1, i + 1)];
        h.push(segAngle(a, b));
      }
      return h;
    }
    function pathLen(path) {
      let L = 0;
      for (let i = 1; i < path.length; i++) L += Math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]);
      return L;
    }

    const H0 = headingsAlong(P0);
    const STAGES = [
      {
        path: P0, headings: H0, color: "#f59e0b",
        len: "12.4 m", cost: "28.0 s",
        title: "① Initial A* search",
        text: "A* over the yaw-specific layers returns a feasible path through polygon-edge midpoints. It zig-zags, but a valid yaw is guaranteed at every state."
      },
      {
        path: P12, headings: H0.slice(0, P12.length), color: "#ef4444",
        len: "11.6 m", cost: "30.8 s",
        title: "② String pulling",
        text: "String pulling straightens the path inside the polygon corridor (−6% length). But the yaws carried over from stage 1 no longer match the new segments, so the cost rises ~10%."
      },
      {
        path: P12, headings: headingsAlong(P12), color: "#22a06b",
        len: "11.6 m", cost: "24.4 s",
        title: "③ Yaw refinement",
        text: "A second A* re-optimizes yaw along the straightened path. Headings line up with motion again, and the final cost drops to ~87% of the initial path."
      }
    ];

    let cur = 0, timer = null;

    function render(stageIdx) {
      const S = STAGES[stageIdx];
      stage.innerHTML = "";
      // corridor
      POLYS.forEach((p) => stage.appendChild(svg("polygon", {
        points: p.pts.map((q) => q.join(",")).join(" "),
        fill: p.fill, stroke: "#93c5fd", "stroke-width": 1.2
      })));
      const lbl = svg("text", { x: 245, y: 110, "font-size": 10, fill: "#1e40af", "text-anchor": "middle", "font-style": "italic" });
      lbl.textContent = "narrow passage";
      stage.appendChild(lbl);

      // faint other stages' paths for reference
      STAGES.forEach((other, i) => {
        if (i === stageIdx) return;
        const d = other.path.map((p, k) => (k ? "L" : "M") + p[0] + " " + p[1]).join(" ");
        stage.appendChild(svg("path", { d, fill: "none", stroke: "#cbd5e1", "stroke-width": 1.5, "stroke-dasharray": "4,4" }));
      });

      // active path
      const d = S.path.map((p, k) => (k ? "L" : "M") + p[0] + " " + p[1]).join(" ");
      stage.appendChild(svg("path", { d, fill: "none", stroke: S.color, "stroke-width": 3.5, "stroke-linejoin": "round", "stroke-linecap": "round" }));

      // heading arrows
      S.path.forEach((p, i) => {
        const th = S.headings[i];
        const L = 16;
        const x2 = p[0] + Math.cos(th) * L, y2 = p[1] + Math.sin(th) * L;
        stage.appendChild(svg("line", { x1: p[0], y1: p[1], x2, y2, stroke: S.color, "stroke-width": 2 }));
        stage.appendChild(svg("circle", { cx: x2, cy: y2, r: 2.6, fill: S.color }));
        stage.appendChild(svg("circle", { cx: p[0], cy: p[1], r: 3.2, fill: "#fff", stroke: S.color, "stroke-width": 1.6 }));
      });

      // start / goal
      function marker(p, color, label) {
        stage.appendChild(svg("circle", { cx: p[0], cy: p[1], r: 6, fill: color, stroke: "#fff", "stroke-width": 2 }));
        const t = svg("text", { x: p[0], y: p[1] - 10, "font-size": 11, fill: color, "text-anchor": "middle", "font-weight": "700" });
        t.textContent = label;
        stage.appendChild(t);
      }
      marker(START, "#22a06b", "start");
      marker(GOAL, "#ef5b45", "goal");

      info.innerHTML = "<p class='asa-info-title'>" + S.title + "</p><p>" + S.text + "</p>";
      el("asa-len").textContent = S.len;
      el("asa-cost").textContent = S.cost;
      document.querySelectorAll(".asa-step-btn").forEach((b) =>
        b.classList.toggle("is-active", Number(b.dataset.asaStage) === stageIdx));
    }

    function stop() {
      if (timer) { clearInterval(timer); timer = null; }
      const btn = el("asa-play");
      if (btn) btn.textContent = "Auto-play";
    }

    document.querySelectorAll(".asa-step-btn").forEach((b) =>
      b.addEventListener("click", () => { stop(); cur = Number(b.dataset.asaStage); render(cur); }));

    const play = el("asa-play");
    if (play) play.addEventListener("click", () => {
      if (timer) { stop(); return; }
      play.textContent = "Pause";
      timer = setInterval(() => { cur = (cur + 1) % STAGES.length; render(cur); }, 1900);
    });

    render(0);
  }

  /* ── Chart.js SPC benchmark plot ──────────────────────────────── */
  function setupSpcChart() {
    const canvas = el("spc-canvas");
    if (!canvas || typeof Chart === "undefined") return;

    const planners = ["ASA", "PRM-SE2NM", "PRM", "RRT*-SE2NM", "RRT-SE2NM", "RRT", "RRT*"];
    const gym = [0.41, 0.27, 0.15, 0.18, 0.13, 0.13, 0.16];
    const studio = [0.98, 0.63, 0.42, 0.26, 0.29, 0.29, 0.25];
    const highlight = (base) => planners.map((p) => p === "ASA" ? "#13866f" : base);
    const isMobile = window.matchMedia("(max-width: 768px)").matches;

    new Chart(canvas, {
      type: "bar",
      data: {
        labels: planners,
        datasets: [
          { label: "Gym SPC", data: gym, backgroundColor: highlight("#9ec5d8"), borderRadius: 3 },
          { label: "Studio SPC", data: studio, backgroundColor: highlight("#7fb27e"), borderRadius: 3 }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        plugins: {
          legend: { position: "bottom", labels: { boxWidth: isMobile ? 10 : 13, padding: isMobile ? 10 : 16, font: { size: isMobile ? 10 : 12 } } },
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
    setupAsaDemo();
    setupSpcChart();
  });
})();
