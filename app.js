/* ==========================================================================
   reBot 采集端 · 站点交互
   ① 悬停（聚光 + 倾斜 + 磁吸按钮）   ② 滚动 morph   ③ 拖拽物理
   ④ 滚动 3D 穿越                     ⑤ 指针视差
   全部原生实现，无外部依赖；统一由一个 rAF 循环驱动。
   ========================================================================== */
(() => {
  "use strict";

  const reduce = matchMedia("(prefers-reduced-motion: reduce)").matches;
  const $ = (s, r = document) => r.querySelector(s);
  const $$ = (s, r = document) => Array.from(r.querySelectorAll(s));
  const clamp = (v, a = 0, b = 1) => Math.min(b, Math.max(a, v));
  const lerp = (a, b, t) => a + (b - a) * t;
  const smooth = (t) => t * t * (3 - 2 * t);

  /* ── 统一帧循环 ───────────────────────────────────────────── */
  const tasks = [];
  let last = performance.now();
  function tick(now) {
    const dt = Math.min(0.05, (now - last) / 1000);
    last = now;
    for (const t of tasks) t(now, dt);
    requestAnimationFrame(tick);
  }
  requestAnimationFrame(tick);

  /* ── 滚动进度（缓存 offsetTop，避免每帧读布局）─────────────── */
  const measures = [];
  function measure() {
    const y = window.scrollY;
    for (const m of measures) {
      const r = m.el.getBoundingClientRect();
      m.top = r.top + y;
      m.height = r.height;
    }
  }
  /** 元素滚动进度：0 = 刚进入视野顶端，1 = 完全滚过 */
  function progress(m) {
    const span = Math.max(1, m.height - window.innerHeight);
    return clamp((window.scrollY - m.top) / span);
  }
  const registerScroll = (sel) => {
    const el = $(sel);
    if (!el) return null;
    const m = { el, top: 0, height: 1 };
    measures.push(m);
    return m;
  };

  /* ── 顶部：进度条 + 导航背景 + 逐段出场 ────────────────────── */
  const bar = $("#progressBar"), nav = $("#nav");
  tasks.push(() => {
    const h = document.documentElement;
    const max = h.scrollHeight - window.innerHeight;
    if (bar) bar.style.width = (max > 0 ? (window.scrollY / max) * 100 : 0).toFixed(2) + "%";
    if (nav) nav.classList.toggle("solid", window.scrollY > 40);
  });

  const io = new IntersectionObserver((entries) => {
    entries.forEach((e, i) => {
      if (e.isIntersecting) {
        e.target.style.transitionDelay = (i % 4) * 70 + "ms";
        e.target.classList.add("in");
        io.unobserve(e.target);
      }
    });
  }, { rootMargin: "0px 0px -12% 0px", threshold: 0.12 });
  $$(".reveal").forEach((el) => io.observe(el));

  /* ── 数字滚动（进入视野后从 0 数到目标值）──────────────────── */
  const counters = $$("[data-count]");
  const counterIO = new IntersectionObserver((entries) => {
    entries.forEach((e) => {
      if (!e.isIntersecting) return;
      const el = e.target, to = parseFloat(el.dataset.count), dec = parseInt(el.dataset.decimals || "0", 10);
      const t0 = performance.now(), dur = 1100;
      const step = (now) => {
        const t = clamp((now - t0) / dur);
        el.textContent = lerp(0, to, smooth(t)).toFixed(dec);
        if (t < 1) requestAnimationFrame(step);
        else el.textContent = to.toFixed(dec);
      };
      requestAnimationFrame(step);
      counterIO.unobserve(el);
    });
  }, { threshold: 0.6 });
  counters.forEach((c) => counterIO.observe(c));

  /* ══════════════════════════════════════════════════════════════
     ① 悬停：聚光 / 倾斜 / 磁吸按钮
     ══════════════════════════════════════════════════════════════ */
  if (!reduce) {
    $$("[data-spot]").forEach((card) => {
      card.addEventListener("pointermove", (e) => {
        const r = card.getBoundingClientRect();
        card.style.setProperty("--mx", e.clientX - r.left + "px");
        card.style.setProperty("--my", e.clientY - r.top + "px");
      });
    });
    $$(".tilt").forEach((card) => {
      card.addEventListener("pointermove", (e) => {
        const r = card.getBoundingClientRect();
        const nx = (e.clientX - r.left) / r.width - 0.5;
        const ny = (e.clientY - r.top) / r.height - 0.5;
        card.style.transform =
          `perspective(900px) rotateY(${(nx * 7).toFixed(2)}deg) rotateX(${(-ny * 7).toFixed(2)}deg) translateY(-3px)`;
      });
      card.addEventListener("pointerleave", () => { card.style.transform = ""; });
    });
    // 磁吸：按钮在自身范围内轻微跟随指针
    $$(".btn").forEach((btn) => {
      btn.addEventListener("pointermove", (e) => {
        const r = btn.getBoundingClientRect();
        const dx = (e.clientX - (r.left + r.width / 2)) / r.width;
        const dy = (e.clientY - (r.top + r.height / 2)) / r.height;
        btn.style.transform = `translate(${(dx * 6).toFixed(1)}px, ${(dy * 4 - 2).toFixed(1)}px)`;
      });
      btn.addEventListener("pointerleave", () => { btn.style.transform = ""; });
    });
  }

  /* ══════════════════════════════════════════════════════════════
     ⑤ 指针视差（hero 分层 + 背景粒子）
     ══════════════════════════════════════════════════════════════ */
  const hero = $(".hero");
  const pv = { x: 0, y: 0, tx: 0, ty: 0 };
  if (hero && !reduce) {
    hero.addEventListener("pointermove", (e) => {
      const r = hero.getBoundingClientRect();
      pv.tx = ((e.clientX - r.left) / r.width) * 2 - 1;
      pv.ty = ((e.clientY - r.top) / r.height) * 2 - 1;
    });
    hero.addEventListener("pointerleave", () => { pv.tx = 0; pv.ty = 0; });
  }
  const depthLayers = $$("[data-depth]");
  tasks.push(() => {
    if (reduce) return;
    pv.x = lerp(pv.x, pv.tx, 0.06);
    pv.y = lerp(pv.y, pv.ty, 0.06);
    for (const el of depthLayers) {
      const d = parseFloat(el.dataset.depth) || 0.3;
      el.style.transform =
        `translate3d(${(pv.x * 24 * d).toFixed(2)}px, ${(pv.y * 20 * d).toFixed(2)}px, 0)`;
    }
  });

  // 背景粒子：随滚动产生纵深位移，随指针偏移
  const cv = $("#heroCanvas");
  if (cv) {
    const ctx = cv.getContext("2d");
    let dots = [], W = 0, H = 0, DPR = 1;
    const build = () => {
      DPR = Math.min(2, window.devicePixelRatio || 1);
      W = cv.clientWidth; H = cv.clientHeight;
      cv.width = Math.floor(W * DPR); cv.height = Math.floor(H * DPR);
      ctx.setTransform(DPR, 0, 0, DPR, 0, 0);
      dots = Array.from({ length: W < 700 ? 46 : 92 }, () => ({
        x: Math.random() * W, y: Math.random() * H,
        z: 0.2 + Math.random() * 0.8, r: 0.5 + Math.random() * 1.6,
        a: 0.06 + Math.random() * 0.3,
      }));
    };
    build();
    addEventListener("resize", build);
    tasks.push((now) => {
      const t = now / 1000;
      ctx.clearRect(0, 0, W, H);
      const sy = window.scrollY;
      for (const d of dots) {
        const y = (((d.y - sy * 0.08 * d.z) % H) + H) % H;
        const x = d.x + (reduce ? 0 : pv.x * 18 * d.z);
        ctx.globalAlpha = d.a * (0.5 + 0.5 * d.z);
        ctx.fillStyle = d.z > 0.75 ? "#3ac0ff" : "#5b6577";
        ctx.beginPath();
        ctx.arc(x + (reduce ? 0 : Math.sin(t * 0.3 + d.x) * 2), y, d.r * d.z, 0, Math.PI * 2);
        ctx.fill();
      }
      ctx.globalAlpha = 1;
    });
  }

  /* ── hero 里的 HUD：关节条 + 笔速（让它像在跑）──────────────── */
  const jointBox = $("#jointRows");
  if (jointBox) {
    const names = ["J1 肩部水平", "J2 肩部俯仰", "J3 肘部俯仰", "J4 腕部俯仰",
                   "J5 腕部偏航", "J6 腕部自转", "夹爪"];
    const rows = names.map((n) => {
      const el = document.createElement("div");
      el.className = "joint";
      el.innerHTML = `<span>${n}</span><span class="track"><i class="fill"></i></span><span class="val">—</span>`;
      jointBox.appendChild(el);
      return { fill: $(".fill", el), val: $(".val", el) };
    });
    const penHz = $("#penHz");
    tasks.push((now) => {
      const t = now / 1000;
      rows.forEach((r, i) => {
        const v = i === 6
          ? 0.5 + 0.45 * Math.sin(t * 0.5 + 2)
          : 0.35 + 0.6 * Math.abs(Math.sin(t * 0.32 + i * 0.9));
        r.fill.style.width = (v * 100).toFixed(1) + "%";
        r.val.textContent = (v * 6.28 - 3.14).toFixed(3);
      });
      if (penHz) penHz.textContent = (58 + 42 * Math.abs(Math.sin(t * 0.7))).toFixed(0);
    });
  }

  /* ══════════════════════════════════════════════════════════════
     ② 滚动驱动 · 采集流程左侧的 3D 场景
     现成模型（CC-BY，Poly Pizza）：机械臂 + 数位板；懒加载 three.js
     ══════════════════════════════════════════════════════════════ */
  const morphM = registerScroll("#morphWrap");
  const stage = $("#stage3d");
  const badge3d = $("#morphBadge");
  const poster3d = $("#stage3dPoster");
  const BADGES = ["① 输入 · 笔", "② 采集 · 关节轨迹", "③ 产出 · 数据集"];
  const morphSteps2 = $$(".morph-step");
  let scene3d = null, sceneLoading = false, sceneStep = -1;

  function applyStep(i) {
    if (i === sceneStep) return;
    sceneStep = i;
    if (badge3d) badge3d.textContent = BADGES[i] || "";
    morphSteps2.forEach((s, k) => s.classList.toggle("active", k === i));
  }
  applyStep(0);

  if (stage && !reduce) {
    const io3d = new IntersectionObserver(async (entries) => {
      if (!entries.some((e) => e.isIntersecting) || sceneLoading || scene3d) return;
      sceneLoading = true;
      try {
        const mod = await import("./scene3d.js");      // 进入视口才下载 three.js（1.3MB）
        scene3d = await mod.initScene3D({
          canvas: $("#scene3d"), badge: badge3d, poster: poster3d, onStep: applyStep,
        });
        if (stage) stage.dataset.ready = "1";
      } catch (err) {
        console.warn("[3D] 场景初始化失败，保留静态图：", err);
        sceneLoading = false;
      }
    }, { rootMargin: "220px" });
    io3d.observe(stage);
  } else if (stage) {
    applyStep(2);                    // 减弱动态效果：直接展示终态
  }

  tasks.push(() => {
    if (!morphM) return;
    const p = reduce ? 1 : progress(morphM);
    scene3d?.setProgress(p);
  });

  /* 旧的 SVG morph 参数保留：shapePath 仍被下面这段用于……（已由 3D 取代）
     —— 保留函数定义以免其它处引用报错，实际不再绘制。 */

  /* ══════════════════════════════════════════════════════════════
     ④ 滚动 3D 穿越四层
     ══════════════════════════════════════════════════════════════ */
  const flyM = registerScroll("#fly");
  const flyLayers = $$(".fly-layer");
  const GAP = 640;
  let flyV = 0;
  /** 单层的可见度：不是对称淡入淡出（那会让两层文字叠在一起），
      而是「渐显 → 可读窗口 → 快速淡出」；越过的层直接隐藏。 */
  function layerOpacity(d) {           // d = flyV - i（>0 表示已经越过它）
    if (d < -0.85) return 0.09;                                    // 远处：只留一点影子
    if (d < -0.35) return 0.09 + 0.91 * smooth((d + 0.85) / 0.5);  // 渐显
    if (d > 0.32) return Math.max(0, 1 - smooth((d - 0.32) / 0.53)); // 淡出
    return 1;                                                      // 可读窗口
  }
  tasks.push(() => {
    if (!flyM || !flyLayers.length) return;
    const p = reduce ? flyLayers.length - 1 : progress(flyM) * (flyLayers.length - 1);
    flyV = lerp(flyV, p, 0.16);
    flyLayers.forEach((el, i) => {
      const d = flyV - i;
      const vis = layerOpacity(d);
      el.style.transform =
        `translate3d(${(d * 26).toFixed(1)}px, 0, ${(d * GAP).toFixed(1)}px) ` +
        `rotateX(${(-d * 5).toFixed(2)}deg)`;
      el.style.opacity = vis.toFixed(3);
      el.style.filter = vis > 0.97 ? "none" : `blur(${((1 - vis) * 4.5).toFixed(2)}px)`;
      el.style.visibility = (d > 0.9 || d < -1.9) ? "hidden" : "visible";
      el.style.pointerEvents = vis > 0.8 ? "auto" : "none";
      el.style.zIndex = String(100 - Math.abs(Math.round(d * 10)));
    });
  });

  /* ── 截图灯箱：点开看大图 ───────────────────────────────────── */
  const shots = $$(".shot img");
  if (shots.length) {
    const lb = document.createElement("div");
    lb.className = "lightbox";
    lb.innerHTML = '<img alt="" /><p>点击任意处 / Esc 关闭</p>';
    document.body.appendChild(lb);
    const img = $("img", lb);
    const close = () => lb.classList.remove("open");
    shots.forEach((s) => s.addEventListener("click", () => {
      img.src = s.currentSrc || s.src;
      lb.classList.add("open");
    }));
    lb.addEventListener("click", close);
    addEventListener("keydown", (e) => { if (e.key === "Escape") close(); });
  }

  /* ══════════════════════════════════════════════════════════════
     ③ 拖拽物理：把留档拖进 datasets/
     数据取自仓库真实 episode（7 集：A×1 / B×4 / F×2）
     ══════════════════════════════════════════════════════════════ */
  const pc = $("#playCanvas");
  if (pc) {
    const ctx = pc.getContext("2d");
    const EPS = [
      { n: "2 #1", g: "A", s: 85.6 },
      { n: "2 #2", g: "B", s: 80.5 },
      { n: "0905_b #3", g: "B", s: 79.6 },
      { n: "0905_all #3", g: "B", s: 79.6 },
      { n: "3 #2", g: "B", s: 78.7 },
      { n: "0905_all #1", g: "F", s: 80.9 },
      { n: "0905_all #2", g: "F", s: 83.0 },
    ];
    const GRADE = { A: "#39d98a", B: "#3ac0ff", F: "#ff6b6b", C: "#f5c451" };
    let items = [], box = { x: 0, y: 0, w: 0, h: 0 };
    let W = 0, H = 0, DPR = 1;
    let drag = null, flash = 0, refuse = 0;
    const S = { raw: 0, packed: 0 };

    const layout = () => {
      DPR = Math.min(2, window.devicePixelRatio || 1);
      W = pc.clientWidth; H = pc.clientHeight;
      pc.width = Math.floor(W * DPR); pc.height = Math.floor(H * DPR);
      ctx.setTransform(DPR, 0, 0, DPR, 0, 0);
      const bw = Math.max(150, Math.min(210, W * 0.26));
      box = { x: W - bw - 18, y: H * 0.5 - 78, w: bw, h: 156 };
      const cols = W > 720 ? 3 : 2;
      const cw = Math.min(150, (box.x - 30) / cols - 12);
      const ch = 46;
      items.forEach((it, i) => {
        it.w = cw; it.h = ch;
        it.hx = 20 + (i % cols) * (cw + 12);
        it.hy = 30 + Math.floor(i / cols) * (ch + 12) + 20;
        if (!it.grabbed && !it.done) { it.x = it.hx; it.y = it.hy; }
      });
    };

    const reset = () => {
      items = EPS.map((e, i) => ({
        ...e, i, x: 0, y: 0, vx: 0, vy: 0, hx: 0, hy: 0, w: 120, h: 46,
        rot: (Math.random() - 0.5) * 0.06, grabbed: false, done: false, scale: 1,
      }));
      S.raw = items.length; S.packed = 0;
      layout();
      for (const it of items) { it.x = it.hx; it.y = it.hy; }
      syncHud();
    };
    const syncHud = () => {
      const raw = $("#playRaw"), packed = $("#playPacked"), labels = $("#playLabels");
      if (raw) raw.textContent = S.raw;
      if (packed) packed.textContent = S.packed;
      if (labels) labels.textContent = S.packed ? `${S.packed} 集 / A·B` : "—";
    };

    const pick = (mx, my) => {
      for (let i = items.length - 1; i >= 0; i--) {
        const it = items[i];
        if (it.done) continue;
        if (mx >= it.x && mx <= it.x + it.w * it.scale && my >= it.y && my <= it.y + it.h * it.scale) return it;
      }
      return null;
    };
    const pos = (e) => {
      const r = pc.getBoundingClientRect();
      return { x: (e.clientX - r.left) * (W / r.width), y: (e.clientY - r.top) * (H / r.height) };
    };

    let lastMove = { t: 0, x: 0, y: 0 };
    pc.addEventListener("pointerdown", (e) => {
      const p = pos(e);
      const it = pick(p.x, p.y);
      if (!it) return;
      pc.setPointerCapture(e.pointerId);
      pc.classList.add("dragging");
      it.grabbed = true;
      drag = { it, ox: p.x - it.x, oy: p.y - it.y };
      lastMove = { t: performance.now(), x: p.x, y: p.y };
      it.vx = it.vy = 0;
    });
    pc.addEventListener("pointermove", (e) => {
      if (!drag) return;
      const p = pos(e);
      const it = drag.it;
      it.x = p.x - drag.ox; it.y = p.y - drag.oy;
      const now = performance.now();
      const dt = Math.max(8, now - lastMove.t) / 1000;
      it.vx = (p.x - lastMove.x) / dt;
      it.vy = (p.y - lastMove.y) / dt;
      lastMove = { t: now, x: p.x, y: p.y };
    });
    const endDrag = (e) => {
      if (!drag) return;
      const it = drag.it;
      it.grabbed = false;
      drag = null;
      pc.classList.remove("dragging");
      const cx = it.x + it.w / 2, cy = it.y + it.h / 2;
      const inside = cx > box.x && cx < box.x + box.w && cy > box.y && cy < box.y + box.h;
      if (inside) {
        if (it.g === "F") {                      // F 级：被硬门弹回来
          it.vx = -Math.max(420, Math.abs(it.vx)); it.vy = -260;
          refuse = 1;
        } else {                                  // A/B：吸进数据集
          it.done = true; it.target = { x: box.x + box.w / 2 - it.w / 2, y: box.y + box.h / 2 - it.h / 2 };
          S.packed += 1; S.raw -= 1; flash = 1;
          syncHud();
        }
      }
      if (e && pc.hasPointerCapture?.(e.pointerId)) pc.releasePointerCapture(e.pointerId);
    };
    pc.addEventListener("pointerup", endDrag);
    pc.addEventListener("pointercancel", endDrag);

    const roundRect = (x, y, w, h, r) => {
      ctx.beginPath();
      ctx.moveTo(x + r, y);
      ctx.arcTo(x + w, y, x + w, y + h, r);
      ctx.arcTo(x + w, y + h, x, y + h, r);
      ctx.arcTo(x, y + h, x, y, r);
      ctx.arcTo(x, y, x + w, y, r);
      ctx.closePath();
    };

    tasks.push((now, dt) => {
      // 物理积分
      for (const it of items) {
        if (it.done) {
          if (it.target) {
            it.x = lerp(it.x, it.target.x, 0.18);
            it.y = lerp(it.y, it.target.y, 0.18);
            it.scale = lerp(it.scale, 0.34, 0.14);
            if (Math.abs(it.x - it.target.x) < 1 && Math.abs(it.y - it.target.y) < 1) it.target = null;
          }
          continue;
        }
        if (it.grabbed) continue;
        // 阻尼 + 回位弹簧（松手后自己回到槽位）
        it.vx *= Math.pow(0.14, dt);
        it.vy *= Math.pow(0.14, dt);
        it.vx += (it.hx - it.x) * 5.5 * dt;
        it.vy += (it.hy - it.y) * 5.5 * dt;
        it.x += it.vx * dt; it.y += it.vy * dt;
        // 边界反弹
        if (it.x < 8) { it.x = 8; it.vx = Math.abs(it.vx) * 0.55; }
        if (it.x + it.w > box.x - 8) { it.x = box.x - 8 - it.w; it.vx = -Math.abs(it.vx) * 0.55; }
        if (it.y < 8) { it.y = 8; it.vy = Math.abs(it.vy) * 0.55; }
        if (it.y + it.h > H - 8) { it.y = H - 8 - it.h; it.vy = -Math.abs(it.vy) * 0.55; }
      }
      flash = Math.max(0, flash - dt * 2);
      refuse = Math.max(0, refuse - dt * 1.6);

      // 绘制
      ctx.clearRect(0, 0, W, H);

      // 数据集框
      const packedRatio = S.packed / items.length;
      ctx.setLineDash([7, 6]);
      ctx.lineWidth = 1.4;
      ctx.strokeStyle = refuse > 0 ? `rgba(255,107,107,${0.5 + refuse * 0.5})`
                                   : `rgba(58,192,255,${0.3 + flash * 0.6})`;
      roundRect(box.x, box.y, box.w, box.h, 12);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = "rgba(58,192,255,0.05)";
      roundRect(box.x, box.y, box.w, box.h, 12);
      ctx.fill();
      // 已入库的填充条
      ctx.fillStyle = "rgba(57,217,138,0.16)";
      roundRect(box.x + 6, box.y + box.h - 8 - (box.h - 16) * packedRatio,
                box.w - 12, (box.h - 16) * packedRatio, 7);
      ctx.fill();
      ctx.font = "600 13px ui-monospace, Menlo, monospace";
      ctx.fillStyle = "#8b95a8";
      ctx.textAlign = "center";
      ctx.fillText("datasets/", box.x + box.w / 2, box.y - 10);
      ctx.font = "500 11px ui-monospace, Menlo, monospace";
      ctx.fillStyle = "#5b6577";
      ctx.fillText(`${S.packed} / ${items.length} 集`, box.x + box.w / 2, box.y + box.h + 20);

      // 方块
      for (const it of items) {
        const w = it.w * it.scale, h = it.h * it.scale;
        ctx.save();
        ctx.translate(it.x + w / 2, it.y + h / 2);
        ctx.rotate(it.rot);
        ctx.shadowColor = "rgba(0,0,0,.45)";
        ctx.shadowBlur = it.grabbed ? 26 : 12;
        ctx.shadowOffsetY = it.grabbed ? 10 : 4;
        ctx.fillStyle = "#171d2a";
        roundRect(-w / 2, -h / 2, w, h, 9);
        ctx.fill();
        ctx.shadowColor = "transparent";
        ctx.lineWidth = 1.2;
        ctx.strokeStyle = it.done ? "rgba(57,217,138,.5)" : "#232b3a";
        roundRect(-w / 2, -h / 2, w, h, 9);
        ctx.stroke();
        if (it.scale > 0.5) {
          ctx.font = "700 11px ui-monospace, Menlo, monospace";
          ctx.fillStyle = GRADE[it.g] || "#8b95a8";
          ctx.textAlign = "left";
          ctx.fillText(it.g, -w / 2 + 11, 4);
          ctx.font = "500 10px ui-monospace, Menlo, monospace";
          ctx.fillStyle = "#93a0b5";
          ctx.fillText(it.n, -w / 2 + 26, 4);
          ctx.fillStyle = "#5b6577";
          ctx.textAlign = "right";
          ctx.font = "600 10px ui-monospace, Menlo, monospace";
          ctx.fillText(it.s.toFixed(1), w / 2 - 10, 4);
        }
        ctx.restore();
      }

      // 顶部提示
      ctx.textAlign = "center";
      ctx.font = "500 12px -apple-system, PingFang SC, sans-serif";
      ctx.fillStyle = refuse > 0 ? `rgba(255,107,107,${refuse})` : "rgba(139,149,168,.75)";
      ctx.fillText(refuse > 0 ? "F 级：硬门未通过，拒绝入库" : "拖动方块 → 松手有惯性；A/B 级会被 datasets/ 吸进去", W / 2, H - 14);
    });

    const ro = new ResizeObserver(() => { layout(); });
    ro.observe(pc);
    reset();
    const rb = $("#playReset");
    if (rb) rb.addEventListener("click", reset);

    // 只读调试钩子（自动化测试用；不影响交互）
    window.__rebotPlay = {
      snapshot: () => ({
        box: { ...box }, packed: S.packed, raw: S.raw,
        items: items.map((it, i) => ({ i, x: it.x, y: it.y, w: it.w, h: it.h, g: it.g, done: it.done })),
      }),
    };
  }

  /* ── 首次测量 + 窗口变化时重测 ─────────────────────────────── */
  measure();
  addEventListener("resize", measure);
  addEventListener("load", measure);
})();
