/* RDP 采集端 UI —— 无框架，直连本地 REST + WebSocket */
"use strict";

const state = {
  profile: null,
  status: null,
  live: null,
  wsState: null,
  wsTeleop: null,
  recording: false,
  sessionActive: false,
  taskTarget: 30,
  zMode: false,
  touching: false,
  lastSample: null,
  camSrcIndex: null,
};

const $ = (id) => document.getElementById(id);

/* ------------------------------------------------------------------ */
/* 基础请求                                                            */
/* ------------------------------------------------------------------ */
async function api(path, method = "GET", body = null) {
  const res = await fetch(path, {
    method,
    headers: body ? { "Content-Type": "application/json" } : undefined,
    body: body ? JSON.stringify(body) : undefined,
  });
  const text = await res.text();
  let data = null;
  try { data = text ? JSON.parse(text) : null; } catch (_) { data = { raw: text }; }
  if (!res.ok) throw new Error((data && data.detail) || res.statusText);
  return data;
}

function setDot(cls) {
  const d = $("health-dot");
  d.className = "dot " + cls;
}

/* ------------------------------------------------------------------ */
/* 渲染                                                                */
/* ------------------------------------------------------------------ */
function renderChips() {
  const st = state.status;
  if (!st) return;
  const dev = st.device || {};
  const chips = [
    ["后端", st.backend],
    ["型号", st.profile?.arm_model || "-"],
    ["电机", `${dev.motor_count ?? 0}/7`],
    ["电压", `${dev.voltage ?? "-"} V`],
    ["温度", dev.temps_c?.length ? `${Math.max(...dev.temps_c).toFixed(0)} °C` : "-"],
    ["标定", dev.calibrated ? "✓" : "✗"],
  ];
  $("status-chips").innerHTML = chips
    .map(([k, v]) => `<span class="chip">${k} <b>${v}</b></span>`)
    .join("");
}

function renderJoints() {
  const live = state.live;
  if (!live || !live.joints) return;
  const box = $("joints");
  if (!box.dataset.ready) {
    box.dataset.ready = "1";
    box.innerHTML = "";
    const extras = [{ name: "gripper", pos: 0, target: 0, margin: 1, at_limit: false }];
    for (const j of [...live.joints, ...extras]) {
      const el = document.createElement("div");
      el.className = "joint";
      el.innerHTML = `<span class="name">${j.name}</span>
        <span class="track"><span class="fill"></span></span>
        <span class="val">—</span>`;
      box.appendChild(el);
    }
  }
  const rows = box.querySelectorAll(".joint");
  live.joints.forEach((j, i) => {
    const el = rows[i];
    if (!el) return;
    const lo = -3.14, hi = 3.14; // 仅用于显示比例（真实限位见设备档案）
    const pct = Math.max(0, Math.min(100, ((j.pos - lo) / (hi - lo)) * 100));
    const fill = el.querySelector(".fill");
    fill.style.left = "0%";
    fill.style.width = pct + "%";
    fill.classList.toggle("at-limit", !!j.at_limit);
    el.querySelector(".val").textContent = j.pos.toFixed(2);
  });
  const gripRow = rows[rows.length - 1];
  if (gripRow) {
    const g = Math.max(0, Math.min(100, (live.gripper ?? 0) * 100));
    gripRow.querySelector(".fill").style.width = g + "%";
    gripRow.querySelector(".val").textContent = (live.gripper ?? 0).toFixed(2);
  }

  $("fps-line").textContent = `${live.fps_actual ?? 0} fps`;
  $("pen-hz").textContent = `${live.pen_hz ?? 0} Hz`;

  const lights = live.lights || {};
  setLight("light-drop", !lights.drop, lights.drop ? "bad" : "on");
  setLight("light-limit", !lights.limit, lights.limit ? "bad" : "on");
  setLight("light-smooth", lights.smooth, "on");
  setLight("light-occlusion", lights.occlusion, "on");

  const rec = $("rec-state");
  if (live.recording) {
    rec.textContent = `录制中 #${live.recording.episode_index} · ${live.recording.frames} 帧`;
    rec.className = "badge rec";
  } else if (live.session_active) {
    rec.textContent = "待录";
    rec.className = "badge ready";
  } else {
    rec.textContent = "空闲";
    rec.className = "badge";
  }

  state.recording = !!live.recording;
  state.sessionActive = !!live.session_active;
  updateButtons();
}

function setLight(id, on, cls) {
  const el = $(id);
  el.className = "light " + (on ? cls : "");
}

function updateButtons() {
  const rec = state.recording;
  const ses = state.sessionActive;
  $("btn-session").textContent = ses ? "结束会话" : "开始会话";
  $("btn-start").disabled = rec;
  $("btn-stop").disabled = !rec;
  $("btn-stop-fail").disabled = !rec;
  $("btn-discard").disabled = !rec;
  $("session-info").textContent = ses
    ? "会话进行中"
    : "未开始会话";
  const n = state.live?.episodes ?? 0;
  $("task-progress").style.width = Math.min(100, (n / state.taskTarget) * 100) + "%";
}

function renderEpisodes(episodes) {
  const box = $("episodes");
  if (!episodes?.length) {
    box.innerHTML = '<div class="muted small">暂无数据</div>';
    return;
  }
  box.innerHTML = episodes
    .slice()
    .reverse()
    .map(
      (e) => `<div class="episode">
        <span class="idx">#${e.index}</span>
        <span>${e.frames} 帧 · ${e.duration_s}s ${e.success ? "" : "· 失败"}</span>
        <span class="grade ${e.grade}">${e.grade}</span>
        <span class="score">${e.score?.toFixed?.(1) ?? e.score}</span>
      </div>`
    )
    .join("");
}

/* ------------------------------------------------------------------ */
/* 遥操控制（teleop_core，与 spatial_teleop 同键位）                     */
/* ------------------------------------------------------------------ */
const JRATE = 1.047;          // 60°/s
const TW = 0.611;             // 35°/s
const SPEED_NAMES = ["慢", "中", "快"];

async function postJSON(path, body) {
  return api(path, "POST", body || {});
}

function coreButtons() {
  const jbox = $("core-joints");
  jbox.innerHTML = "";
  for (let i = 0; i < 7; i++) {
    const b = document.createElement("button");
    b.className = "btn small";
    b.textContent = i < 6 ? `J${i + 1}` : "夹爪";
    b.dataset.joint = i;
    b.onmousedown = async (ev) => {
      ev.preventDefault();
      await postJSON("/api/teleop/joint", { index: i, hold: JRATE });
    };
    b.onmouseup = () => postJSON("/api/teleop/joint", { hold: 0 });
    b.onmouseleave = () => postJSON("/api/teleop/joint", { hold: 0 });
    jbox.appendChild(b);
  }
  const pbox = $("core-presets");
  pbox.innerHTML = "";
  for (let i = 1; i <= 4; i++) {
    const b = document.createElement("button");
    b.className = "btn small";
    b.textContent = `预设${i}`;
    b.title = "点击=前往；Shift+点击=记录";
    b.onclick = (ev) => postJSON("/api/teleop/preset", { action: ev.shiftKey ? "record" : "goto", index: i });
    pbox.appendChild(b);
  }
}

function renderCore(t) {
  if (!t) return;
  $("core-status").textContent =
    `${t.mode === "ori" ? "姿态" : "位置"}${t.freeze ? " · 冻结" : ""}${t.float ? " · 悬停" : ""} · ${t.rate_hz}Hz`;
  $("c-freeze").textContent = t.freeze ? "解冻 (Space)" : "冻结 (Space)";
  $("c-freeze").classList.toggle("danger", !!t.freeze);
  $("c-speed").textContent = `速度：${SPEED_NAMES[t.speed_index] ?? "?"} (F)`;
  $("c-mode").textContent = `模式：${t.mode === "ori" ? "姿态" : "位置"} (O)`;
  $("c-float").classList.toggle("primary", !!t.float);
  $("core-msg").textContent = t.alarm || t.msg || "";
  document.querySelectorAll("#core-joints button").forEach((b) => {
    b.classList.toggle("primary", Number(b.dataset.joint) === t.joint_index);
  });
}

/* ------------------------------------------------------------------ */
/* 相机                                                                */
/* ------------------------------------------------------------------ */
function cameraButtons(cam) {
  const box = $("cam-list");
  box.innerHTML = "";
  (cam.cameras || []).forEach((c) => {
    const b = document.createElement("button");
    b.className = "btn small";
    b.textContent = `cam${c.index}${c.opened ? " ✓" : ""}`;
    b.onclick = async () => {
      const r = await postJSON("/api/camera/open", { index: c.index });
      if (r && r.ok === false) alert("打开失败：" + (r.error || "未知"));
    };
    box.appendChild(b);
  });
}

function renderCamera(cam) {
  if (!cam) return;
  const st = $("cam-status"), img = $("cam-stream"), hint = $("cam-hint");
  const opened = (cam.cameras || []).filter((c) => c.opened);
  const cur = opened.find((c) => c.index === cam.selected) || null;
  cameraButtons(cam);
  if (cur) {
    st.textContent = `cam${cur.index} · ${cur.fps_actual} fps${cur.recording ? " · 录制中" : ""}`;
    hint.style.display = "none";
    img.classList.add("live");
    if (state.camSrcIndex !== cur.index) {
      state.camSrcIndex = cur.index;
      img.src = "/api/camera/stream?t=" + Date.now();
    }
  } else {
    st.textContent = cam.has_opencv ? "未连接" : "未装 opencv";
    hint.textContent =
      cam.error ||
      (cam.has_opencv
        ? "点「探测相机」→ 选设备 → 预览"
        : "需要 opencv：pip install 'rebot-capture[camera]'");
    hint.style.display = "block";
    img.classList.remove("live");
    state.camSrcIndex = null;
  }
}

function teleopKeys() {  const S = () => state.live?.teleop || {};
  window.addEventListener("keydown", async (e) => {
    if (e.repeat) return;
    if (e.target && ["INPUT", "TEXTAREA"].includes(e.target.tagName)) return;
    const t = S();
    switch (e.code) {
      case "Space": e.preventDefault(); await postJSON("/api/teleop/freeze", {}); break;
      case "KeyF": await postJSON("/api/teleop/speed", {}); break;
      case "KeyO": await postJSON("/api/teleop/mode", { mode: t.mode === "ori" ? "pos" : "ori" }); break;
      case "KeyH": await postJSON("/api/teleop/float", { on: !t.float }); break;
      case "KeyR": await postJSON("/api/teleop/align", {}); break;
      case "KeyQ": await postJSON("/api/teleop/joint", { hold: JRATE }); break;
      case "KeyA": await postJSON("/api/teleop/joint", { hold: -JRATE }); break;
      case "BracketLeft": await postJSON("/api/teleop/joint", { index: ((t.joint_index ?? 3) + 6) % 7 }); break;
      case "BracketRight": await postJSON("/api/teleop/joint", { index: ((t.joint_index ?? 3) + 1) % 7 }); break;
      case "Comma": await postJSON("/api/teleop/twist", { value: -TW }); break;
      case "Period": await postJSON("/api/teleop/twist", { value: TW }); break;
      case "Escape": await postJSON("/api/teleop/freeze", { on: true }); break;
      default: {
        const m = /^Digit([1-4])$/.exec(e.code);
        if (m) await postJSON("/api/teleop/preset", { action: e.shiftKey ? "record" : "goto", index: Number(m[1]) });
      }
    }
  });
  window.addEventListener("keyup", async (e) => {
    if (e.code === "KeyQ" || e.code === "KeyA") await postJSON("/api/teleop/joint", { hold: 0 });
    if (e.code === "Comma" || e.code === "Period") await postJSON("/api/teleop/twist", { value: 0 });
  });
}

/* ------------------------------------------------------------------ */
/* 数位板输入                                                          */
/* ------------------------------------------------------------------ */
function setupPad() {
  const pad = $("pad");
  let cursor = null;

  const send = (ev, touching) => {
    if (!state.wsTeleop || state.wsTeleop.readyState !== 1) return;
    const rect = pad.getBoundingClientRect();
    const x = (ev.clientX - rect.left) / rect.width;
    const y = (ev.clientY - rect.top) / rect.height;
    const sample = {
      type: "pen",
      data: {
        t: performance.now() / 1000,
        x: Math.max(0, Math.min(1, x)),
        y: Math.max(0, Math.min(1, y)),
        pressure: ev.pressure || (touching ? 0.5 : 0),
        tiltX: 0,
        tiltY: 0,
        twist: ev.twist || 0,
        touching,
        shift: state.zMode,
      },
    };
    state.touching = touching;
    state.lastSample = sample.data;
    state.wsTeleop.send(JSON.stringify(sample));

    $("pen-p").textContent = (sample.data.pressure || 0).toFixed(2);
    $("pen-tilt").textContent = "0,0";
    $("pen-touch").textContent = touching ? (state.zMode ? "落笔·升降" : "落笔") : "悬停";

    if (!cursor) {
      cursor = document.createElement("div");
      cursor.className = "pen-cursor";
      pad.appendChild(cursor);
    }
    cursor.style.left = (sample.data.x * rect.width) + "px";
    cursor.style.top = (sample.data.y * rect.height) + "px";
  };

  // 落笔期间以 50Hz 持续重发（保持服务端"输入未超时"，松手/断连即停）
  setInterval(() => {
    if (!state.touching || !state.lastSample) return;
    if (!state.wsTeleop || state.wsTeleop.readyState !== 1) return;
    state.lastSample.t = performance.now() / 1000;
    state.wsTeleop.send(JSON.stringify({ type: "pen", data: state.lastSample }));
  }, 20);

  pad.addEventListener("pointerdown", (ev) => {
    pad.setPointerCapture?.(ev.pointerId);
    send(ev, true);
  });
  pad.addEventListener("pointermove", (ev) => send(ev, ev.buttons > 0 || ev.pointerType === "pen" ? ev.pressure > 0 : false));
  pad.addEventListener("pointerup", (ev) => send(ev, false));
  pad.addEventListener("pointerleave", (ev) => { if (ev.buttons === 0) send(ev, false); });
}

/* ------------------------------------------------------------------ */
/* WebSocket                                                           */
/* ------------------------------------------------------------------ */
function setupWS() {
  const proto = location.protocol === "https:" ? "wss" : "ws";

  const connectState = () => {
    const ws = new WebSocket(`${proto}://${location.host}/ws/state`);
    ws.onmessage = (ev) => {
      try {
        const payload = JSON.parse(ev.data);
        state.live = payload.live;
        state.status = payload.status;
        renderChips();
        renderJoints();
        renderCore(payload.live?.teleop);
        renderCamera(payload.live?.camera);
      } catch (_) {}
    };
    ws.onopen = () => setDot("ok");
    ws.onclose = () => { setDot("bad"); setTimeout(connectState, 1500); };
    ws.onerror = () => ws.close();
  };

  const connectTeleop = () => {
    const ws = new WebSocket(`${proto}://${location.host}/ws/teleop`);
    ws.onopen = () => { state.wsTeleop = ws; };
    ws.onclose = () => { state.wsTeleop = null; setTimeout(connectTeleop, 1500); };
    ws.onerror = () => ws.close();
  };

  connectState();
  connectTeleop();
}

/* ------------------------------------------------------------------ */
/* 录制控制                                                            */
/* ------------------------------------------------------------------ */
async function refreshEpisodes(details = false) {
  const data = await api(`/api/episodes?details=${details ? "true" : "false"}`);
  renderEpisodes(data.episodes || []);
  return data.episodes || [];
}

function setupControls() {
  $("btn-cam-probe").onclick = async () => {
    $("cam-hint").textContent = "探测中…";
    try { await postJSON("/api/camera/probe", {}); } catch (e) { alert(e.message); }
  };
  $("btn-cam-close").onclick = async () => {
    try { await postJSON("/api/camera/close", {}); } catch (e) { alert(e.message); }
    state.camSrcIndex = null;
    $("cam-stream").classList.remove("live");
  };

  $("c-freeze").onclick = () => postJSON("/api/teleop/freeze", {});
  $("c-speed").onclick = () => postJSON("/api/teleop/speed", {});
  $("c-mode").onclick = () => {
    const t = state.live?.teleop || {};
    postJSON("/api/teleop/mode", { mode: t.mode === "ori" ? "pos" : "ori" });
  };
  $("c-float").onclick = () => {
    const t = state.live?.teleop || {};
    postJSON("/api/teleop/float", { on: !t.float });
  };
  $("c-align").onclick = () => postJSON("/api/teleop/align", {});

  $("btn-zmode").onclick = () => {
    state.zMode = !state.zMode;
    $("btn-zmode").textContent = "升降模式：" + (state.zMode ? "开" : "关");
    $("btn-zmode").classList.toggle("primary", state.zMode);
  };

  $("btn-session").onclick = async () => {
    try {
      if (state.sessionActive) {
        await api("/api/session/stop", "POST", {});
      } else {
        await api("/api/session/start", "POST", { task_id: "grasp-and-place", operator: "web" });
      }
      state.sessionActive = !state.sessionActive;
      updateButtons();
    } catch (e) { alert("会话操作失败：" + e.message); }
  };

  $("btn-start").onclick = async () => {
    try { await api("/api/episode/start", "POST", {}); } catch (e) { alert(e.message); }
  };

  const stop = async (success) => {
    try {
      const ep = await api("/api/episode/stop", "POST", { success });
      const failed = (ep.qc || []).filter((r) => !r.passed).map((r) => r.name);
      $("pack-result").textContent =
        `#${ep.index} 等级 ${ep.grade}（${ep.score} 分）` +
        (failed.length ? ` · 未过：${failed.join(", ")}` : " · 硬门全过 ✓");
      await refreshEpisodes();
    } catch (e) { alert("结束失败：" + e.message); }
  };
  $("btn-stop").onclick = () => stop(true);
  $("btn-stop-fail").onclick = () => stop(false);

  $("btn-discard").onclick = async () => {
    try { await api("/api/episode/discard", "POST", {}); $("pack-result").textContent = "已丢弃当前 episode"; }
    catch (e) { alert(e.message); }
  };

  $("btn-pack").onclick = async () => {
    try {
      const res = await api("/api/pack", "POST", { task_instruction: "grasp and place" });
      $("pack-result").textContent = `已打包：${res.dataset} · ${res.episodes} 集 / ${res.frames} 帧 / ${res.size_mb}MB → ${res.path}`;
    } catch (e) { alert("打包失败：" + e.message); }
  };

  $("btn-park").onclick = async () => {
    if (!confirm("确认回零？机械臂会以限速缓慢回到折叠零位（保持悬停）。")) return;
    $("pack-result").textContent = "回零中…（请勿触碰机械臂）";
    try {
      await api("/api/device/park", "POST", {});
      $("pack-result").textContent = "已回零并保持悬停 ✅";
    } catch (e) { alert("回零失败：" + e.message); }
  };

  const doReplay = async (speed) => {
    $("replay-status").textContent = `${speed}× 回放中…（按 Space 可中止）`;
    try {
      const r = await postJSON("/api/replay", { speed, max_joint_speed_deg: Math.min(720, 240 * speed) });
      $("replay-status").textContent = r.ok
        ? `完成：episode ${r.episode} · 原长 ${r.duration_s}s · ${speed}×`
        : `已中止（${r.reason || "冻结/力矩保护"}）`;
    } catch (e) {
      $("replay-status").textContent = "回放失败：" + e.message;
    }
  };
  $("btn-replay-1").onclick = () => doReplay(1.0);
  $("btn-replay-2").onclick = () => doReplay(2.0);
  $("btn-replay-4").onclick = () => doReplay(4.0);
}

/* ------------------------------------------------------------------ */
/* 启动                                                                */
/* ------------------------------------------------------------------ */
async function boot() {
  try {
    const health = await api("/api/health");
    $("version").textContent = "v" + health.version;
    setDot("ok");
  } catch (_) { setDot("bad"); }

  try {
    state.profile = await api("/api/profile");
    state.status = await api("/api/device/status");
    renderChips();
  } catch (_) {}

  try { await refreshEpisodes(); } catch (_) {}
  setupPad();
  setupWS();
  setupControls();
  coreButtons();
  teleopKeys();
  updateButtons();
}

boot();
