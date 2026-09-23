/* ==========================================================================
   采集流程 · 3D 场景（three.js，ES module，按需加载）
   - 模型：现成 CC-BY 模型（Poly Pizza），非自建
       Robot Arm        by Yali Izzo        CC-BY 3.0  https://poly.pizza/m/1JbGi41I1l1
       Tablet, Pen      by Poly by Google   CC-BY 3.0  https://poly.pizza/m/2LxocCCiDy-
   - 交互：自动转台 + 滚动驱动转角 + 指针拖拽（带惯性）+ 指针视差倾斜
   ========================================================================== */
import * as THREE from "three";
import { GLTFLoader } from "./vendor/three/examples/jsm/loaders/GLTFLoader.js";

const EASE = (t) => t * t * (3 - 2 * t);
const clamp = (v, a = 0, b = 1) => Math.min(b, Math.max(a, v));
const lerp = (a, b, t) => a + (b - a) * t;

/** 把模型缩到目标尺寸，底面中心落到原点 */
function normalize(obj, target) {
  const box = new THREE.Box3().setFromObject(obj);
  const size = box.getSize(new THREE.Vector3());
  const scale = target / Math.max(size.x, size.y, size.z);
  obj.scale.setScalar(scale);
  const box2 = new THREE.Box3().setFromObject(obj);
  const c = box2.getCenter(new THREE.Vector3());
  obj.position.sub(new THREE.Vector3(c.x, box2.min.y, c.z));
  return obj;
}

/** 地面：程序绘制的圆盘（径向高光 + 接触阴影），让"落地"一眼可读 */
function groundDisc(radius = 3.4) {
  const S = 1024;
  const c = document.createElement("canvas");
  c.width = c.height = S;
  const g = c.getContext("2d");
  // 底色：中心微亮
  const base = g.createRadialGradient(S / 2, S / 2, S * 0.03, S / 2, S / 2, S * 0.5);
  base.addColorStop(0, "rgba(58,192,255,.14)");
  base.addColorStop(0.45, "rgba(58,192,255,.055)");
  base.addColorStop(1, "rgba(58,192,255,0)");
  g.fillStyle = base;
  g.fillRect(0, 0, S, S);
  // 接触阴影：世界坐标 (x,z) → 贴图坐标
  const toUV = (x, z) => [((x / radius) + 1) / 2 * S, ((z / radius) + 1) / 2 * S];
  const blob = (x, z, r, a) => {
    const [u, v] = toUV(x, z);
    const rr = (r / radius) * S * 0.5;
    const gr = g.createRadialGradient(u, v, 0, u, v, rr);
    gr.addColorStop(0, `rgba(0,0,0,${a})`);
    gr.addColorStop(0.55, `rgba(0,0,0,${a * 0.45})`);
    gr.addColorStop(1, "rgba(0,0,0,0)");
    g.fillStyle = gr;
    g.beginPath();
    g.ellipse(u, v, rr, rr * 0.72, 0, 0, Math.PI * 2);
    g.fill();
  };
  blob(-0.38, -0.18, 0.62, 0.55);   // 机械臂底座
  blob(0.80, 0.88, 0.95, 0.42);     // 数位板下方
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 4;
  const mesh = new THREE.Mesh(
    new THREE.CircleGeometry(radius, 72),
    new THREE.MeshBasicMaterial({ map: tex, transparent: true, depthWrite: false })
  );
  mesh.rotation.x = -Math.PI / 2;
  mesh.position.y = -0.001;
  return mesh;
}

export async function initScene3D({ canvas, badge, poster, onStep }) {
  const reduce = matchMedia("(prefers-reduced-motion: reduce)").matches;
  const host = canvas.parentElement;
  const rect = () => {
    const r = host.getBoundingClientRect();
    return { w: Math.max(1, r.width), h: Math.max(1, r.height) };
  };

  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(1.75, window.devicePixelRatio || 1));
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.06;
  renderer.outputColorSpace = THREE.SRGBColorSpace;

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(34, 1, 0.05, 60);
  camera.position.set(2.58, 2.32, 4.05);
  camera.lookAt(0.36, 0.86, 0.30);

  scene.add(new THREE.HemisphereLight(0xcfe0ff, 0x0a0d13, 1.1));
  const key = new THREE.DirectionalLight(0xffffff, 2.1);
  key.position.set(3.4, 5.2, 3.2);
  scene.add(key);
  const rim = new THREE.DirectionalLight(0x3ac0ff, 1.9);
  rim.position.set(-4.2, 2.4, -3.4);
  scene.add(rim);
  const fill = new THREE.DirectionalLight(0x39d98a, 0.55);
  fill.position.set(-2.4, -1.6, 3.6);
  scene.add(fill);
  scene.add(groundDisc());

  const root = new THREE.Group();
  scene.add(root);

  const loader = new GLTFLoader();
  const load = (url) => new Promise((res, rej) => loader.load(url, res, undefined, rej));

  const [armGltf, tabGltf, penGltf] = await Promise.all([
    load("./assets/models/arm_yali.glb"),
    load("./assets/models/tablet.glb"),
    load("./assets/models/pen.glb"),
  ]);

  // 机械臂：转台
  const arm = normalize(armGltf.scene, 1.85);
  const armPivot = new THREE.Group();
  armPivot.add(arm);
  armPivot.position.set(-0.38, -0.28, -0.18);
  armPivot.rotation.z = -0.045;
  root.add(armPivot);

  // 数位板 + 笔：放在臂的前右侧，略微倾斜
  const tablet = normalize(tabGltf.scene, 1.28);
  const tabletPivot = new THREE.Group();
  tabletPivot.add(tablet);
  tabletPivot.position.set(0.80, 0.02, 0.88);
  tabletPivot.rotation.set(-0.08, -0.55, 0.02);
  root.add(tabletPivot);

  const pen = normalize(penGltf.scene, 0.72);
  const penPivot = new THREE.Group();
  penPivot.add(pen);
  penPivot.position.set(1.00, 0.10, 0.68);
  penPivot.rotation.set(-0.16, -0.45, 1.32);
  root.add(penPivot);

  // 「数据」小方块：只在第 3 步出现，从夹爪飞向数据集
  const GRIP_LOCAL = new THREE.Vector3(0.02, 1.78, 0.06);   // 夹爪在机械臂局部坐标里的位置
  const dot = new THREE.Mesh(
    new THREE.IcosahedronGeometry(0.075, 1),
    new THREE.MeshStandardMaterial({ color: 0x3ac0ff, emissive: 0x2ea8e6, emissiveIntensity: 1.5,
                                     roughness: 0.35, metalness: 0.1 })
  );
  dot.visible = false;
  root.add(dot);

  // ── 状态 ────────────────────────────────────────────────────────────
  const st = {
    progress: 0, target: 0,          // 0..1 滚动进度
    spin: 0, spinVel: 0,             // 拖拽惯性
    azim: 0.55, azimTarget: 0.55,    // 自动转台角度
    tilt: 0, tiltTarget: 0,          // 指针视差
    px: 0, py: 0,                    // 指针（-1..1）
    step: -1, visible: true,
  };

  // 指针拖拽
  let dragging = false, lastX = 0, moved = 0;
  canvas.addEventListener("pointerdown", (e) => {
    dragging = true; lastX = e.clientX; moved = 0;
    canvas.setPointerCapture(e.pointerId);
    canvas.style.cursor = "grabbing";
  });
  canvas.addEventListener("pointermove", (e) => {
    if (dragging) {
      const dx = e.clientX - lastX;
      lastX = e.clientX; moved += Math.abs(dx);
      st.spinVel += dx * 0.00035;
      st.spin += dx * 0.006;
    } else {
      const r = host.getBoundingClientRect();
      st.px = ((e.clientX - r.left) / r.width) * 2 - 1;
      st.py = ((e.clientY - r.top) / r.height) * 2 - 1;
      st.tiltTarget = st.px * 0.12;
    }
  });
  const endDrag = (e) => {
    if (!dragging) return;
    dragging = false;
    canvas.style.cursor = "grab";
    if (canvas.hasPointerCapture?.(e.pointerId)) canvas.releasePointerCapture(e.pointerId);
  };
  canvas.addEventListener("pointerup", endDrag);
  canvas.addEventListener("pointercancel", endDrag);
  canvas.addEventListener("pointerleave", () => { st.tiltTarget = 0; st.py = 0; });

  function resize() {
    const { w, h } = rect();
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
  resize();
  new ResizeObserver(resize).observe(host);

  // 离开视口就停（省电）
  new IntersectionObserver(([e]) => { st.visible = e.isIntersecting; },
    { rootMargin: "120px" }).observe(host);
  document.addEventListener("visibilitychange", () => { st.visible = !document.hidden; });

  // ── 每帧 ────────────────────────────────────────────────────────────
  const clock = new THREE.Clock();
  let t = 0;
  function frame() {
    const dt = Math.min(0.05, clock.getDelta());
    t += dt;
    st.progress = lerp(st.progress, st.target, 0.12);
    const p = st.progress;
    const step = p < 0.34 ? 0 : p < 0.67 ? 1 : 2;
    if (step !== st.step) {           // 步骤/文案与可见性无关，始终更新
      st.step = step;
      onStep?.(step);
    }
    if (st.visible) {
      // 转台：自动 + 滚动 + 拖拽惯性
      st.spinVel *= Math.pow(0.12, dt);
      st.spin += st.spinVel;
      st.azimTarget = 0.55 + p * Math.PI * 0.95;         // 滚动驱动转角
      st.azim = lerp(st.azim, st.azimTarget, 0.06);
      const azim = st.azim + st.spin;
      armPivot.rotation.y = azim;

      // 数位板：第 1 步向镜头抬起来，第 3 步放平并轻轻浮动
      const lift = step === 0 ? 1 : step === 1 ? 0.4 : 0;
      const floatY = reduce ? 0 : Math.sin(t * 1.1) * 0.022;
      tabletPivot.rotation.x = -0.08 + lift * -0.30;
      tabletPivot.rotation.z = 0.02 + (reduce ? 0 : Math.sin(t * 0.7) * 0.02);
      tabletPivot.position.y = 0.02 + floatY + lift * 0.06;
      penPivot.rotation.z = 1.32 + lift * -0.28;
      penPivot.position.y = 0.10 + floatY * 1.4;

      // 「数据」方块：第 3 步从夹爪飞向数位板
      const q = clamp((p - 0.67) / 0.33);
      dot.visible = q > 0.02;
      if (dot.visible) {
        // 起点跟着机械臂转（用局部坐标实时换算），终点在数位板上
        const a = armPivot.localToWorld(GRIP_LOCAL.clone());
        const b = new THREE.Vector3(0.95, 0.30, 0.95);
        const k = EASE(clamp(q * 1.35 % 1.0001));
        dot.position.lerpVectors(a, b, k);
        dot.position.y += Math.sin(k * Math.PI) * 0.42;
        dot.rotation.y += dt * 2.4;
        dot.scale.setScalar(0.75 + 0.25 * Math.sin(t * 6));
        dot.material.emissiveIntensity = 1.1 + 0.7 * Math.sin(t * 6);
      }

      // 指针视差
      st.tilt = lerp(st.tilt, st.tiltTarget, 0.06);
      root.rotation.z = st.tilt * 0.35;
      root.rotation.x = -st.py * 0.05;
      camera.position.y = lerp(camera.position.y, 2.32 + st.py * 0.12, 0.06);
      camera.lookAt(0.36, 0.86, 0.30);
    }
    renderer.render(scene, camera);
    requestAnimationFrame(frame);
  }
  frame();

  poster?.remove();
  canvas.style.cursor = "grab";
  return { setProgress: (p) => { st.target = clamp(p); } };
}
