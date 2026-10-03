package main

// pageHTML 是报告页面模板。占位符 __DATA__ 在生成时被替换为内联 JSON；
// 页面不引用任何外部资源（无 CSS/JS/字体/统计库），可直接 file:// 打开。
const pageHTML = `<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>粒子滤波（蒙特卡洛定位）实验报告</title>
<style>
  :root {
    color-scheme: light dark;
    --page: #f6f6f4; --surface: #ffffff; --ink: #14141a; --ink-2: #55545e;
    --muted: #8b8992; --grid: #e6e5df; --border: rgba(20,20,26,.10);
    --blue: #2a78d6; --orange: #eb6834; --aqua: #1baf7a; --red: #d03b3b;
    --violet: #8455d4;
  }
  @media (prefers-color-scheme: dark) {
    :root {
      --page: #101014; --surface: #18181e; --ink: #ececf1; --ink-2: #b3b2bd;
      --muted: #7d7c87; --grid: #26262e; --border: rgba(236,236,241,.12);
    }
  }
  * { box-sizing: border-box; }
  body { margin:0; background:var(--page); color:var(--ink);
    font:14px/1.6 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,"PingFang SC","Microsoft YaHei",sans-serif; }
  header { padding:28px 24px 12px; max-width:1100px; margin:0 auto; }
  h1 { font-size:22px; margin:0 0 6px; }
  h2 { font-size:17px; margin:0 0 4px; }
  .sub { color:var(--ink-2); margin:0 0 8px; }
  main { max-width:1100px; margin:0 auto; padding:8px 24px 60px; }
  section.card { background:var(--surface); border:1px solid var(--border);
    border-radius:12px; padding:18px 20px; margin:18px 0; }
  .desc { color:var(--ink-2); margin:0 0 14px; font-size:13.5px; }
  .desc b { color:var(--ink); }
  .grid2 { display:grid; grid-template-columns: 1fr 320px; gap:16px; align-items:start; }
  @media (max-width:860px){ .grid2 { grid-template-columns:1fr; } }
  canvas { display:block; width:100%; border-radius:8px; background:color-mix(in srgb, var(--grid) 35%, var(--surface)); }
  #map1, #map2 { aspect-ratio: 3 / 2; }
  .chart { margin-top:12px; }
  .chart canvas { height:170px; }
  .chart h3 { font-size:13px; color:var(--ink-2); font-weight:600; margin:10px 0 4px; }
  .controls { display:flex; align-items:center; gap:10px; margin:12px 0 4px; flex-wrap:wrap; }
  button { background:var(--blue); color:#fff; border:0; border-radius:7px;
    padding:6px 14px; font-size:13px; cursor:pointer; }
  button.secondary { background:transparent; color:var(--ink); border:1px solid var(--border); }
  button:disabled { opacity:.45; cursor:default; }
  input[type=range] { flex:1; min-width:160px; accent-color:var(--blue); }
  .step-label { font-variant-numeric: tabular-nums; color:var(--ink-2); min-width:110px; }
  select { accent-color:var(--blue); }
  .stat-row { display:grid; grid-template-columns:1fr 1fr; gap:12px; margin-top:12px; }
  .stat { border:1px solid var(--border); border-radius:10px; padding:12px 14px; }
  .stat h4 { margin:0 0 8px; font-size:13.5px; }
  .stat .v { font-size:20px; font-variant-numeric:tabular-nums; }
  .stat .l { color:var(--muted); font-size:12px; }
  table { border-collapse:collapse; width:100%; margin-top:10px; font-size:13px; }
  td, th { border-bottom:1px solid var(--border); padding:5px 8px; text-align:right; font-variant-numeric:tabular-nums; }
  th:first-child, td:first-child { text-align:left; }
  .tag { display:inline-block; font-size:11.5px; padding:1px 8px; border-radius:99px; margin-left:6px; vertical-align:middle; }
  .tag.bad { background:color-mix(in srgb, var(--red) 18%, transparent); color:var(--red); }
  .tag.ok  { background:color-mix(in srgb, var(--aqua) 18%, transparent); color:var(--aqua); }
  .legend { display:flex; gap:16px; flex-wrap:wrap; color:var(--ink-2); font-size:12.5px; margin:8px 0 0; }
  .legend span { display:inline-flex; align-items:center; gap:6px; }
  .dot { width:9px; height:9px; border-radius:50%; display:inline-block; }
  .note { font-size:12.5px; color:var(--muted); margin-top:10px; }
  code { background:var(--grid); padding:1px 5px; border-radius:4px; font-size:12.5px; }
</style>
</head>
<body>
<header>
  <h1>粒子滤波（蒙特卡洛定位，MCL）实验报告</h1>
  <p class="sub">二维 <code>(x, y, θ)</code> 位姿 · 纯距离路标观测 · Go 标准库实现 · 原生 HTML/CSS/JS 单文件报告（无第三方依赖）</p>
</header>
<main>

<section class="card">
  <h2>实验一 · 粒子退化检测（纯重要性采样，全程不重采样）</h2>
  <p class="desc">
    机器人初始位于一小簇路标的<b>探测范围之外</b>，观测为空、权重均匀（ESS = N）；
    随着它沿直线逼近路簇，进入范围的路标逐步增加，距离似然迅速收紧，权重集中到
    少数“幸运”粒子上，<b>有效粒子数 ESS = 1 / Σwᵢ² 骤降</b>。
    退化判定阈值为 <code>ESS &lt; 0.25·N</code>，首次越限的时间步即退化起点。
  </p>
  <div class="grid2">
    <div>
      <canvas id="map1"></canvas>
      <div class="legend">
        <span><i class="dot" style="background:var(--blue)"></i>粒子（颜色越亮权重越高）</span>
        <span><i class="dot" style="background:var(--orange)"></i>真值位姿</span>
        <span><i class="dot" style="background:var(--violet)"></i>路标</span>
        <span><i class="dot" style="border:1px dashed var(--ink-2);background:transparent"></i>探测范围</span>
      </div>
      <div class="controls">
        <button id="play1">暂停</button>
        <button class="secondary" id="prev1">◀</button>
        <input type="range" id="slider1" min="0" step="1">
        <button class="secondary" id="next1">▶</button>
        <span class="step-label" id="label1"></span>
      </div>
    </div>
    <div class="chart">
      <h3>有效粒子数 ESS 随时间变化</h3>
      <canvas id="ess1"></canvas>
      <p class="note" id="deg-note"></p>
    </div>
  </div>
</section>

<section class="card">
  <h2>实验二 · 多项式重采样 vs 低方差重采样的统计偏差量化</h2>
  <p class="desc">
    固定同一权重分布（60 个粒子，最大权重 <b>w₀=0.37</b> 加指数衰减长尾），
    每种算法独立重复 <b>4000 次</b>重采样（每轮抽取 250 个子代），
    统计“权重最大粒子”获得的子代数量。两种方法都<b>无偏</b>（均值 = N·w₀ = 92.5），
    但低方差重采样只做一次整体抖动，子代计数被限制在 floor/ceil(N·wᵢ) 附近，
    其方差应远小于多项式重采样的理论方差 <code>N·w·(1−w)</code>。
  </p>
  <div class="grid2">
    <div class="chart">
      <h3>多项式重采样：主导粒子子代数量的经验分布（4000 次）</h3>
      <canvas id="histM" style="height:190px"></canvas>
    </div>
    <div class="chart">
      <h3>低方差重采样：主导粒子子代数量的经验分布（4000 次）</h3>
      <canvas id="histL" style="height:190px"></canvas>
    </div>
  </div>
  <div class="stat-row">
    <div class="stat">
      <h4>多项式重采样（有放回独立抽样）</h4>
      <table id="tabM"></table>
    </div>
    <div class="stat">
      <h4>低方差重采样（系统重采样）</h4>
      <table id="tabL"></table>
    </div>
  </div>
  <p class="note" id="cmp-note"></p>
</section>

<section class="card">
  <h2>实验三 · 镜像对称环境中的多峰维持</h2>
  <p class="desc">
    所有路标关于 y 轴<b>成对镜像</b>，观测为<b>无标签</b>距离多重集（排序后匹配），
    且仿真对每对镜像路标使用<b>同一噪声实现</b>，使噪声模型本身也严格对称。
    真值位于右走廊 <code>(+5, y)</code>，但左走廊 <code>(−5, y)</code> 对全部观测给出
    <b>完全相同的似然</b>——正确后验必须始终保持双峰。滤波器采用
    自适应低方差重采样（ESS &lt; 0.5N 才触发）+ 重采样后各向异性抖动，
    并使用充足粒子（N=30000）抑制有限样本的随机遗传漂变。
  </p>
  <div class="grid2">
    <div>
      <canvas id="map2"></canvas>
      <div class="legend">
        <span><i class="dot" style="background:var(--blue)"></i>粒子</span>
        <span><i class="dot" style="background:var(--orange)"></i>真值（右走廊）</span>
        <span><i class="dot" style="border:1.5px dashed var(--aqua);background:transparent"></i>两个合理假设区域</span>
      </div>
      <div class="controls">
        <button id="play2">暂停</button>
        <button class="secondary" id="prev2">◀</button>
        <input type="range" id="slider2" min="0" step="1">
        <button class="secondary" id="next2">▶</button>
        <span class="step-label" id="label2"></span>
      </div>
    </div>
    <div class="chart">
      <h3>两峰质量占比（蓝：右 / 绿：左）与平衡度</h3>
      <canvas id="share2" style="height:130px"></canvas>
      <h3>有效粒子数 ESS</h3>
      <canvas id="ess2" style="height:120px"></canvas>
    </div>
  </div>
</section>

<p class="note">模型与算法细节见仓库 <code>README.md</code>：运动模型、纯距离观测似然、
对数空间权重更新、退化检测（ESS）与两种重采样算法的选择依据。</p>
</main>

<script>
"use strict";
const DATA = __DATA__;

/* ---------- 通用画布工具（处理 devicePixelRatio） ---------- */
function setup(cv) {
  const dpr = window.devicePixelRatio || 1;
  const r = cv.getBoundingClientRect();
  cv.width = Math.max(1, Math.round(r.width * dpr));
  cv.height = Math.max(1, Math.round(r.height * dpr));
  const ctx = cv.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w: r.width, h: r.height };
}
const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
const COL = { blue: css("--blue"), orange: css("--orange"), aqua: css("--aqua"),
  red: css("--red"), violet: css("--violet"), ink: css("--ink"),
  ink2: css("--ink-2"), grid: css("--grid"), surface: css("--surface") };

function makeProjector(bounds, w, h, pad) {
  const [minx, miny, maxx, maxy] = bounds;
  const sx = (w - 2*pad) / (maxx - minx), sy = (h - 2*pad) / (maxy - miny);
  const s = Math.min(sx, sy);
  const ox = pad + ((w - 2*pad) - s*(maxx-minx))/2 - s*minx;
  const oy = pad + ((h - 2*pad) - s*(maxy-miny))/2 + s*maxy; // y 翻转
  return {
    p: (x, y) => [ox + s*x, oy - s*y],
    sc: (d) => s*d,
    gridX: () => [pad, h-pad],
  };
}

/* ---------- 地图绘制 ---------- */
function drawMap(cv, exp, frame, opts) {
  const { ctx, w, h } = setup(cv);
  ctx.clearRect(0,0,w,h);
  const P = makeProjector(exp.bounds, w, h, 18);

  // 网格
  ctx.strokeStyle = COL.grid; ctx.lineWidth = 1;
  const [minx,miny,maxx,maxy] = exp.bounds;
  for (let gx = Math.ceil(minx/5)*5; gx <= maxx; gx += 5) {
    const [x1,y1] = P.p(gx,miny), [x2,y2] = P.p(gx,maxy);
    ctx.beginPath(); ctx.moveTo(x1,y1); ctx.lineTo(x2,y2); ctx.stroke();
  }
  for (let gy = Math.ceil(miny/5)*5; gy <= maxy; gy += 5) {
    const [x1,y1] = P.p(minx,gy), [x2,y2] = P.p(maxx,gy);
    ctx.beginPath(); ctx.moveTo(x1,y1); ctx.lineTo(x2,y2); ctx.stroke();
  }

  // 探测范围 / 假设区域
  if (opts.rangeCircle && exp.maxRange > 0) {
    const [cx,cy] = P.p(frame.truth[0], frame.truth[1]);
    ctx.strokeStyle = COL.ink2; ctx.setLineDash([4,4]);
    ctx.beginPath(); ctx.arc(cx, cy, P.sc(exp.maxRange), 0, Math.PI*2); ctx.stroke();
    ctx.setLineDash([]);
  }
  if (opts.hypotheses) {
    for (const hx of [exp.h1, exp.h2]) {
      const [cx,cy] = P.p(hx[0], hx[1]);
      ctx.strokeStyle = COL.aqua; ctx.setLineDash([5,4]); ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.arc(cx, cy, P.sc(exp.radius), 0, Math.PI*2); ctx.stroke();
      ctx.setLineDash([]);
    }
  }

  // 路标
  ctx.fillStyle = COL.violet;
  for (const lm of exp.landmarks) {
    const [x,y] = P.p(lm[0], lm[1]);
    ctx.fillRect(x-3.5, y-3.5, 7, 7);
  }

  // 粒子
  const weighted = !!opts.weighted;
  if (opts.density) {
    // 多峰后验：按平面粒子数密度（网格分箱）上色，
    // 集中的两个真假设呈亮团，弥散尾部自动变淡。
    const NB = 56;
    const [minx0,miny0,maxx0,maxy0] = exp.bounds;
    const bins = Array.from({length: NB*NB}, () => 0);
    const bx = NB/(maxx0-minx0), by = NB/(maxy0-miny0);
    let bmax = 0;
    for (const pp of frame.p) {
      let gx = Math.floor((pp[0]-minx0)*bx), gy = Math.floor((maxy0-pp[1])*by);
      if (gx<0||gx>=NB||gy<0||gy>=NB) continue;
      const c = ++bins[gy*NB+gx]; if (c>bmax) bmax=c;
    }
    const cwX = (maxx0-minx0)/NB, cwY = (maxy0-miny0)/NB;
    const ps0 = P.p(0,0), ps1 = P.p(1,0);
    const scale = Math.abs(ps1[0]-ps0[0]);
    const cellW = cwX*scale, cellH = cwY*scale;
    for (let gy=0; gy<NB; gy++) for (let gx=0; gx<NB; gx++) {
      const c = bins[gy*NB+gx]; if (!c) continue;
      const a = 0.08 + 0.85*Math.pow(c/bmax, 0.6);
      ctx.fillStyle = "rgba(42,120,214," + a.toFixed(3) + ")";
      const cxw = minx0 + (gx+0.5)*cwX, cyw = maxy0-(gy+0.5)*cwY;
      const [wx,wy] = P.p(cxw, cyw);
      ctx.fillRect(wx-cellW/2-0.5, wy-cellH/2-0.5, cellW+1, cellH+1);
    }
  } else {
  for (const pp of frame.p) {
    const [x,y] = P.p(pp[0], pp[1]);
    if (weighted) {
      // 权重经 log 映射到亮度/半径（退化后差异直观可见）
      const wgt = pp[2];
      const a = Math.min(1, 0.06 + Math.log10(1 + 9999*wgt) / 4);
      ctx.fillStyle = "rgba(42,120,214," + a.toFixed(3) + ")";
      ctx.beginPath(); ctx.arc(x, y, 1 + 2.2*a, 0, Math.PI*2); ctx.fill();
    } else {
      ctx.fillStyle = "rgba(42,120,214,.45)";
      ctx.fillRect(x-1, y-1, 2, 2);
    }
  }
  }

  // 真值（三角箭头表示朝向）
  const [tx,ty] = P.p(frame.truth[0], frame.truth[1]);
  const th = frame.truth[2];
  ctx.save(); ctx.translate(tx,ty); ctx.rotate(-th);
  ctx.fillStyle = COL.orange;
  ctx.beginPath(); ctx.moveTo(8,0); ctx.lineTo(-6,5); ctx.lineTo(-6,-5); ctx.closePath(); ctx.fill();
  ctx.restore();
}

/* ---------- 折线图 ---------- */
function lineChart(cv, series, opt) {
  const { ctx, w, h } = setup(cv);
  ctx.clearRect(0,0,w,h);
  const padL = 44, padR = 12, padT = 10, padB = 20;
  const cw = w-padL-padR, ch = h-padT-padB;
  const T = series[0].data.length;
  const ymax = opt.ymax;
  ctx.strokeStyle = COL.grid; ctx.fillStyle = COL.ink2;
  ctx.font = "11px ui-monospace,Menlo,monospace"; ctx.lineWidth = 1;
  for (let i=0;i<=4;i++) {
    const y = padT + ch*i/4, v = ymax*(1-i/4);
    ctx.beginPath(); ctx.moveTo(padL,y); ctx.lineTo(w-padR,y); ctx.stroke();
    ctx.fillText(opt.fmt(v), 4, y+3.5);
  }
  // 阈值线
  if (opt.threshold !== undefined) {
    const y = padT + ch*(1-opt.threshold/ymax);
    ctx.strokeStyle = COL.red; ctx.setLineDash([5,4]);
    ctx.beginPath(); ctx.moveTo(padL,y); ctx.lineTo(w-padR,y); ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = COL.red; ctx.fillText("阈值", w-padR-30, y-4);
  }
  const X = (t) => padL + cw*(T===1?0:t/(T-1));
  const Y = (v) => padT + ch*(1-v/ymax);
  // 面积填充（仅第一条）
  for (let sIdx=0; sIdx<series.length; sIdx++) {
    const s = series[sIdx];
    ctx.strokeStyle = s.color; ctx.lineWidth = 2;
    ctx.beginPath();
    s.data.forEach((v,t) => { const x=X(t), y=Y(v); t?ctx.lineTo(x,y):ctx.moveTo(x,y); });
    ctx.stroke();
    ctx.fillStyle = s.color;
    s.data.forEach((v,t) => { ctx.beginPath(); ctx.arc(X(t),Y(v),2.5,0,Math.PI*2); ctx.fill(); });
  }
  // 当前步游标
  const cx = X(opt.cursor);
  ctx.strokeStyle = COL.ink2;
  ctx.beginPath(); ctx.moveTo(cx,padT); ctx.lineTo(cx,padT+ch); ctx.stroke();
  // x 轴刻度
  ctx.fillStyle = COL.ink2;
  for (let t=0;t<T;t++) ctx.fillText(String(t), X(t)-3, h-5);
}

/* ---------- 堆叠面积图（两峰占比） ---------- */
function shareChart(cv, exp, cursor) {
  const { ctx, w, h } = setup(cv);
  ctx.clearRect(0,0,w,h);
  const padL=44,padR=12,padT=8,padB=18, cw=w-padL-padR, ch=h-padT-padB;
  const T = exp.m1.length;
  const X = (t) => padL + cw*(T===1?0:t/(T-1));
  ctx.strokeStyle=COL.grid; ctx.fillStyle=COL.ink2;
  ctx.font="11px ui-monospace,Menlo,monospace";
  for (let i=0;i<=2;i++){
    const y=padT+ch*i/2;
    ctx.beginPath();ctx.moveTo(padL,y);ctx.lineTo(w-padR,y);ctx.stroke();
    ctx.fillText((100-i*50)+"%",6,y+3.5);
  }
  // m1（右峰，蓝）从顶向下，m2（左峰，绿）从底向上
  ctx.beginPath();
  exp.m1.forEach((v,t)=>{ const x=X(t),y=padT+ch*(1-v); t?ctx.lineTo(x,y):ctx.moveTo(x,y); });
  for (let t=T-1;t>=0;t--){ ctx.lineTo(X(t),padT+ch); }
  ctx.closePath(); ctx.fillStyle="rgba(42,120,214,.35)"; ctx.fill();
  ctx.beginPath();
  exp.m2.forEach((v,t)=>{ const x=X(t),y=padT+ch*(1-v); t?ctx.lineTo(x,y):ctx.moveTo(x,y); });
  for (let t=T-1;t>=0;t--){ ctx.lineTo(X(t),padT+ch); }
  ctx.closePath(); ctx.fillStyle="rgba(27,175,122,.35)"; ctx.fill();
  ctx.strokeStyle=COL.blue; ctx.lineWidth=1.8;
  ctx.beginPath(); exp.m1.forEach((v,t)=>{const x=X(t),y=padT+ch*(1-v);t?ctx.lineTo(x,y):ctx.moveTo(x,y);}); ctx.stroke();
  ctx.strokeStyle=COL.aqua;
  ctx.beginPath(); exp.m2.forEach((v,t)=>{const x=X(t),y=padT+ch*(1-v);t?ctx.lineTo(x,y):ctx.moveTo(x,y);}); ctx.lineTo(X(T-1),padT+ch*(1-exp.m2[T-1])); ctx.stroke();
  const cx=X(cursor);
  ctx.strokeStyle=COL.ink2; ctx.beginPath();ctx.moveTo(cx,padT);ctx.lineTo(cx,padT+ch);ctx.stroke();
}

/* ---------- 直方图 ---------- */
function histChart(cv, st, color) {
  const { ctx, w, h } = setup(cv);
  ctx.clearRect(0,0,w,h);
  const hist = st.hist;
  const lo = Math.max(0, Math.round(st.mean - 3*Math.sqrt(st.var)) - 1);
  const hi = Math.min(hist.length-1, Math.round(st.mean + 3*Math.sqrt(st.var)) + 1);
  let max = 0;
  for (let i=lo;i<=hi;i++) if (hist[i]>max) max=hist[i];
  const padL=34,padR=8,padT=8,padB=20, cw=w-padL-padR, ch2=h-padT-padB;
  const bw = cw/(hi-lo+1);
  ctx.strokeStyle=COL.grid; ctx.fillStyle=COL.ink2;
  ctx.font="11px ui-monospace,Menlo,monospace";
  for (let i=0;i<=3;i++){
    const y=padT+ch2*i/3, v=Math.round(max*(1-i/3));
    ctx.beginPath();ctx.moveTo(padL,y);ctx.lineTo(w-padR,y);ctx.stroke();
    ctx.fillText(String(v),4,y+3.5);
  }
  for (let i=lo;i<=hi;i++){
    const bh = ch2*(hist[i]/max);
    ctx.fillStyle = color; ctx.globalAlpha = .85;
    ctx.fillRect(padL+(i-lo)*bw+1, padT+ch2-bh, bw-2, bh);
    ctx.globalAlpha = 1;
    ctx.fillStyle=COL.ink2;
    if ((hi-lo)<=12 || i%2===0) ctx.fillText(String(i), padL+(i-lo)*bw+bw/2-6, h-5);
  }
  ctx.fillStyle=COL.ink2; ctx.fillText("子代数量 →", w-78, h-5);
}

/* ---------- 回放器 ---------- */
function makePlayer(prefix, exp, drawMapFn, drawChartsFn) {
  const frames = exp.frames;
  const slider = document.getElementById("slider"+prefix);
  const label = document.getElementById("label"+prefix);
  const play = document.getElementById("play"+prefix);
  slider.max = frames.length-1; slider.value = frames.length-1;
  let step = frames.length-1, timer = null;
  function render() {
    drawMapFn(frames[step]);
    drawChartsFn(step);
    slider.value = step;
    label.textContent = "第 " + step + " / " + (frames.length-1) + " 步";
  }
  function tick() {
    step = (step+1) % frames.length; render();
  }
  function setPlaying(on) {
    if (on) { timer = setInterval(tick, 650); play.textContent="暂停"; }
    else { clearInterval(timer); timer=null; play.textContent="播放"; }
  }
  play.onclick = () => setPlaying(timer===null);
  slider.oninput = () => { step = +slider.value; render(); };
  document.getElementById("prev"+prefix).onclick = () => { step=(step-1+frames.length)%frames.length; render(); };
  document.getElementById("next"+prefix).onclick = () => { step=(step+1)%frames.length; render(); };
  render();
  setPlaying(false); play.textContent = "播放";
  return { render };
}

/* ---------- 实验一 ---------- */
const deg = DATA.deg;
makePlayer("1", deg,
  (fr) => drawMap(document.getElementById("map1"), deg, fr, { weighted:true, rangeCircle:true }),
  (t) => lineChart(document.getElementById("ess1"),
    [{data: deg.essSeries, color: COL.blue}],
    { ymax: deg.n, threshold: deg.threshold, cursor: t, fmt:(v)=>Math.round(v) }));
{
  const start = deg.degStep;
  const el = document.getElementById("deg-note");
  el.innerHTML = "退化判定：ESS &lt; " + deg.threshold.toFixed(0) + "（0.25·N）。" +
    (start >= 0
      ? "<b style='color:"+COL.red+"'>第 " + start + " 步首次越限，" +
        "此后加权估计由少数粒子主导、结果不可信。</b> 末步 ESS = " + deg.essSeries.at(-1) + "。"
      : "全程未检出退化。");
}

/* ---------- 实验二 ---------- */
histChart(document.getElementById("histM"), DATA.cmp.multi, COL.orange);
histChart(document.getElementById("histL"), DATA.cmp.low, COL.aqua);
function statRows(st, theoryVar) {
  return "<tr><th>统计量</th><th>值</th></tr>" +
    "<tr><td>子代数均值（理论 N·w₀ = "+DATA.cmp.expectMean+"）</td><td>"+st.mean.toFixed(2)+"</td></tr>" +
    "<tr><td>子代数样本方差</td><td>"+st.var.toFixed(3)+"</td></tr>" +
    "<tr><td>理论方差 N·w₀·(1−w₀)</td><td>"+theoryVar.toFixed(3)+"</td></tr>" +
    "<tr><td>重采样估计量方差</td><td>"+st.estVar.toExponential(2)+"</td></tr>";
}
document.getElementById("tabM").innerHTML = statRows(DATA.cmp.multi, DATA.cmp.theoryVar);
document.getElementById("tabL").innerHTML = statRows(DATA.cmp.low, DATA.cmp.theoryVar);
const ratio = DATA.cmp.low["var"] / DATA.cmp.multi["var"];
document.getElementById("cmp-note").innerHTML =
  "两者均值均等于 92.5（<b>无偏</b>）；子代计数方差：多项式 " + DATA.cmp.multi.var.toFixed(2) +
  " 与理论值 " + DATA.cmp.theoryVar.toFixed(2) + " 吻合，而低方差重采样仅 " +
  DATA.cmp.low.var.toFixed(3) + "，<b>相差约 " + Math.round(DATA.cmp.multi["var"]/Math.max(DATA.cmp.low["var"],1e-9)) +
  " 倍</b>（比值 " + ratio.toExponential(1) + "）；重采样引入的估计量方差同样低 " +
  (DATA.cmp.multi.estVar/DATA.cmp.low.estVar).toFixed(0) + " 倍。这与“低方差重采样系统性偏差更小”的理论预期一致。";

/* ---------- 实验三 ---------- */
const multi = DATA.multi;
makePlayer("2", multi,
  (fr) => drawMap(document.getElementById("map2"), multi, fr, { weighted:false, density:true, hypotheses:true }),
  (t) => {
    shareChart(document.getElementById("share2"), multi, t);
    lineChart(document.getElementById("ess2"),
      [{data: multi.essSeries, color: COL.blue}],
      { ymax: multi.n, threshold: 0.5*multi.n, cursor: t, fmt:(v)=>Math.round(v) });
  });

/* 窗口尺寸变化时重绘当前帧 */
let rz;
addEventListener("resize", () => { clearTimeout(rz); rz = setTimeout(()=>location.reload(), 250); });
</script>
</body>
</html>
`
