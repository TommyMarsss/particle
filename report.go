package particle

import (
	"encoding/json"
	"io"
	"math"
	"strings"
)

// JSON view of a run, with coordinates rounded to keep the embedded
// payload small.
type jsonRun struct {
	Name           string       `json:"name"`
	Description    string       `json:"description"`
	Landmarks      [][2]float64 `json:"landmarks"`
	NParticles     int          `json:"nParticles"`
	EssThreshold   float64      `json:"essThreshold"`
	Degenerate     bool         `json:"degenerate"`
	DegeneracyStep int          `json:"degeneracyStep"`
	Frames         []jsonFrame  `json:"frames"`
}

type jsonFrame struct {
	True      [3]float64   `json:"true"`
	ESS       float64      `json:"ess"`
	Resampled bool         `json:"rs"`
	Particles [][3]float64 `json:"parts"` // x, y, weight
}

func roundTo(x, p float64) float64 { return math.Round(x*p) / p }

func toJSONRun(r *RunResult) jsonRun {
	jr := jsonRun{
		Name:           r.Name,
		Description:    r.Description,
		NParticles:     r.NumParticles,
		EssThreshold:   r.Degeneracy.Threshold,
		Degenerate:     r.Degeneracy.Degenerate,
		DegeneracyStep: r.Degeneracy.FirstStep,
	}
	for _, lm := range r.Landmarks {
		jr.Landmarks = append(jr.Landmarks, [2]float64{roundTo(lm.X, 100), roundTo(lm.Y, 100)})
	}
	for _, f := range r.Frames {
		jf := jsonFrame{
			True:      [3]float64{roundTo(f.True.X, 100), roundTo(f.True.Y, 100), roundTo(f.True.Theta, 1000)},
			ESS:       roundTo(f.ESS, 10),
			Resampled: f.Resampled,
		}
		for _, p := range f.Particles {
			jf.Particles = append(jf.Particles, [3]float64{
				roundTo(p.Pose.X, 100), roundTo(p.Pose.Y, 100), roundTo(p.Weight, 100000),
			})
		}
		jr.Frames = append(jr.Frames, jf)
	}
	return jr
}

// WriteHTML renders one or more runs as a single self-contained static HTML
// file (all data and logic inlined, no external resources) that replays the
// particle distribution frame by frame and plots the ESS curve.
func WriteHTML(w io.Writer, runs []*RunResult) error {
	jr := make([]jsonRun, len(runs))
	for i, r := range runs {
		jr[i] = toJSONRun(r)
	}
	data, err := json.Marshal(jr)
	if err != nil {
		return err
	}
	page := strings.Replace(htmlPage, "__DATA__", string(data), 1)
	_, err = io.WriteString(w, page)
	return err
}

const htmlPage = `<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<title>粒子滤波（蒙特卡洛定位）回放报告</title>
<style>
body{font-family:-apple-system,"Helvetica Neue",Arial,"PingFang SC",sans-serif;margin:24px;background:#fafafa;color:#222}
h1{font-size:20px;margin-bottom:12px}
.tabs button{margin:0 8px 8px 0;padding:6px 14px;border:1px solid #ccc;background:#fff;cursor:pointer;border-radius:4px;font-size:14px}
.tabs button.active{background:#246bfe;color:#fff;border-color:#246bfe}
#desc{max-width:980px;color:#555;font-size:14px;line-height:1.6}
.panel{display:flex;gap:24px;margin-top:8px;flex-wrap:wrap}
canvas{background:#fff;border:1px solid #ddd;border-radius:4px}
.controls{margin-top:10px;display:flex;gap:12px;align-items:center;flex-wrap:wrap;font-size:14px}
.controls button{padding:4px 14px;border:1px solid #ccc;background:#fff;cursor:pointer;border-radius:4px}
.badge{padding:2px 10px;border-radius:10px;font-size:12px}
.badge.ok{background:#e2f5e9;color:#1c7c3c}
.badge.warn{background:#fdeaea;color:#b3261e}
#stats{font-size:14px;line-height:2}
.legend{font-size:12px;color:#777;margin-top:6px}
</style>
</head>
<body>
<h1>粒子滤波（蒙特卡洛定位）回放报告</h1>
<div class="tabs" id="tabs"></div>
<p id="desc"></p>
<div class="panel">
  <div>
    <canvas id="map" width="640" height="640"></canvas>
    <div class="controls">
      <button id="playBtn">播放</button>
      <input id="slider" type="range" min="0" value="0" style="width:220px">
      <span id="stepLabel"></span>
      <select id="speed">
        <option value="5">5 帧/秒</option>
        <option value="15" selected>15 帧/秒</option>
        <option value="30">30 帧/秒</option>
      </select>
    </div>
    <div class="legend">红点=地标，绿点+箭头=真实位姿，蓝点=粒子（越不透明权重越大）</div>
  </div>
  <div>
    <canvas id="chart" width="640" height="300"></canvas>
    <div class="legend">蓝线=有效粒子数 ESS，红色虚线=退化阈值，橙色竖线=退化开始时间步，底部蓝刻度=发生重采样的帧</div>
    <div id="stats"></div>
  </div>
</div>
<script>
var RUNS = __DATA__;
var scen = 0, frame = 0, timer = null;
var mapC = document.getElementById('map'), mx = mapC.getContext('2d');
var chartC = document.getElementById('chart'), cx = chartC.getContext('2d');
var boundsCache = [];

function curRun(){ return RUNS[scen]; }

function computeBounds(run){
  var minX=1e18,minY=1e18,maxX=-1e18,maxY=-1e18;
  function eat(x,y){ if(x<minX)minX=x; if(x>maxX)maxX=x; if(y<minY)minY=y; if(y>maxY)maxY=y; }
  run.landmarks.forEach(function(l){ eat(l[0],l[1]); });
  run.frames.forEach(function(f){ eat(f.true[0],f.true[1]); });
  var gx=(maxX-minX)*0.12+0.5, gy=(maxY-minY)*0.12+0.5;
  return {minX:minX-gx, maxX:maxX+gx, minY:minY-gy, maxY:maxY+gy};
}

function w2p(b,W,H,x,y){
  var s=Math.min(W/(b.maxX-b.minX), H/(b.maxY-b.minY));
  var cx0=(b.minX+b.maxX)/2, cy0=(b.minY+b.maxY)/2;
  return [W/2+(x-cx0)*s, H/2-(y-cy0)*s];
}

function drawMap(){
  var run=curRun(), fr=run.frames[frame], W=mapC.width, H=mapC.height, b=boundsCache[scen];
  mx.clearRect(0,0,W,H);
  // true path up to current frame
  mx.strokeStyle='rgba(28,160,80,0.45)'; mx.lineWidth=1.5; mx.beginPath();
  for(var i=0;i<=frame;i++){
    var p=w2p(b,W,H,run.frames[i].true[0],run.frames[i].true[1]);
    if(i===0) mx.moveTo(p[0],p[1]); else mx.lineTo(p[0],p[1]);
  }
  mx.stroke();
  // landmarks
  run.landmarks.forEach(function(l){
    var p=w2p(b,W,H,l[0],l[1]);
    mx.fillStyle='#d33'; mx.fillRect(p[0]-4,p[1]-4,8,8);
  });
  // particles, opacity scaled by weight
  var maxW=0;
  fr.parts.forEach(function(pt){ if(pt[2]>maxW) maxW=pt[2]; });
  fr.parts.forEach(function(pt){
    var p=w2p(b,W,H,pt[0],pt[1]);
    var a = maxW>0 ? 0.12+0.88*pt[2]/maxW : 0.5;
    mx.fillStyle='rgba(36,107,254,'+a.toFixed(3)+')';
    mx.beginPath(); mx.arc(p[0],p[1],2.2,0,6.2832); mx.fill();
  });
  // true pose
  var tp=w2p(b,W,H,fr.true[0],fr.true[1]);
  mx.fillStyle='#1ca050';
  mx.beginPath(); mx.arc(tp[0],tp[1],5,0,6.2832); mx.fill();
  mx.strokeStyle='#1ca050'; mx.lineWidth=2; mx.beginPath();
  mx.moveTo(tp[0],tp[1]);
  mx.lineTo(tp[0]+16*Math.cos(-fr.true[2]), tp[1]+16*Math.sin(-fr.true[2]));
  mx.stroke();
  mx.lineWidth=1;
}

function drawChart(){
  var run=curRun(), W=chartC.width, H=chartC.height, pad=40, bottom=H-24, top=12;
  cx.clearRect(0,0,W,H);
  var n=run.nParticles, T=run.frames.length-1;
  function X(t){ return pad+(W-pad-10)*t/Math.max(T,1); }
  function Y(e){ return bottom-(bottom-top)*Math.min(e,n*1.05)/(n*1.05); }
  cx.strokeStyle='#ccc'; cx.beginPath();
  cx.moveTo(pad,top); cx.lineTo(pad,bottom); cx.lineTo(W-10,bottom); cx.stroke();
  cx.fillStyle='#666'; cx.font='11px sans-serif';
  cx.fillText('ESS', 6, top+4);
  cx.fillText('N='+n, 4, Y(n)+4);
  cx.fillText('0', pad-12, bottom+4);
  cx.fillText('t', W-16, bottom+16);
  // degeneracy threshold
  cx.strokeStyle='#d33'; cx.setLineDash([4,3]); cx.beginPath();
  cx.moveTo(pad,Y(run.essThreshold)); cx.lineTo(W-10,Y(run.essThreshold)); cx.stroke();
  cx.setLineDash([]);
  cx.fillStyle='#d33'; cx.fillText('退化阈值 '+run.essThreshold.toFixed(0), pad+6, Y(run.essThreshold)-5);
  // resample ticks
  cx.strokeStyle='rgba(36,107,254,0.4)';
  run.frames.forEach(function(f,t){
    if(f.rs){ cx.beginPath(); cx.moveTo(X(t),bottom); cx.lineTo(X(t),bottom-7); cx.stroke(); }
  });
  // ESS curve
  cx.strokeStyle='#246bfe'; cx.lineWidth=1.6; cx.beginPath();
  run.frames.forEach(function(f,t){
    if(t===0) cx.moveTo(X(t),Y(f.ess)); else cx.lineTo(X(t),Y(f.ess));
  });
  cx.stroke(); cx.lineWidth=1;
  // degeneracy onset
  if(run.degenerate && run.degeneracyStep>=0){
    cx.strokeStyle='#e80'; cx.beginPath();
    cx.moveTo(X(run.degeneracyStep),top); cx.lineTo(X(run.degeneracyStep),bottom); cx.stroke();
    cx.fillStyle='#e80';
    cx.fillText('退化开始 t='+run.degeneracyStep, Math.min(X(run.degeneracyStep)+5, W-130), top+12);
  }
  // current frame cursor
  cx.strokeStyle='#1ca050'; cx.beginPath();
  cx.moveTo(X(frame),top); cx.lineTo(X(frame),bottom); cx.stroke();
}

function drawStats(){
  var run=curRun(), fr=run.frames[frame];
  var neg=0, pos=0;
  fr.parts.forEach(function(p){ if(p[0]<0) neg+=p[2]; else pos+=p[2]; });
  var deg = run.degenerate
    ? '<span class="badge warn">已退化，始于 t='+run.degeneracyStep+'</span>'
    : '<span class="badge ok">未检测到退化</span>';
  document.getElementById('stats').innerHTML =
    '当前 ESS：<b>'+fr.ess.toFixed(1)+'</b> / N='+run.nParticles+'（退化阈值 '+run.essThreshold.toFixed(0)+'）<br>'+
    '本帧是否重采样：'+(fr.rs?'是':'否')+'<br>'+
    '退化状态：'+deg+'<br>'+
    '权重质量 x&lt;0 / x&ge;0：'+(100*neg).toFixed(1)+'% / '+(100*pos).toFixed(1)+'%';
}

function drawAll(){
  drawMap(); drawChart(); drawStats();
  document.getElementById('stepLabel').textContent = 't = '+frame+' / '+(curRun().frames.length-1);
  document.getElementById('slider').value = frame;
}

function stop(){
  if(timer){ clearInterval(timer); timer=null; }
  document.getElementById('playBtn').textContent='播放';
}
function play(){
  stop();
  var fps=parseInt(document.getElementById('speed').value,10);
  timer=setInterval(function(){
    if(frame>=curRun().frames.length-1){ stop(); return; }
    frame++; drawAll();
  }, 1000/fps);
  document.getElementById('playBtn').textContent='暂停';
}

function selectScenario(i){
  stop();
  scen=i; frame=0;
  var run=curRun();
  document.getElementById('desc').textContent = run.description;
  document.getElementById('slider').max = run.frames.length-1;
  var btns=document.getElementById('tabs').children;
  for(var k=0;k<btns.length;k++){ btns[k].className = (k===i?'active':''); }
  drawAll();
}

RUNS.forEach(function(run,i){
  boundsCache.push(computeBounds(run));
  var b=document.createElement('button');
  b.textContent=run.name;
  b.onclick=function(){ selectScenario(i); };
  document.getElementById('tabs').appendChild(b);
});
document.getElementById('playBtn').onclick=function(){ if(timer) stop(); else play(); };
document.getElementById('slider').oninput=function(){ stop(); frame=parseInt(this.value,10); drawAll(); };
document.getElementById('speed').onchange=function(){ if(timer) play(); };
selectScenario(0);
</script>
</body>
</html>
`
