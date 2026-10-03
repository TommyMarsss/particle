package particle

import (
	"math"
	"math/rand"
	"sort"
)

// Frame 记录一个时间步的完整快照，用于回放与统计。
type Frame struct {
	Step       int
	Truth      Pose
	Estimate   Pose
	ESS        float64
	Resampled  bool
	Particles  []Particle
	Mode1Share float64 // 多峰场景：假设 1 的权重占比
	Mode2Share float64 // 多峰场景：假设 2 的权重占比
	Balance    float64 // 两峰平衡度，见 HypothesisShare
}

// World 描述一次仿真实验的静态环境与真值轨迹。
type World struct {
	Bounds    Bounds
	Landmarks []Point
	Start     Pose
	Moves     []Move
	// RangeNoise 是仿真真值观测使用的距离噪声标准差。
	RangeNoise float64
	// MaxRange 是传感器最大探测距离；≤0 表示无限制。范围外路标返回 NaN。
	MaxRange float64
}

// SenseLabeled 从真值位姿生成带标签的带噪距离观测；范围外路标的读数为 NaN。
func (w *World) SenseLabeled(rng *rand.Rand, pose Pose) []float64 {
	r := make([]float64, len(w.Landmarks))
	for k, lm := range w.Landmarks {
		d := math.Hypot(pose.X-lm.X, pose.Y-lm.Y)
		if w.MaxRange > 0 && d > w.MaxRange {
			r[k] = math.NaN()
			continue
		}
		r[k] = d + rng.NormFloat64()*w.RangeNoise
	}
	return r
}

// SenseAnonymous 生成“无标签”观测：探测范围内的带噪距离排序后作为多重集返回，
// 不携带路标身份。对称环境中，镜像位姿产生的距离多重集完全相同，
// 因而后验会（正确地）保持双峰。
func (w *World) SenseAnonymous(rng *rand.Rand, pose Pose) []float64 {
	var r []float64
	for _, lm := range w.Landmarks {
		d := math.Hypot(pose.X-lm.X, pose.Y-lm.Y)
		if w.MaxRange > 0 && d > w.MaxRange {
			continue
		}
		r = append(r, d+rng.NormFloat64()*w.RangeNoise)
	}
	sort.Float64s(r)
	return r
}

// SenseAnonymousSymmetric 与 SenseAnonymous 相同，但对“关于 y 轴镜像的路标对”
// 注入同一个噪声实现（按 (|x|, y) 分组），使噪声模型也严格镜像对称：
// 真值在 (+x0, y) 与 (-x0, y) 处产生的距离多重集逐元素相等。
// 这排除了“独立传感器噪声的偶然不对称”造成的伪漂变——后验维持双峰由此
// 只取决于滤波器本身（重采样的有限样本方差），使实验三成为干净的对照。
func (w *World) SenseAnonymousSymmetric(rng *rand.Rand, pose Pose) []float64 {
	groupNoise := map[[2]int64]float64{}
	key := func(p Point) [2]int64 {
		return [2]int64{int64(math.Round(math.Abs(p.X) * 1e6)), int64(math.Round(p.Y * 1e6))}
	}
	var r []float64
	for _, lm := range w.Landmarks {
		d := math.Hypot(pose.X-lm.X, pose.Y-lm.Y)
		if w.MaxRange > 0 && d > w.MaxRange {
			continue
		}
		k := key(lm)
		n, ok := groupNoise[k]
		if !ok {
			n = rng.NormFloat64() * w.RangeNoise
			groupNoise[k] = n
		}
		r = append(r, d+n)
	}
	sort.Float64s(r)
	return r
}

// ObserveAnonymous 用无标签距离多重集更新权重：
// 对每个粒子把预测距离排序后，与观测逐位匹配求高斯似然。
func (f *Filter) ObserveAnonymous(landmarks []Point, observed []float64) {
	pred := make([]float64, 0, len(landmarks))
	maxLog := math.Inf(-1)
	for i, p := range f.Particles {
		pred = pred[:0]
		for _, lm := range landmarks {
			d := math.Hypot(p.X-lm.X, p.Y-lm.Y)
			if f.MaxRange > 0 && d > f.MaxRange {
				continue
			}
			pred = append(pred, d)
		}
		sort.Float64s(pred)
		ll := 0.0
		m := len(pred)
		if m < len(observed) {
			m = len(observed)
		}
		for k := 0; k < m; k++ {
			var d, o float64
			if k < len(pred) {
				d = pred[k]
			}
			if k < len(observed) {
				o = observed[k]
			}
			ll += math.Log(gaussPDF(d-o, f.SigmaRange))
		}
		f.logW[i] += ll
		if f.logW[i] > maxLog {
			maxLog = f.logW[i]
		}
	}
	sum := 0.0
	for i := range f.Particles {
		w := math.Exp(f.logW[i] - maxLog)
		f.Particles[i].Weight = w
		sum += w
	}
	if sum == 0 {
		f.resetWeights()
		return
	}
	for i := range f.Particles {
		f.Particles[i].Weight /= sum
		f.logW[i] = math.Log(f.Particles[i].Weight)
	}
}

// RunDegradation 运行“纯重要性采样、全程不重采样”的全局定位实验，
// 用于暴露粒子退化：起初没有路标在探测范围内（权重均匀），随着机器人
// 逼近路簇，观测约束逐步收紧，权重集中到少数粒子，ESS 骤降。
func RunDegradation(rng *rand.Rand, n int, w World, motionNoise MotionNoise, sigmaRange float64) []Frame {
	f := NewFilter(rng, n, w.Bounds, motionNoise, sigmaRange)
	f.MaxRange = w.MaxRange
	frames := make([]Frame, 0, len(w.Moves)+1)

	truth := w.Start
	f.Observe(w.Landmarks, w.SenseLabeled(rng, truth))
	frames = append(frames, snapshot(0, truth, f, false, 0, 0))

	for step, m := range w.Moves {
		f.Motion(m)
		truth = advance(truth, m)
		f.Observe(w.Landmarks, w.SenseLabeled(rng, truth))
		frames = append(frames, snapshot(step+1, truth, f, false, 0, 0))
	}
	return frames
}

// RunMultimodal 在关于 y 轴镜像对称的环境中运行定位（无标签距离观测），
// 采用“低方差重采样 + 自适应触发 + 轻微抖动”，
// 验证滤波器能同时维持 x=±x0 两个合理位姿假设而不提前塌缩。
// jitterX/jitterY 为重采样后 x/y 方向各自的抖动标准差（各向异性：
// x 向需要较大抖动以对抗跨峰遗传漂变，y 向只需很小的抖动以跟踪轨迹）。
func RunMultimodal(rng *rand.Rand, n int, w World, motionNoise MotionNoise, sigmaRange,
	essRatio, jitterX, jitterY, jitterTheta float64, h1, h2 Point, radius float64) []Frame {

	f := NewFilter(rng, n, w.Bounds, motionNoise, sigmaRange)
	f.MaxRange = w.MaxRange
	frames := make([]Frame, 0, len(w.Moves)+1)

	take := func(step int, resampled bool, truth Pose) Frame {
		c1, c2, bal := f.HypothesisShare(h1, h2, radius)
		fr := snapshot(step, truth, f, resampled, c1, c2)
		fr.Balance = bal
		return fr
	}

	truth := w.Start
	f.ObserveAnonymous(w.Landmarks, w.SenseAnonymousSymmetric(rng, truth))
	frames = append(frames, take(0, false, truth))

	for step, m := range w.Moves {
		f.Motion(m)
		truth = advance(truth, m)
		f.ObserveAnonymous(w.Landmarks, w.SenseAnonymousSymmetric(rng, truth))
		did := f.AdaptiveResample(essRatio, f.LowVarianceResample)
		if did {
			f.Jitter(jitterX, jitterY, jitterTheta)
		}
		frames = append(frames, take(step+1, did, truth))
	}
	return frames
}

// DegradationStart 返回 ESS 首次低于 threshold 的时间步；
// 从未低于阈值时返回 -1。
func DegradationStart(frames []Frame, threshold float64) int {
	for _, fr := range frames {
		if fr.ESS < threshold {
			return fr.Step
		}
	}
	return -1
}

func advance(pose Pose, m Move) Pose {
	theta := pose.Theta + m.Turn
	return Pose{
		X:     pose.X + m.Dist*math.Cos(theta),
		Y:     pose.Y + m.Dist*math.Sin(theta),
		Theta: wrapAngle(theta),
	}
}

func snapshot(step int, truth Pose, f *Filter, resampled bool, m1, m2 float64) Frame {
	ps := make([]Particle, len(f.Particles))
	copy(ps, f.Particles)
	return Frame{
		Step:       step,
		Truth:      truth,
		Estimate:   f.Estimate(),
		ESS:        f.ESS(),
		Resampled:  resampled,
		Particles:  ps,
		Mode1Share: m1,
		Mode2Share: m2,
	}
}
