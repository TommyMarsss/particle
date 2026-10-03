package particle

import (
	"math"
	"math/rand"
	"sort"
)

// Filter 是一个二维位姿粒子滤波器。
type Filter struct {
	Rng        *rand.Rand
	Particles  []Particle
	Noise      MotionNoise
	SigmaRange float64 // 距离观测的高斯似然标准差
	// MaxRange 为传感器探测上限（≤0 无限制），仅匿名观测时用于预测可见路标数。
	MaxRange float64
	logW     []float64
}

// NewFilter 在给定矩形区域内均匀采样 n 个粒子（朝向也均匀）。
func NewFilter(rng *rand.Rand, n int, b Bounds, noise MotionNoise, sigmaRange float64) *Filter {
	f := &Filter{
		Rng:        rng,
		Particles:  make([]Particle, n),
		Noise:      noise,
		SigmaRange: sigmaRange,
		logW:       make([]float64, n),
	}
	lw := math.Log(1.0 / float64(n))
	for i := range f.Particles {
		f.Particles[i] = Particle{
			X:      b.MinX + rng.Float64()*(b.MaxX-b.MinX),
			Y:      b.MinY + rng.Float64()*(b.MaxY-b.MinY),
			Theta:  -math.Pi + rng.Float64()*(2*math.Pi),
			Weight: 1.0 / float64(n),
		}
		f.logW[i] = lw
	}
	return f
}

// N 返回粒子数量。
func (f *Filter) N() int { return len(f.Particles) }

// Estimate 返回当前加权平均位姿估计。
func (f *Filter) Estimate() Pose { return weightedPose(f.Particles) }

// Motion 应用“先旋转 Turn、再平移 Dist”的运动模型，对每个粒子注入高斯噪声。
func (f *Filter) Motion(m Move) {
	for i := range f.Particles {
		p := &f.Particles[i]
		theta := wrapAngle(p.Theta + m.Turn + f.Rng.NormFloat64()*f.Noise.SigmaTurn)
		d := m.Dist + f.Rng.NormFloat64()*f.Noise.SigmaDist
		p.X += d * math.Cos(theta)
		p.Y += d * math.Sin(theta)
		p.Theta = theta
	}
}

// Observe 根据“到已知路标的纯距离观测”更新权重（对数空间累积、最大值平移归一化）。
// ranges[k] 是到 landmarks[k] 的带噪距离；NaN 表示该路标本轮在探测范围外、被忽略。
func (f *Filter) Observe(landmarks []Point, ranges []float64) {
	maxLog := math.Inf(-1)
	for i, p := range f.Particles {
		ll := 0.0
		for k, lm := range landmarks {
			r := ranges[k]
			if math.IsNaN(r) {
				continue
			}
			d := math.Hypot(p.X-lm.X, p.Y-lm.Y)
			ll += math.Log(gaussPDF(d-r, f.SigmaRange))
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
		// 所有似然都下溢为 0：退化为均匀权重，避免产生 NaN。
		f.resetWeights()
		return
	}
	for i := range f.Particles {
		f.Particles[i].Weight /= sum
		f.logW[i] = math.Log(f.Particles[i].Weight)
	}
}

func (f *Filter) resetWeights() {
	w := 1.0 / float64(len(f.Particles))
	lw := math.Log(w)
	for i := range f.Particles {
		f.Particles[i].Weight = w
		f.logW[i] = lw
	}
}

// ESS 返回有效粒子数 N_eff = 1 / Σ w_i²（基于归一化权重）。
// 取值范围 [1, N]：N 表示权重完全均匀，1 表示全部质量集中在单个粒子上。
func (f *Filter) ESS() float64 {
	sum := 0.0
	for _, p := range f.Particles {
		sum += p.Weight * p.Weight
	}
	return 1.0 / sum
}

// commit 用重采样得到的下标集合替换粒子集，子代代权重均等。
// 子代数量可与父代不同（数量变化时 logW 缓存一并重建）。
func (f *Filter) commit(idx []int) {
	next := make([]Particle, len(idx))
	for j, i := range idx {
		next[j] = f.Particles[i]
	}
	f.Particles = next
	f.logW = make([]float64, len(idx))
	f.resetWeights()
}

// MultinomialResample 执行简单多项式重采样：
// 每个子代都独立地按归一化权重有放回地抽取父代。子代数量与父代相同。
func (f *Filter) MultinomialResample() {
	f.multinomialN(len(f.Particles))
}

// LowVarianceResample 执行低方差（系统性）重采样：
// 在 CDF 上以 1/N 等间距放置采样点，整体只抖动一次。
// 它是无偏的（每个粒子的期望子代数量仍为 N·w_i），但子代计数方差更小。
func (f *Filter) LowVarianceResample() {
	f.lowVarianceN(len(f.Particles))
}

func (f *Filter) buildCDF() []float64 {
	n := len(f.Particles)
	cdf := make([]float64, n)
	c := 0.0
	for i, p := range f.Particles {
		c += p.Weight
		cdf[i] = c
	}
	cdf[n-1] = 1.0 // 消除浮点累积误差
	return cdf
}

func (f *Filter) multinomialN(m int) {
	n := len(f.Particles)
	cdf := f.buildCDF()
	idx := make([]int, m)
	for j := range idx {
		k := sort.SearchFloat64s(cdf, f.Rng.Float64())
		if k >= n {
			k = n - 1
		}
		idx[j] = k
	}
	f.commit(idx)
}

func (f *Filter) lowVarianceN(m int) {
	n := len(f.Particles)
	cdf := f.buildCDF()
	r := f.Rng.Float64() / float64(m)
	idx := make([]int, m)
	i := 0
	for j := 0; j < m; j++ {
		u := r + float64(j)/float64(m)
		for i < n-1 && u > cdf[i] {
			i++
		}
		idx[j] = i
	}
	f.commit(idx)
}

// AdaptiveResample 仅在 ESS 低于阈值（thresholdRatio·N）时重采样，
// 以减少不必要的样本贫化；resample 为具体算法。
func (f *Filter) AdaptiveResample(thresholdRatio float64, resample func()) bool {
	if f.ESS() < thresholdRatio*float64(len(f.Particles)) {
		resample()
		return true
	}
	return false
}

// Jitter 对全部粒子施加小幅高斯扰动（重采样后“再激活”，缓解样本贫化）。
func (f *Filter) Jitter(sigmaX, sigmaY, sigmaTheta float64) {
	for i := range f.Particles {
		p := &f.Particles[i]
		p.X += f.Rng.NormFloat64() * sigmaX
		p.Y += f.Rng.NormFloat64() * sigmaY
		p.Theta = wrapAngle(p.Theta + f.Rng.NormFloat64()*sigmaTheta)
	}
}

// HypothesisShare 统计落在两个候选位姿假设半径内的粒子占比。
// 返回 (假设1占比, 假设2占比, 平衡度)，平衡度 = 1 - |f1-f2|/(f1+f2)，
// 接近 1 表示两个峰势均力敌，接近 0 表示已经塌缩到单一峰。
func (f *Filter) HypothesisShare(h1, h2 Point, radius float64) (float64, float64, float64) {
	var c1, c2 float64
	for _, p := range f.Particles {
		if math.Hypot(p.X-h1.X, p.Y-h1.Y) <= radius {
			c1 += p.Weight
		}
		if math.Hypot(p.X-h2.X, p.Y-h2.Y) <= radius {
			c2 += p.Weight
		}
	}
	tot := c1 + c2
	if tot == 0 {
		return c1, c2, 0
	}
	return c1, c2, 1 - math.Abs(c1-c2)/tot
}
