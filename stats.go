package particle

import (
	"math"
	"math/rand"
)

// ResampleMethod 选择重采样算法。
type ResampleMethod string

const (
	// ResampleMultinomial 简单多项式（有放回多项式）重采样。
	ResampleMultinomial ResampleMethod = "multinomial"
	// ResampleLowVariance 低方差（系统性）重采样。
	ResampleLowVariance ResampleMethod = "low_variance"
)

// offspringCounts 执行一轮重采样并返回每个父代粒子产生的子代数量。
// 父代粒子以其下标作为 X 坐标以便事后辨认；m 为子代总数。
func offspringCounts(rng *rand.Rand, weights []float64, m int, method ResampleMethod) []int {
	f := &Filter{
		Rng:       rng,
		Particles: make([]Particle, len(weights)),
		logW:      make([]float64, len(weights)),
	}
	for i, w := range weights {
		f.Particles[i] = Particle{X: float64(i), Weight: w}
		f.logW[i] = math.Log(w)
	}
	switch method {
	case ResampleLowVariance:
		f.lowVarianceN(m)
	default:
		f.multinomialN(m)
	}
	counts := make([]int, len(weights))
	for _, p := range f.Particles {
		counts[int(math.Round(p.X))]++
	}
	return counts
}

// ResamplingStats 是对同一权重分布重复执行重采样得到的蒙特卡洛统计量。
type ResamplingStats struct {
	Trials int
	// DominantMean / DominantVar：权重最大粒子的子代数量的均值与样本方差。
	DominantMean float64
	DominantVar  float64
	// EstimatorMean / EstimatorVar：估计量 (1/N)·Σ_j w_{父代(j)} 的均值与样本方差。
	// 它度量“重采样这一步本身”给基于粒子的估计引入的额外波动。
	EstimatorMean float64
	EstimatorVar  float64
	// CountVarMean：所有父代粒子子代计数方差（跨试验）的平均值。
	CountVarMean float64
	// Histogram：最大权重粒子子代数量的频次分布 [次数]，直方图边界为 [0..N]。
	Histogram []int
}

// CompareResampling 用不同随机种子重复 trials 轮重采样，
// 分别统计简单多项式重采样与低方差重采样的子代计数波动。
// n 为每轮抽取的子代总数（通常等于粒子数）。
func CompareResampling(weights []float64, n int, trials int, seed int64) (ResamplingStats, ResamplingStats) {
	return resamplingStats(weights, n, trials, seed, ResampleMultinomial),
		resamplingStats(weights, n, trials, seed, ResampleLowVariance)
}

func resamplingStats(weights []float64, n, trials int, seed int64, method ResampleMethod) ResamplingStats {
	dom := make([]float64, trials)
	est := make([]float64, trials)
	countAcc := make([][]float64, len(weights))
	for i := range countAcc {
		countAcc[i] = make([]float64, trials)
	}
	hist := make([]int, n+1)

	for t := 0; t < trials; t++ {
		rng := rand.New(rand.NewSource(seed + int64(t)*2654435761))
		counts := offspringCounts(rng, weights, n, method)
		e := 0.0
		for i, c := range counts {
			countAcc[i][t] = float64(c)
			e += float64(c) * weights[i]
		}
		dom[t] = float64(counts[0])
		est[t] = e / float64(n)
		hist[counts[0]]++
	}

	st := ResamplingStats{
		Trials:        trials,
		DominantMean:  Mean(dom),
		DominantVar:   SampleVariance(dom),
		EstimatorMean: Mean(est),
		EstimatorVar:  SampleVariance(est),
		Histogram:     hist,
	}
	vSum := 0.0
	for _, series := range countAcc {
		vSum += SampleVariance(series)
	}
	st.CountVarMean = vSum / float64(len(weights))
	return st
}

// Mean 返回算术平均值。
func Mean(xs []float64) float64 {
	s := 0.0
	for _, x := range xs {
		s += x
	}
	return s / float64(len(xs))
}

// SampleVariance 返回无偏样本方差（除以 len-1）。
func SampleVariance(xs []float64) float64 {
	if len(xs) < 2 {
		return 0
	}
	m := Mean(xs)
	s := 0.0
	for _, x := range xs {
		d := x - m
		s += d * d
	}
	return s / float64(len(xs)-1)
}

// MultinomialCountVariance 返回多项式重采样下某个权重为 w 的父代、
// 抽取 n 次时子代计数的理论方差 n·w·(1-w)。
func MultinomialCountVariance(w float64, n int) float64 {
	return float64(n) * w * (1 - w)
}

// MultinomialEstimatorVariance 返回多项式重采样下估计量
// (1/N)Σ_j w_{父代(j)} 的理论方差 (Σw³-(Σw²)²)/N。
func MultinomialEstimatorVariance(weights []float64, n int) float64 {
	var s2, s3 float64
	for _, w := range weights {
		s2 += w * w
		s3 += w * w * w
	}
	return (s3 - s2*s2) / float64(n)
}
