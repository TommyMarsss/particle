package particle

import (
	"math"
	"math/rand"
	"testing"
)

// TestESSKnownWeights 用已知权重向量校验有效粒子数公式 1/Σw²。
func TestESSKnownWeights(t *testing.T) {
	f := &Filter{Particles: []Particle{
		{Weight: 0.5}, {Weight: 0.25}, {Weight: 0.25},
	}}
	if got, want := f.ESS(), 1.0/(0.5*0.5+2*0.25*0.25); math.Abs(got-want) > 1e-12 {
		t.Fatalf("ESS = %v, want %v", got, want)
	}

	// 均匀权重时 ESS = N；权重集中于一个粒子时 ESS → 1。
	const n = 500
	uni := make([]Particle, n)
	for i := range uni {
		uni[i] = Particle{Weight: 1.0 / n}
	}
	if got := (&Filter{Particles: uni}).ESS(); math.Abs(got-n) > 1e-9 {
		t.Fatalf("uniform ESS = %v, want %v", got, n)
	}
	dom := make([]Particle, n)
	dom[0].Weight = 1 - 1e-12
	for i := 1; i < n; i++ {
		dom[i].Weight = 1e-12 / (n - 1)
	}
	if got := (&Filter{Particles: dom}).ESS(); got > 1.001 {
		t.Fatalf("dominated ESS = %v, want ≈1", got)
	}
}

// TestWeightNormalization 校验观测更新后权重归一化且非负。
func TestWeightNormalization(t *testing.T) {
	rng := rand.New(rand.NewSource(42))
	f := NewFilter(rng, 300, Bounds{-10, -10, 10, 10}, MotionNoise{0.05, 0.1}, 0.5)
	lm := []Point{{0, 0}, {3, 3}}
	r := []float64{4.0, 3.0}
	f.Observe(lm, r)
	sum := 0.0
	for _, p := range f.Particles {
		if p.Weight < 0 || math.IsNaN(p.Weight) {
			t.Fatalf("invalid weight %v", p.Weight)
		}
		sum += p.Weight
	}
	if math.Abs(sum-1) > 1e-9 {
		t.Fatalf("weight sum = %v, want 1", sum)
	}
}

// TestDegradationDetected 运行退化场景：ESS 应从接近 N 骤降到阈值以下，
// 且退化起始时间步可被定位。
func TestDegradationDetected(t *testing.T) {
	const n = 1000
	rng := rand.New(rand.NewSource(20260930))
	frames := RunDegradation(rng, n, DegradationWorld(),
		MotionNoise{SigmaTurn: 0.04, SigmaDist: 0.08}, 0.4)

	first, last := frames[0], frames[len(frames)-1]
	if first.ESS < 0.5*n {
		t.Fatalf("initial ESS = %.1f, expected ≥ 0.5N (filters should start healthy)", first.ESS)
	}
	threshold := 0.25 * n
	start := DegradationStart(frames, threshold)
	if start < 1 {
		t.Fatalf("degradation not detected: final ESS = %.1f, threshold = %.1f", last.ESS, threshold)
	}
	if last.ESS >= threshold {
		t.Fatalf("final ESS = %.1f, expected below %.1f", last.ESS, threshold)
	}
	// 退化应是“进行性”的：起始步之后的 ESS 整体显著低于起始步之前。
	if frames[start-1].ESS < threshold {
		t.Fatalf("degradation start step %d off: previous frame already below threshold", start)
	}
	t.Logf("ESS: %.0f -> %.0f (N=%d), degradation starts at step %d",
		first.ESS, last.ESS, n, start)
}

// skewedWeights 构造一个“一个大权重粒子 + 指数衰减长尾”的权重分布，
// 模拟退化边缘的粒子集。w0·N 不是整数，低方差重采样的子代计数
// 因此仍存在（很小的）取整方差，对比更诚实。
func skewedWeights(k int) []float64 {
	w := make([]float64, k)
	w[0] = 0.37
	tail := 0.0
	for i := 1; i < k; i++ {
		w[i] = math.Pow(0.92, float64(i))
		tail += w[i]
	}
	rem := 1 - w[0]
	for i := 1; i < k; i++ {
		w[i] *= rem / tail
	}
	return w
}

// TestResamplingVariance 用重复试验量化两种重采样的统计差异：
// 两者均值都应无偏（≈N·w），但低方差重采样的子代计数方差必须显著更小，
// 且简单重采样的经验方差应与多项式理论方差 n·w·(1-w) 吻合。
func TestResamplingVariance(t *testing.T) {
	const (
		n      = 250 // 每轮子代数量（w0·N=92.5，非整数）
		k      = 60  // 父代粒子数
		trials = 4000
	)
	w := skewedWeights(k)
	multi, low := CompareResampling(w, n, trials, 777)

	wantMean := n * w[0]
	for name, st := range map[string]ResamplingStats{"multi": multi, "low": low} {
		if math.Abs(st.DominantMean-wantMean) > 0.15*wantMean {
			t.Fatalf("%s: dominant offspring mean = %.2f, want %.2f (resampling must be unbiased)",
				name, st.DominantMean, wantMean)
		}
	}
	theory := MultinomialCountVariance(w[0], n)
	if math.Abs(multi.DominantVar-theory) > 0.1*theory {
		t.Fatalf("multinomial empirical var = %.2f, theory = %.2f", multi.DominantVar, theory)
	}
	if low.DominantVar >= multi.DominantVar {
		t.Fatalf("low-variance var (%.3f) should be smaller than multinomial var (%.3f)",
			low.DominantVar, multi.DominantVar)
	}
	ratio := low.DominantVar / multi.DominantVar
	if ratio > 0.25 {
		t.Fatalf("variance ratio = %.3f, expected low-variance to be at least 4x less variable", ratio)
	}
	if low.EstimatorVar >= multi.EstimatorVar {
		t.Fatalf("estimator variance: low(%.2e) >= multi(%.2e)", low.EstimatorVar, multi.EstimatorVar)
	}
	t.Logf("dominant offspring: mean=%.1f  multinomial var=%.2f (theory %.2f)  low-var var=%.2f  ratio=%.3f",
		wantMean, multi.DominantVar, theory, low.DominantVar, ratio)
	t.Logf("estimator variance: multinomial=%.3e  low-var=%.3e", multi.EstimatorVar, low.EstimatorVar)
}

// TestLowVarianceCoversSupport 低方差重采样必须覆盖所有高权重粒子：
// 每个 w_i ≥ 1/N 的粒子在每一轮都应至少得到一个子代。
func TestLowVarianceCoversSupport(t *testing.T) {
	rng := rand.New(rand.NewSource(9))
	w := []float64{0.3, 0.25, 0.2, 0.15, 0.1}
	for trial := 0; trial < 50; trial++ {
		counts := offspringCounts(rng, w, 100, ResampleLowVariance)
		var sum int
		for _, c := range counts {
			sum += c
		}
		if sum != 100 {
			t.Fatalf("offspring total = %d, want 100", sum)
		}
		for i := range w {
			if counts[i] == 0 {
				t.Fatalf("trial %d: particle with w=%.2f got zero offspring", trial, w[i])
			}
		}
	}
}

// TestMultimodalNoCollapse 在镜像对称环境中，自适应低方差重采样 + 抖动
// 必须全程维持两个峰，不能过早塌缩到错误的单峰。
//
// 关键工程条件：
//   - 无标签观测 + 镜像对共享同一噪声实现（SenseAnonymousSymmetric），
//     使两个镜像假设的似然严格相等，排除观测噪声偶然不对称造成的伪漂变；
//   - 较短探测距离（每步信息量受限）+ 较宽似然 σ，防止首步观测把后验钉死；
//   - 充足粒子数（N=30000）抑制有限样本的随机遗传漂变；
//   - 重采样后各向异性抖动：x 向较大以维持双峰间距，y 向较小以跟踪轨迹。
func TestMultimodalNoCollapse(t *testing.T) {
	const n = 30000
	rng := rand.New(rand.NewSource(31415))
	w := MultimodalWorld()
	w.MaxRange = 5
	h1, h2, radius := MultimodalHypotheses()
	frames := RunMultimodal(rng, n, w,
		MotionNoise{SigmaTurn: 0.03, SigmaDist: 0.05}, 1.2,
		0.5, 0.4, 0.25, 0.05, h1, h2, radius)

	last := frames[len(frames)-1]
	const minShare = 0.15
	if last.Mode1Share < minShare || last.Mode2Share < minShare {
		t.Fatalf("final mode shares = (%.3f, %.3f), want both ≥ %.2f (premature collapse)",
			last.Mode1Share, last.Mode2Share, minShare)
	}
	if last.Balance < 0.7 {
		t.Fatalf("final mode balance = %.3f, want ≥ 0.7", last.Balance)
	}
	// 两个镜像假设应近似等权：场景设计上似然完全相同。
	if d := math.Abs(last.Mode1Share - last.Mode2Share); d > 0.25 {
		t.Fatalf("mode share gap = %.3f, symmetric world should split mass ~50/50", d)
	}
	for _, fr := range frames[1:] {
		if fr.Mode1Share < 0.1 || fr.Mode2Share < 0.1 {
			t.Fatalf("step %d collapsed to one mode: shares=(%.3f,%.3f)",
				fr.Step, fr.Mode1Share, fr.Mode2Share)
		}
	}
	t.Logf("final mode shares: %.3f / %.3f, balance=%.3f, final ESS=%.0f",
		last.Mode1Share, last.Mode2Share, last.Balance, last.ESS)
}
