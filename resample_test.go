package particle

import (
	"math"
	"math/rand/v2"
	"testing"
)

func skewedWeights(n int) []float64 {
	w := make([]float64, n)
	sum := 0.0
	for i := range w {
		w[i] = math.Exp(-float64(i) / 12.0)
		sum += w[i]
	}
	for i := range w {
		w[i] /= sum
	}
	return w
}

func sumFloat(v []float64) float64 {
	s := 0.0
	for _, x := range v {
		s += x
	}
	return s
}

// Both resamplers must be unbiased: the mean selection count of index i
// over many trials must equal N*w_i.
func TestResamplersUnbiased(t *testing.T) {
	w := skewedWeights(50)
	n := len(w)
	trials := 4000
	for name, rs := range map[string]ResampleFunc{
		"multinomial": MultinomialResample,
		"lowvariance": LowVarianceResample,
	} {
		rng := rand.New(rand.NewPCG(1, 2))
		cnt := make([]float64, n)
		for tr := 0; tr < trials; tr++ {
			for _, i := range rs(w, rng) {
				cnt[i]++
			}
		}
		for i := range w {
			mean := cnt[i] / float64(trials)
			want := float64(n) * w[i]
			// Tolerance ~ 5 standard errors of the mean.
			tol := 5 * math.Sqrt(float64(n)*w[i]*(1-w[i])/float64(trials))
			if tol < 0.02 {
				tol = 0.02
			}
			if math.Abs(mean-want) > tol {
				t.Errorf("%s: index %d mean count %v, want %v (tol %v)", name, i, mean, want, tol)
			}
		}
	}
}

// The key statistical claim: on the same weight distribution, low-variance
// resampling has substantially lower selection-count variance than
// multinomial resampling. Verified empirically over repeated trials.
func TestLowVarianceResamplingHasLowerVariance(t *testing.T) {
	w := skewedWeights(200)
	trials := 2000

	rngM := rand.New(rand.NewPCG(11, 22))
	rngL := rand.New(rand.NewPCG(11, 22))
	varM := CountVariances(w, trials, MultinomialResample, rngM)
	varL := CountVariances(w, trials, LowVarianceResample, rngL)

	sumM, sumL := sumFloat(varM), sumFloat(varL)
	t.Logf("total count variance: multinomial=%.2f low-variance=%.2f ratio=%.3f",
		sumM, sumL, sumL/sumM)

	// Theory: multinomial total variance = N*(1 - sum w_i^2); low-variance
	// total variance <= N/4 and typically far less. Require low-variance to
	// be at least 2x better, with wide margin for random fluctuation.
	if sumL >= 0.5*sumM {
		t.Fatalf("low-variance total variance %.2f not < 50%% of multinomial %.2f", sumL, sumM)
	}
}

// When every N*w_i is an integer, low-variance resampling selects each index
// exactly N*w_i times: zero variance. Multinomial variance stays N*w_i*(1-w_i).
func TestLowVarianceZeroVarianceOnIntegerCounts(t *testing.T) {
	w := []float64{0.25, 0.25, 0.25, 0.25}
	trials := 5000
	rng := rand.New(rand.NewPCG(3, 4))
	vars := CountVariances(w, trials, LowVarianceResample, rng)
	for i, v := range vars {
		if v != 0 {
			t.Fatalf("low-variance: index %d count variance = %v, want 0", i, v)
		}
	}
	rng2 := rand.New(rand.NewPCG(3, 4))
	varsM := CountVariances(w, trials, MultinomialResample, rng2)
	// Multinomial variance per index should be near N*w*(1-w) = 0.75.
	for i, v := range varsM {
		if math.Abs(v-0.75) > 0.12 {
			t.Fatalf("multinomial: index %d count variance = %v, want ~0.75", i, v)
		}
	}
}

func TestResampleOutputShape(t *testing.T) {
	w := skewedWeights(30)
	rng := rand.New(rand.NewPCG(5, 6))
	for name, rs := range map[string]ResampleFunc{
		"multinomial": MultinomialResample,
		"lowvariance": LowVarianceResample,
	} {
		idx := rs(w, rng)
		if len(idx) != len(w) {
			t.Fatalf("%s: got %d indices, want %d", name, len(idx), len(w))
		}
		for _, i := range idx {
			if i < 0 || i >= len(w) {
				t.Fatalf("%s: index %d out of range", name, i)
			}
		}
	}
}
