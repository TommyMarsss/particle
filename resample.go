package particle

import "math/rand/v2"

// ResampleFunc maps a normalized weight vector to N selected ancestor
// indices (with repetition). The expected number of times index i is
// selected must equal N*w_i for the resampling to be unbiased.
type ResampleFunc func(w []float64, rng *rand.Rand) []int

// MultinomialResample draws N i.i.d. indices from the categorical
// distribution defined by w. It is unbiased but has the highest variance
// of the common resampling schemes: the count of index i is binomial with
// variance N*w_i*(1-w_i).
func MultinomialResample(w []float64, rng *rand.Rand) []int {
	n := len(w)
	cdf := make([]float64, n)
	c := 0.0
	for i, wi := range w {
		c += wi
		cdf[i] = c
	}
	out := make([]int, n)
	for k := 0; k < n; k++ {
		// Multiply by the total to be robust to weights that do not sum
		// to exactly 1 due to floating-point rounding.
		u := rng.Float64() * cdf[n-1]
		lo, hi := 0, n-1
		for lo < hi {
			mid := (lo + hi) / 2
			if cdf[mid] < u {
				lo = mid + 1
			} else {
				hi = mid
			}
		}
		out[k] = lo
	}
	return out
}

// LowVarianceResample implements low-variance (a.k.a. systematic)
// resampling: a single random offset r in [0, 1/N) is drawn and the
// cumulative weight distribution is sampled at the evenly spaced points
// r, r+1/N, ..., r+(N-1)/N.
//
// It is unbiased like multinomial resampling, but the number of times
// index i is selected is either floor(N*w_i) or ceil(N*w_i), so its
// count variance is at most 1/4 — and exactly 0 whenever N*w_i is an
// integer — instead of N*w_i*(1-w_i).
func LowVarianceResample(w []float64, rng *rand.Rand) []int {
	n := len(w)
	out := make([]int, n)
	step := 1.0 / float64(n)
	r := rng.Float64() * step
	c := w[0]
	i := 0
	for m := 0; m < n; m++ {
		u := r + float64(m)*step
		for u > c && i < n-1 {
			i++
			c += w[i]
		}
		out[m] = i
	}
	return out
}

// CountVariances empirically estimates, per index, the variance of the
// number of times that index is selected by resampler rs over `trials`
// independent runs on the same weight vector. This is the quantity used to
// compare resampling algorithms statistically.
func CountVariances(w []float64, trials int, rs ResampleFunc, rng *rand.Rand) []float64 {
	n := len(w)
	sum := make([]float64, n)
	sq := make([]float64, n)
	cnt := make([]float64, n)
	for t := 0; t < trials; t++ {
		idx := rs(w, rng)
		for i := range cnt {
			cnt[i] = 0
		}
		for _, i := range idx {
			cnt[i]++
		}
		for i := 0; i < n; i++ {
			sum[i] += cnt[i]
			sq[i] += cnt[i] * cnt[i]
		}
	}
	vars := make([]float64, n)
	for i := 0; i < n; i++ {
		mean := sum[i] / float64(trials)
		vars[i] = sq[i]/float64(trials) - mean*mean
	}
	return vars
}
