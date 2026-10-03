package particle

// EffectiveSampleSize computes the effective sample size (ESS) of a
// normalized weight vector: ESS = 1 / sum(w_i^2).
//
// ESS ranges from 1 (a single particle carries all weight — fully
// degenerate) to N (uniform weights — no degeneracy).
func EffectiveSampleSize(w []float64) float64 {
	s := 0.0
	for _, wi := range w {
		s += wi * wi
	}
	if s == 0 {
		return 0
	}
	return 1 / s
}

// DegeneracyReport summarizes a degeneracy analysis of an ESS time series.
type DegeneracyReport struct {
	// Degenerate is true if the ESS stayed below Threshold for at least
	// SustainedSteps consecutive time steps.
	Degenerate bool
	// FirstStep is the first time step of the first sustained run below
	// Threshold, or -1 if the particle set never degenerated.
	FirstStep int
	// Threshold is the ESS value below which the set is considered degenerate.
	Threshold float64
	// SustainedSteps is the number of consecutive below-threshold steps
	// required to declare degeneracy.
	SustainedSteps int
}

// DefaultDegeneracyRatio is the fraction of N below which the particle set
// is considered degenerate (ESS < ratio*N).
const DefaultDegeneracyRatio = 0.3

// DefaultSustainedSteps is the number of consecutive below-threshold steps
// required to declare degeneracy. A single low-ESS step right before a
// resample is normal filter operation; only a sustained collapse — where
// resampling can no longer restore weight diversity — means a few particles
// permanently dominate and estimates become untrustworthy.
const DefaultSustainedSteps = 3

// DetectDegeneracy scans an ESS series (one value per time step, computed
// after weighting and before resampling) and reports whether — and if so,
// exactly when — the particle set degenerated. See DefaultSustainedSteps
// for the detection criterion.
func DetectDegeneracy(ess []float64, n int, ratio float64) DegeneracyReport {
	return DetectDegeneracySustained(ess, n, ratio, DefaultSustainedSteps)
}

// DetectDegeneracySustained is DetectDegeneracy with an explicit
// sustained-steps parameter k: the filter is degenerate from the first
// step that begins a run of at least k consecutive below-threshold ESS
// values.
func DetectDegeneracySustained(ess []float64, n int, ratio float64, k int) DegeneracyReport {
	rep := DegeneracyReport{
		FirstStep:      -1,
		Threshold:      ratio * float64(n),
		SustainedSteps: k,
	}
	if k < 1 {
		k = 1
	}
	run := 0
	for t, e := range ess {
		if e < rep.Threshold {
			run++
			if run >= k {
				rep.Degenerate = true
				rep.FirstStep = t - k + 1
				break
			}
		} else {
			run = 0
		}
	}
	return rep
}
