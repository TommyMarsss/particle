package particle

import (
	"math"
	"math/rand/v2"
)

// Filter is a sequential-importance-resampling particle filter for
// Monte Carlo Localization.
type Filter struct {
	Particles []Particle
	Motion    MotionModel
	Sensor    SensorModel
	Landmarks []Landmark
	// Resample is the resampling algorithm (e.g. MultinomialResample or
	// LowVarianceResample).
	Resample ResampleFunc
	// EssRatio triggers resampling whenever ESS < EssRatio*N.
	EssRatio float64

	rng *rand.Rand
}

// NewFilter creates a filter with n particles whose initial poses are drawn
// from init(i), with uniform weights.
func NewFilter(n int, init func(i int) Pose, motion MotionModel, sensor SensorModel,
	landmarks []Landmark, rs ResampleFunc, essRatio float64, rng *rand.Rand) *Filter {
	p := make([]Particle, n)
	for i := range p {
		p[i] = Particle{Pose: init(i), Weight: 1 / float64(n)}
	}
	return &Filter{
		Particles: p,
		Motion:    motion,
		Sensor:    sensor,
		Landmarks: landmarks,
		Resample:  rs,
		EssRatio:  essRatio,
		rng:       rng,
	}
}

// StepInfo reports what happened during one filter update.
type StepInfo struct {
	// ESS is the effective sample size after weighting, before resampling.
	ESS float64
	// Resampled is true if resampling was triggered this step.
	Resampled bool
}

// Step advances the filter by one time step: motion prediction (sampling),
// importance weighting from the observation, normalization, ESS
// computation, and resampling if the ESS is below the threshold.
func (f *Filter) Step(c Control, z []Measurement) StepInfo {
	n := len(f.Particles)

	// Predict: sample from the motion model.
	for i := range f.Particles {
		f.Particles[i].Pose = f.Motion.Sample(f.Particles[i].Pose, c, f.rng)
	}

	// Weight: accumulate log-likelihoods, then normalize via the
	// log-sum-exp trick to avoid numerical underflow.
	logw := make([]float64, n)
	max := math.Inf(-1)
	for i := range f.Particles {
		logw[i] = f.Sensor.LogLikelihood(f.Particles[i].Pose, z, f.Landmarks)
		if logw[i] > max {
			max = logw[i]
		}
	}
	sum := 0.0
	for i := range f.Particles {
		f.Particles[i].Weight = math.Exp(logw[i] - max)
		sum += f.Particles[i].Weight
	}
	if sum <= 0 {
		// Total numerical collapse: reset to uniform so the filter can
		// continue (the ESS series will still show the degeneracy).
		for i := range f.Particles {
			f.Particles[i].Weight = 1 / float64(n)
		}
	} else {
		for i := range f.Particles {
			f.Particles[i].Weight /= sum
		}
	}

	w := make([]float64, n)
	for i := range f.Particles {
		w[i] = f.Particles[i].Weight
	}
	ess := EffectiveSampleSize(w)
	info := StepInfo{ESS: ess}

	if ess < f.EssRatio*float64(n) {
		idx := f.Resample(w, f.rng)
		np := make([]Particle, n)
		for k, i := range idx {
			np[k] = Particle{Pose: f.Particles[i].Pose, Weight: 1 / float64(n)}
		}
		f.Particles = np
		info.Resampled = true
	}
	return info
}
