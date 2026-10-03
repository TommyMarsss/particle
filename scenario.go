package particle

import (
	"math"
	"math/rand/v2"
)

// Frame is a snapshot of the filter state at one time step.
type Frame struct {
	Particles []Particle
	True      Pose
	// ESS after weighting, before resampling (for t=0 it is N).
	ESS       float64
	Resampled bool
}

// RunResult holds everything recorded during one simulated localization run.
type RunResult struct {
	Name         string
	Description  string
	Landmarks    []Landmark
	Frames       []Frame
	NumParticles int
	// EssRatio is the resampling trigger (resample when ESS < EssRatio*N).
	EssRatio float64
	// Degeneracy is the result of scanning the ESS series for degeneracy.
	Degeneracy DegeneracyReport
}

// EssSeries returns the ESS value of every frame.
func (r *RunResult) EssSeries() []float64 {
	out := make([]float64, len(r.Frames))
	for i, f := range r.Frames {
		out[i] = f.ESS
	}
	return out
}

// ClusterMassX splits the particle set at x=0 and returns the total weight
// on each side. Used to verify that a bimodal belief keeps both modes.
func ClusterMassX(p []Particle) (neg, pos float64) {
	for _, pt := range p {
		if pt.Pose.X < 0 {
			neg += pt.Weight
		} else {
			pos += pt.Weight
		}
	}
	return neg, pos
}

// runScenario simulates ground truth (with the "true" models), generates
// observations, runs the filter (with the "filter" models, which may
// deliberately mismatch), and records every frame.
func runScenario(name, description string,
	landmarks []Landmark, start Pose, controls []Control,
	trueMotion, filtMotion MotionModel,
	trueSensor, filtSensor SensorModel,
	init func(i int) Pose,
	n int, rs ResampleFunc, essRatio float64, seed uint64) *RunResult {

	rng := rand.New(rand.NewPCG(seed, seed^0x9e3779b97f4a7c15))

	// Ground-truth trajectory and observations.
	truePath := make([]Pose, len(controls)+1)
	truePath[0] = start
	obs := make([][]Measurement, len(controls))
	p := start
	for t, c := range controls {
		p = trueMotion.Sample(p, c, rng)
		truePath[t+1] = p
		obs[t] = trueSensor.Measure(p, landmarks, rng)
	}

	f := NewFilter(n, init, filtMotion, filtSensor, landmarks, rs, essRatio, rng)

	frames := make([]Frame, 0, len(controls)+1)
	frames = append(frames, Frame{
		Particles: cloneParticles(f.Particles),
		True:      truePath[0],
		ESS:       float64(n),
	})
	for t, c := range controls {
		info := f.Step(c, obs[t])
		frames = append(frames, Frame{
			Particles: cloneParticles(f.Particles),
			True:      truePath[t+1],
			ESS:       info.ESS,
			Resampled: info.Resampled,
		})
	}

	res := &RunResult{
		Name:         name,
		Description:  description,
		Landmarks:    landmarks,
		Frames:       frames,
		NumParticles: n,
		EssRatio:     essRatio,
	}
	res.Degeneracy = DetectDegeneracy(res.EssSeries(), n, DefaultDegeneracyRatio)
	return res
}

// ringLandmarks returns m landmarks evenly spaced on a circle.
func ringLandmarks(m int, radius float64) []Landmark {
	out := make([]Landmark, m)
	for i := range out {
		a := 2 * math.Pi * float64(i) / float64(m)
		out[i] = Landmark{X: radius * math.Cos(a), Y: radius * math.Sin(a)}
	}
	return out
}

// circleControls returns controls that drive the robot along a circle.
func circleControls(steps int, v, w float64) []Control {
	out := make([]Control, steps)
	for i := range out {
		out[i] = Control{V: v, W: w, Dt: 1}
	}
	return out
}

func matchedMotion() MotionModel {
	return MotionModel{A1: 0.03, A2: 0.03, A3: 0.03, A4: 0.03, A5: 0.01, A6: 0.01}
}

// NormalScenario is a well-specified tracking run: the filter's motion and
// sensor models match the true ones, so the ESS stays healthy and no
// degeneracy should be detected.
func NormalScenario(seed uint64) *RunResult {
	landmarks := ringLandmarks(8, 8)
	start := Pose{X: 3.8, Y: 0, Theta: math.Pi / 2}
	controls := circleControls(120, 0.5, 0.13)
	motion := matchedMotion()
	sensor := SensorModel{RangeSigma: 0.4}
	n := 500
	rng := rand.New(rand.NewPCG(seed^0xabcdef, 7))
	init := func(i int) Pose {
		return Pose{
			X:     start.X + 0.2*randn(rng),
			Y:     start.Y + 0.2*randn(rng),
			Theta: wrapAngle(start.Theta + 0.1*randn(rng)),
		}
	}
	return runScenario(
		"正常跟踪",
		"滤波器的运动模型与观测模型和真实模型一致。粒子集应始终保持健康，ESS 远高于退化阈值，不应检测到退化。",
		landmarks, start, controls, motion, motion, sensor, sensor,
		init, n, LowVarianceResample, 0.5, seed,
	)
}

// DegeneracyScenario uses a deliberately overconfident sensor model in the
// filter (sigma 0.03 while the true noise is 0.3). Almost all particles get
// negligible likelihood each step, so the weights collapse onto a handful of
// particles and the ESS crashes — a textbook particle degeneracy.
func DegeneracyScenario(seed uint64) *RunResult {
	landmarks := ringLandmarks(8, 8)
	start := Pose{X: 3.8, Y: 0, Theta: math.Pi / 2}
	controls := circleControls(80, 0.5, 0.13)
	motion := matchedMotion()
	trueSensor := SensorModel{RangeSigma: 0.3}
	filtSensor := SensorModel{RangeSigma: 0.03} // overconfident on purpose
	n := 500
	rng := rand.New(rand.NewPCG(seed^0x12345, 7))
	init := func(i int) Pose {
		return Pose{
			X:     start.X + 0.4*randn(rng),
			Y:     start.Y + 0.4*randn(rng),
			Theta: wrapAngle(start.Theta + 0.15*randn(rng)),
		}
	}
	return runScenario(
		"粒子退化",
		"滤波器的观测模型过度自信（sigma=0.03，真实噪声 sigma=0.3）。每次更新后权重迅速集中到极少数粒子上，ESS 跌破阈值，应能检测到退化及其起始时间步。",
		landmarks, start, controls, motion, motion, trueSensor, filtSensor,
		init, n, LowVarianceResample, 0.5, seed,
	)
}

// MultimodalScenario places the robot in a symmetric environment: the two
// landmarks sit on the y-axis, so ranges measured at (x, y) and at (-x, y)
// are identical. Two pose hypotheses (the true one and its mirror) are both
// fully consistent with every observation. A healthy particle set must keep
// both modes instead of collapsing prematurely onto a single one.
func MultimodalScenario(seed uint64) *RunResult {
	landmarks := []Landmark{{X: 0, Y: 8}, {X: 0, Y: -8}}
	start := Pose{X: 6, Y: 1, Theta: 0}
	controls := make([]Control, 40)
	for i := range controls {
		controls[i] = Control{V: 0.3, W: 0, Dt: 1}
	}
	motion := MotionModel{A1: 0.02, A2: 0.02, A3: 0.02, A4: 0.02, A5: 0.005, A6: 0.005}
	sensor := SensorModel{RangeSigma: 0.5}
	n := 800
	rng := rand.New(rand.NewPCG(seed^0x777, 13))
	init := func(i int) Pose {
		if i < n/2 {
			// True hypothesis.
			return Pose{X: 6 + 0.3*randn(rng), Y: 1 + 0.3*randn(rng), Theta: wrapAngle(0.1 * randn(rng))}
		}
		// Mirror hypothesis: (-x, y, pi-theta) is observationally equivalent.
		return Pose{X: -6 + 0.3*randn(rng), Y: 1 + 0.3*randn(rng), Theta: wrapAngle(math.Pi + 0.1*randn(rng))}
	}
	return runScenario(
		"多峰分布",
		"对称环境：两个地标都在 y 轴上，(x,y) 与 (-x,y) 产生完全相同的距离观测，因此存在两个同样合理的位姿假设。粒子集应长期维持两个峰，而不是过早塌缩到单一峰。",
		landmarks, start, controls, motion, motion, sensor, sensor,
		init, n, LowVarianceResample, 0.4, seed,
	)
}
