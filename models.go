package particle

import (
	"math"
	"math/rand/v2"
)

// randn returns a standard normal sample via the Box-Muller transform.
func randn(rng *rand.Rand) float64 {
	u1 := rng.Float64()
	for u1 <= 0 {
		u1 = rng.Float64()
	}
	u2 := rng.Float64()
	return math.Sqrt(-2*math.Log(u1)) * math.Cos(2*math.Pi*u2)
}

// wrapAngle normalizes an angle to (-pi, pi].
func wrapAngle(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

// MotionModel is the sample-based velocity motion model from
// Thrun, Burgard & Fox, "Probabilistic Robotics" (sample_motion_model_velocity).
// The A parameters scale the noise applied to linear velocity (A1, A2),
// angular velocity (A3, A4) and the final rotation (A5, A6).
type MotionModel struct {
	A1, A2, A3, A4, A5, A6 float64
}

// Sample draws a successor pose for pose p under control c.
func (m MotionModel) Sample(p Pose, c Control, rng *rand.Rand) Pose {
	ev := m.A1*math.Abs(c.V) + m.A2*math.Abs(c.W)
	ew := m.A3*math.Abs(c.V) + m.A4*math.Abs(c.W)
	eg := m.A5*math.Abs(c.V) + m.A6*math.Abs(c.W)
	v := c.V + ev*randn(rng)
	w := c.W + ew*randn(rng)
	g := eg * randn(rng)
	var x, y float64
	if math.Abs(w) < 1e-9 {
		// Straight-line limit (avoids division by zero).
		x = p.X + v*c.Dt*math.Cos(p.Theta)
		y = p.Y + v*c.Dt*math.Sin(p.Theta)
	} else {
		x = p.X - v/w*math.Sin(p.Theta) + v/w*math.Sin(p.Theta+w*c.Dt)
		y = p.Y + v/w*math.Cos(p.Theta) - v/w*math.Cos(p.Theta+w*c.Dt)
	}
	return Pose{X: x, Y: y, Theta: wrapAngle(p.Theta + w*c.Dt + g*c.Dt)}
}

// SensorModel is a range-only landmark sensor with Gaussian noise.
type SensorModel struct {
	RangeSigma float64
}

// LogLikelihood returns the log-likelihood of pose p given measurements z
// (up to an additive constant, which cancels during weight normalization).
func (s SensorModel) LogLikelihood(p Pose, z []Measurement, landmarks []Landmark) float64 {
	ll := 0.0
	inv := 1.0 / (2 * s.RangeSigma * s.RangeSigma)
	for _, m := range z {
		lm := landmarks[m.Landmark]
		exp := math.Hypot(p.X-lm.X, p.Y-lm.Y)
		d := m.Range - exp
		ll -= d * d * inv
	}
	return ll
}

// Measure generates noisy range measurements to every landmark from a true pose.
func (s SensorModel) Measure(true Pose, landmarks []Landmark, rng *rand.Rand) []Measurement {
	out := make([]Measurement, len(landmarks))
	for i, lm := range landmarks {
		r := math.Hypot(true.X-lm.X, true.Y-lm.Y) + s.RangeSigma*randn(rng)
		if r < 0 {
			r = 0
		}
		out[i] = Measurement{Landmark: i, Range: r}
	}
	return out
}
