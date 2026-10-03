package particle

import (
	"math"
	"math/rand/v2"
	"testing"
)

func TestMotionModelStraightLineNoNoise(t *testing.T) {
	m := MotionModel{} // all noise parameters zero
	rng := rand.New(rand.NewPCG(1, 1))
	p := m.Sample(Pose{0, 0, 0}, Control{V: 1, W: 0, Dt: 2}, rng)
	if !almostEq(p.X, 2, 1e-9) || !almostEq(p.Y, 0, 1e-9) || !almostEq(p.Theta, 0, 1e-9) {
		t.Fatalf("straight-line motion: got %+v, want (2,0,0)", p)
	}
}

func TestMotionModelArcNoNoise(t *testing.T) {
	m := MotionModel{}
	rng := rand.New(rand.NewPCG(1, 1))
	// v=1, w=pi/2, dt=1: quarter circle of radius v/w = 2/pi.
	p := m.Sample(Pose{0, 0, 0}, Control{V: 1, W: math.Pi / 2, Dt: 1}, rng)
	r := 2 / math.Pi
	if !almostEq(p.X, r, 1e-9) || !almostEq(p.Y, r, 1e-9) || !almostEq(p.Theta, math.Pi/2, 1e-9) {
		t.Fatalf("arc motion: got %+v, want (%v,%v,pi/2)", p, r, r)
	}
}

// The degeneracy scenario (overconfident sensor model) must be flagged as
// degenerate, with the onset located at a specific early time step whose
// ESS is indeed below the threshold.
func TestDegeneracyScenarioDetected(t *testing.T) {
	res := DegeneracyScenario(42)
	deg := res.Degeneracy
	if !deg.Degenerate {
		t.Fatal("degeneracy scenario was not detected as degenerate")
	}
	if deg.FirstStep < 1 || deg.FirstStep > 40 {
		t.Fatalf("degeneracy onset at t=%d, expected an early step in [1,40]", deg.FirstStep)
	}
	ess := res.EssSeries()
	if ess[deg.FirstStep] >= deg.Threshold {
		t.Fatalf("ESS at onset %.1f not below threshold %.1f", ess[deg.FirstStep], deg.Threshold)
	}
	// Before the onset the series must have been healthy.
	for ti := 0; ti < deg.FirstStep; ti++ {
		if ess[ti] < deg.Threshold {
			t.Fatalf("ESS at t=%d (%.1f) below threshold before reported onset %d", ti, ess[ti], deg.FirstStep)
		}
	}
	t.Logf("degeneracy onset at t=%d (ESS=%.1f, threshold=%.1f)",
		deg.FirstStep, ess[deg.FirstStep], deg.Threshold)
}

// The well-specified scenario must stay healthy: no degeneracy.
func TestNormalScenarioNotDegenerate(t *testing.T) {
	res := NormalScenario(42)
	if res.Degeneracy.Degenerate {
		t.Fatalf("normal scenario unexpectedly degenerate at t=%d", res.Degeneracy.FirstStep)
	}
}

// In the symmetric environment the particle set must keep both pose
// hypotheses (both modes) for the whole run instead of collapsing early
// onto a single peak.
func TestMultimodalScenarioKeepsBothModes(t *testing.T) {
	res := MultimodalScenario(42)
	const minMass = 0.15
	// Check the second half of the run: both modes must retain significant
	// weight mass at every checked frame.
	for ti := len(res.Frames) / 2; ti < len(res.Frames); ti += 5 {
		neg, pos := ClusterMassX(res.Frames[ti].Particles)
		if neg < minMass || pos < minMass {
			t.Fatalf("t=%d: belief collapsed to a single mode (x<0: %.2f, x>=0: %.2f)",
				ti, neg, pos)
		}
	}
	last := res.Frames[len(res.Frames)-1]
	neg, pos := ClusterMassX(last.Particles)
	t.Logf("final frame mass: x<0 %.1f%%, x>=0 %.1f%%", 100*neg, 100*pos)
}

// Sanity check: in the normal scenario the filter should actually track the
// robot — the weighted mean pose should stay near the true pose.
func TestNormalScenarioTracksTruth(t *testing.T) {
	res := NormalScenario(42)
	last := res.Frames[len(res.Frames)-1]
	var mx, my float64
	for _, p := range last.Particles {
		mx += p.Pose.X * p.Weight
		my += p.Pose.Y * p.Weight
	}
	d := math.Hypot(mx-last.True.X, my-last.True.Y)
	if d > 1.0 {
		t.Fatalf("weighted mean pose %.2f away from true pose, want < 1.0", d)
	}
}
