package particle

import (
	"math"
	"testing"
)

func almostEq(a, b, tol float64) bool { return math.Abs(a-b) <= tol }

func TestEffectiveSampleSizeUniform(t *testing.T) {
	n := 100
	w := make([]float64, n)
	for i := range w {
		w[i] = 1.0 / float64(n)
	}
	if got := EffectiveSampleSize(w); !almostEq(got, float64(n), 1e-9) {
		t.Fatalf("uniform weights: ESS = %v, want %v", got, n)
	}
}

func TestEffectiveSampleSizeSingleDominant(t *testing.T) {
	w := []float64{1, 0, 0, 0, 0}
	if got := EffectiveSampleSize(w); !almostEq(got, 1, 1e-12) {
		t.Fatalf("single dominant weight: ESS = %v, want 1", got)
	}
}

func TestEffectiveSampleSizeKnownValue(t *testing.T) {
	// ESS = 1 / (0.5^2 + 0.25^2 + 0.25^2) = 1 / 0.375
	w := []float64{0.5, 0.25, 0.25}
	want := 1.0 / 0.375
	if got := EffectiveSampleSize(w); !almostEq(got, want, 1e-12) {
		t.Fatalf("ESS = %v, want %v", got, want)
	}
}

func TestDetectDegeneracy(t *testing.T) {
	n := 100
	// Threshold is 0.3*100 = 30; first crossing at t=2.
	ess := []float64{100, 80, 29.9, 10, 5}
	rep := DetectDegeneracy(ess, n, DefaultDegeneracyRatio)
	if !rep.Degenerate {
		t.Fatal("expected degeneracy to be detected")
	}
	if rep.FirstStep != 2 {
		t.Fatalf("FirstStep = %d, want 2", rep.FirstStep)
	}
	if !almostEq(rep.Threshold, 30, 1e-9) {
		t.Fatalf("Threshold = %v, want 30", rep.Threshold)
	}
}

func TestDetectDegeneracyHealthy(t *testing.T) {
	ess := []float64{100, 90, 95, 80, 85}
	rep := DetectDegeneracy(ess, 100, DefaultDegeneracyRatio)
	if rep.Degenerate {
		t.Fatalf("unexpected degeneracy, FirstStep=%d", rep.FirstStep)
	}
	if rep.FirstStep != -1 {
		t.Fatalf("FirstStep = %d, want -1", rep.FirstStep)
	}
}

func TestDetectDegeneracyIgnoresTransientDip(t *testing.T) {
	// Isolated below-threshold dips (threshold = 30) shorter than
	// DefaultSustainedSteps must not be flagged as degeneracy.
	ess := []float64{100, 20, 90, 10, 25, 95, 5, 90}
	rep := DetectDegeneracy(ess, 100, DefaultDegeneracyRatio)
	if rep.Degenerate {
		t.Fatalf("transient dips wrongly flagged as degeneracy at t=%d", rep.FirstStep)
	}
	// But a sustained run of 3 is detected, starting at its first step.
	ess2 := []float64{100, 20, 90, 10, 25, 15, 90}
	rep2 := DetectDegeneracy(ess2, 100, DefaultDegeneracyRatio)
	if !rep2.Degenerate || rep2.FirstStep != 3 {
		t.Fatalf("sustained degeneracy: got degenerate=%v FirstStep=%d, want true/3",
			rep2.Degenerate, rep2.FirstStep)
	}
}
