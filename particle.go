// Package particle implements Monte Carlo Localization (a particle filter)
// for a 2D robot, together with tools to detect particle degeneracy and to
// compare resampling algorithms statistically.
//
// Only the Go standard library is used.
package particle

// Pose is a 2D robot pose: position (X, Y) and heading Theta (radians).
type Pose struct {
	X, Y, Theta float64
}

// Control is a velocity command: linear velocity V, angular velocity W
// (rad/s), applied for Dt seconds.
type Control struct {
	V, W, Dt float64
}

// Landmark is a known point feature in the map.
type Landmark struct {
	X, Y float64
}

// Measurement is a range-only observation of a single landmark.
type Measurement struct {
	Landmark int     // index into the landmark list
	Range    float64 // measured distance
}

// Particle is one hypothesis about the robot pose, with an importance weight.
type Particle struct {
	Pose   Pose
	Weight float64
}

func cloneParticles(p []Particle) []Particle {
	out := make([]Particle, len(p))
	copy(out, p)
	return out
}
