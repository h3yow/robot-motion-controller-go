// Package trajectory generates deterministic per-control-cycle joint
// motion commands using a time-synchronized trapezoidal profile.
package trajectory

import (
	"github.com/h3yow/robot-motion-controller-go/internal/motion/config"
)

// Point is one control-cycle sample of the commanded trajectory.
type Point struct {
	T          float64   // time from trajectory start (s)
	Position   []float64 // commanded joint position (rad)
	Velocity   []float64 // commanded joint velocity (rad/s)
	Accel      []float64 // commanded joint acceleration (rad/s^2)
}

// Trajectory is a sampled, time-synchronized multi-axis trajectory.
type Trajectory struct {
	Period float64  // control period (s)
	Points []Point  // one sample per control cycle
	Axes   int
}

// Duration returns the planned duration in seconds.
func (t *Trajectory) Duration() float64 { return 0 }

// Planner validates goals against configuration and builds trajectories.
type Planner struct {
	cfg *config.Config
}

// NewPlanner creates a Planner. The configuration must already have passed
// config.Validate.
func NewPlanner(cfg *config.Config) *Planner { return &Planner{cfg: cfg} }

// PointToPoint plans from start to goal. Any limit violation, non-finite or
// contradictory input is rejected with a distinguishable reason; no default
// value is substituted.
func (p *Planner) PointToPoint(start, goal []float64, maxVel, maxAccel []float64) (*Trajectory, error) {
	return nil, nil
}

// Path plans motion through ordered waypoints, stopping at each.
func (p *Planner) Path(start []float64, waypoints [][]float64, maxVel, maxAccel []float64) (*Trajectory, error) {
	return nil, nil
}
