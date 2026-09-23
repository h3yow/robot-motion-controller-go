// Package plant is the local simulated actuator and encoder: a per-axis
// double integrator with deterministic fixed-step integration.
package plant

// EncoderReading is one control cycle's encoder feedback.
type EncoderReading struct {
	Position []float64 // measured joint position (rad)
	Velocity []float64 // measured joint velocity (rad/s)
	// Valid is false to simulate feedback loss; in that case Position and
	// Velocity must not be trusted.
	Valid bool
}

// Sim is the simulated multi-axis plant.
type Sim struct {
	period    float64
	n         int
	position  []float64
	velocity  []float64
	inertia   []float64
	gravity   []float64
	feedbackOK bool
}

// New creates a simulator for n axes with the given fixed period (s),
// inertia (kg*m^2) and gravity torque bias (N*m).
func New(period float64, inertia, gravity []float64) *Sim {
	return &Sim{}
}

// Reset places the plant at the given joint state.
func (s *Sim) Reset(position, velocity []float64) {}

// Step integrates one control cycle under the applied torques (N*m) and an
// additive deterministic disturbance torque (N*m, use 0 for none).
func (s *Sim) Step(torque, disturbance []float64) EncoderReading {
	return EncoderReading{}
}

// SetFeedbackValid simulates encoder connection/disconnection.
func (s *Sim) SetFeedbackValid(ok bool) {}

// Position returns a copy of the simulated position.
func (s *Sim) Position() []float64 { return nil }

// Velocity returns a copy of the simulated velocity.
func (s *Sim) Velocity() []float64 { return nil }
