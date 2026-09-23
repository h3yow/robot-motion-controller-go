// Package plant is the local simulated actuator and encoder: a per-axis
// double integrator (inertia load + optional constant gravity bias) with
// deterministic semi-implicit Euler integration on a fixed time step.
//
// The plant itself never saturates torque or hides faults; commanded torque
// saturation and fault detection live in the controller/engine. An encoder
// can be marked invalid to deterministically simulate feedback loss.
package plant

import "github.com/h3yow/robot-motion-controller-go/internal/motion/kinematics"

// EncoderReading is one control cycle's encoder feedback.
type EncoderReading struct {
	Position []float64 // measured joint position (rad)
	Velocity []float64 // measured joint velocity (rad/s)
	// Valid is false to simulate feedback loss; Position/Velocity are then
	// nil and must not be trusted.
	Valid bool
}

// Sim is the simulated multi-axis plant.
type Sim struct {
	period     float64
	n          int
	position   []float64
	velocity   []float64
	inertia    []float64
	gravity    []float64
	feedbackOK bool
}

// New creates a simulator. period is the fixed step (s), inertia is per axis
// (kg*m^2, strictly positive), gravity an optional constant torque bias
// (N*m per axis, nil means zero).
func New(period float64, inertia, gravity []float64) *Sim {
	n := len(inertia)
	s := &Sim{
		period:     period,
		n:          n,
		position:   make([]float64, n),
		velocity:   make([]float64, n),
		inertia:    append([]float64(nil), inertia...),
		gravity:    make([]float64, n),
		feedbackOK: true,
	}
	if gravity != nil {
		copy(s.gravity, gravity)
	}
	return s
}

// Reset places the plant at the given joint state.
func (s *Sim) Reset(position, velocity []float64) {
	copy(s.position, position)
	copy(s.velocity, velocity)
	s.feedbackOK = true
}

// Read returns the current encoder reading WITHOUT changing state. It is
// used to implement the correct zero-order-hold order read -> compute torque
// -> Apply.
func (s *Sim) Read() EncoderReading {
	if !s.feedbackOK {
		return EncoderReading{Valid: false}
	}
	return EncoderReading{
		Valid:    true,
		Position: append([]float64(nil), s.position...),
		Velocity: append([]float64(nil), s.velocity...),
	}
}

// Apply integrates one fixed control cycle under the torque (N*m) computed
// for THIS cycle, plus an optional deterministic disturbance (N*m, nil is
// zero). Semi-implicit Euler (velocity then position), deterministic.
func (s *Sim) Apply(torque, disturbance []float64) {
	d := make([]float64, s.n)
	if disturbance != nil {
		copy(d, disturbance)
	}
	for i := 0; i < s.n; i++ {
		var tau, dist float64
		if i < len(torque) {
			tau = torque[i]
		}
		if i < len(d) {
			dist = d[i]
		}
		net := tau + s.gravity[i] + dist
		acc := net / s.inertia[i]
		s.velocity[i] += acc * s.period
		s.position[i] += s.velocity[i] * s.period
	}
}

// Step is retained for simple callers/tests: it applies the given torque for
// one cycle and returns the reading taken AFTER the update (one-cycle delay
// semantics). Closed-loop control should use Read + Apply instead.
func (s *Sim) Step(torque, disturbance []float64) EncoderReading {
	s.Apply(torque, disturbance)
	return s.Read()
}

// SetFeedbackValid simulates encoder connection (true) / loss (false).
func (s *Sim) SetFeedbackValid(ok bool) { s.feedbackOK = ok }

// FeedbackValid reports current encoder link state.
func (s *Sim) FeedbackValid() bool { return s.feedbackOK }

// Position returns a copy of the simulated position.
func (s *Sim) Position() []float64 { return append([]float64(nil), s.position...) }

// Velocity returns a copy of the simulated velocity.
func (s *Sim) Velocity() []float64 { return append([]float64(nil), s.velocity...) }

// checkFinite is used by tests/diagnostics.
func checkFinite(xs []float64) bool { return kinematics.IsFinite(xs...) }
