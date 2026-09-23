// Package controller implements the per-cycle joint-space PID control law
// and local fault detection (tracking error, feedback loss, saturation).
package controller

import (
	"github.com/h3yow/robot-motion-controller-go/internal/motion/config"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/plant"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/trajectory"
)

// FaultKind distinguishes recoverable from unrecoverable faults.
type FaultKind string

const (
	FaultNone               FaultKind = ""
	FaultFeedbackLost       FaultKind = "FEEDBACK_LOST"         // recoverable
	FaultTrackingExceeded   FaultKind = "FEEDBACK_TRACKING_ERROR" // recoverable
	FaultDriveSaturated     FaultKind = "DRIVE_SATURATION_PERSISTENT" // recoverable
	FaultControlDiverged    FaultKind = "CONTROL_DIVERGED"       // unrecoverable
	FaultNonFinite          FaultKind = "NON_FINITE_STATE"       // unrecoverable
)

// Recoverable reports whether normal motion can resume after an explicit
// recovery once the underlying condition clears.
func (k FaultKind) Recoverable() bool { return false }

// Fault describes a detected fault with the evidence that triggered it.
type Fault struct {
	Kind    FaultKind
	Axis    int
	Message string
}

func (f Fault) Error() string { return "" }

// Controller is a multi-axis PID controller.
type Controller struct {
	cfg *config.Config
}

// New creates a Controller from validated configuration.
func New(cfg *config.Config) *Controller { return &Controller{cfg: cfg} }

// Reset clears internal integrator and fault-detection state.
func (c *Controller) Reset() {}

// Step computes torque for one control cycle from the commanded trajectory
// point and encoder reading, updates internal state, and reports any fault.
// When a fault is returned the torque output is the safe (zero) torque.
func (c *Controller) Step(cmd trajectory.Point, fb plant.EncoderReading) (torque []float64, saturated []bool, fault *Fault) {
	return nil, nil, nil
}
