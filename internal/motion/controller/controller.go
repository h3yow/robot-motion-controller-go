// Package controller implements the per-cycle joint-space PID torque
// control law and local fault detection:
//
//   - FEEDBACK_LOST (recoverable): encoder invalid.
//   - FEEDBACK_TRACKING_ERROR (recoverable): |q_actual-q_cmd| > tolerance.
//   - DRIVE_SATURATION_PERSISTENT (recoverable): torque saturates for
//     SaturationCycles consecutive cycles.
//   - CONTROL_DIVERGED (unrecoverable): |error| exceeds the hard divergence
//     envelope (100x tolerance), i.e. the control loop has run away.
//   - NON_FINITE_STATE (unrecoverable): any state/torque becomes NaN/Inf.
//
// On any detected fault the controller outputs exactly zero torque (safe
// torque), so the engine can then command a controlled stop using the
// remaining valid feedback (feedback-loss stops via zero torque coasting in
// the simulation, which is the only safe option without measurements).
package controller

import (
	"fmt"
	"math"

	"github.com/h3yow/robot-motion-controller-go/internal/motion/config"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/kinematics"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/plant"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/trajectory"
)

// FaultKind distinguishes recoverable from unrecoverable faults.
type FaultKind string

const (
	FaultNone             FaultKind = ""
	FaultFeedbackLost     FaultKind = "FEEDBACK_LOST"
	FaultTrackingExceeded FaultKind = "FEEDBACK_TRACKING_ERROR"
	FaultDriveSaturated   FaultKind = "DRIVE_SATURATION_PERSISTENT"
	FaultControlDiverged  FaultKind = "CONTROL_DIVERGED"
	FaultNonFinite        FaultKind = "NON_FINITE_STATE"
)

// Recoverable reports whether motion may resume after an explicit recovery
// once the underlying condition has cleared.
//
// Feedback loss, tracking error and persistent saturation are physical,
// clearable conditions. Control divergence and non-finite state indicate
// corrupted/non-convergent computation and require hard reset
// (reconfiguration); they are intentionally not auto-cleared.
func (k FaultKind) Recoverable() bool {
	switch k {
	case FaultFeedbackLost, FaultTrackingExceeded, FaultDriveSaturated:
		return true
	default:
		return false
	}
}

// Fault describes a detected fault with the evidence that triggered it.
type Fault struct {
	Kind     FaultKind
	Axis     int
	Message  string
	Evidence map[string]float64
}

func (f *Fault) Error() string {
	if f == nil {
		return ""
	}
	return fmt.Sprintf("%s axis=%d %s", f.Kind, f.Axis, f.Message)
}

// Controller is a multi-axis PID controller with fault detection.
type Controller struct {
	cfg *config.Config
	n   int
	// per-axis PID memory
	integral []float64
	prevErr  []float64
	hasPrev  bool
	// per-axis consecutive saturated cycle count
	satCycles []int
}

// New creates a Controller from validated configuration.
func New(cfg *config.Config) *Controller {
	n := cfg.AxisCount()
	return &Controller{
		cfg:       cfg,
		n:         n,
		integral:  make([]float64, n),
		prevErr:   make([]float64, n),
		satCycles: make([]int, n),
	}
}

// Reset clears integrator, derivative history and saturation counters.
func (c *Controller) Reset() {
	for i := range c.integral {
		c.integral[i] = 0
		c.prevErr[i] = 0
		c.satCycles[i] = 0
	}
	c.hasPrev = false
}

func clamp(x, lim float64) float64 {
	if x > lim {
		return lim
	}
	if x < -lim {
		return -lim
	}
	return x
}

// Step computes torque for one control cycle from the commanded trajectory
// point and the encoder reading. Returned values:
//
//	torque    - actuator torque per axis, clamped to MaxTorque (zero on fault)
//	saturated- per-axis flag that clamping occurred this cycle
//	fault     - first detected fault, or nil; torque is safe-zero then
func (c *Controller) Step(cmd trajectory.Point, fb plant.EncoderReading) (torque []float64, saturated []bool, fault *Fault) {
	// 1) Feedback loss -------------------------------------------------
	if !fb.Valid {
		f := &Fault{Kind: FaultFeedbackLost, Axis: -1,
			Message: "encoder reading invalid (feedback lost)"}
		return make([]float64, c.n), make([]bool, c.n), f
	}
	if len(fb.Position) != c.n || len(fb.Velocity) != c.n ||
		len(cmd.Position) != c.n || len(cmd.Velocity) != c.n || len(cmd.Accel) != c.n {
		f := &Fault{Kind: FaultNonFinite, Axis: -1,
			Message: fmt.Sprintf("dimension mismatch cmd=%d/%d/%d fb=%d/%d n=%d",
				len(cmd.Position), len(cmd.Velocity), len(cmd.Accel),
				len(fb.Position), len(fb.Velocity), c.n)}
		return make([]float64, c.n), make([]bool, c.n), f
	}

	torque = make([]float64, c.n)
	saturated = make([]bool, c.n)
	dt := c.cfg.ControlPeriod
	tol := c.cfg.Controller.TrackingTolerance
	divergenceThreshold := 100 * tol

	for i, axis := range c.cfg.Axes {
		// 2) Non-finite detection --------------------------------------
		if !kinematics.IsFinite(cmd.Position[i], cmd.Velocity[i], cmd.Accel[i],
			fb.Position[i], fb.Velocity[i]) {
			return c.faultZero(FaultNonFinite, i,
				fmt.Sprintf("non-finite command or feedback: cmdP=%v fbP=%v fbV=%v",
					cmd.Position[i], fb.Position[i], fb.Velocity[i]))
		}

		// Feed-forward + PID on the tracking error.
		errP := cmd.Position[i] - fb.Position[i]

		// 3) Divergence: error outside the hard envelope. Checked before
		// the ordinary tolerance trip so a runaway is always classified as
		// the unrecoverable CONTROL_DIVERGED rather than a tracking fault.
		absErr := math.Abs(errP)
		if absErr > divergenceThreshold {
			return c.faultZero(FaultControlDiverged, i,
				fmt.Sprintf("position error %v exceeds hard divergence threshold %v (100x tolerance %v)",
					absErr, divergenceThreshold, tol))
		}

		// PID update with conditional integration anti-windup.
		ff := axis.Inertia*cmd.Accel[i] - axis.GravityTorque
		newIntegral := c.integral[i] + errP*dt
		if lim := c.cfg.Controller.IntegralWindup; lim > 0 {
			newIntegral = clamp(newIntegral, lim)
		}
		dErr := errP
		if c.hasPrev {
			dErr = (errP - c.prevErr[i]) / dt
		}
		pid := c.cfg.Controller.Kp*errP + c.cfg.Controller.Ki*newIntegral + c.cfg.Controller.Kd*(dErr)
		raw := ff + pid
		t := clamp(raw, axis.MaxTorque)
		sat := raw != t
		if !sat {
			c.integral[i] = newIntegral
		} // else: keep integral (clamped output)
		torque[i], saturated[i] = t, sat

		// 4) Persistent saturation -------------------------------------
		if sat {
			c.satCycles[i]++
			if c.satCycles[i] >= c.cfg.Controller.SaturationCycles {
				return c.faultZero(FaultDriveSaturated, i,
					fmt.Sprintf("torque saturated at %v N*m for %d consecutive cycles",
						t, c.satCycles[i]))
			}
		} else {
			c.satCycles[i] = 0
		}

		// 5) Tracking tolerance ----------------------------------------
		if absErr > tol {
			return c.faultZero(FaultTrackingExceeded, i,
				fmt.Sprintf("position error %v > tolerance %v (cmd %v vs actual %v)",
					absErr, tol, cmd.Position[i], fb.Position[i]))
		}

		c.prevErr[i] = errP
	}
	c.hasPrev = true
	return torque, saturated, nil
}

func (c *Controller) faultZero(kind FaultKind, axis int, msg string) ([]float64, []bool, *Fault) {
	f := &Fault{Kind: kind, Axis: axis, Message: msg}
	return make([]float64, c.n), make([]bool, c.n), f
}
