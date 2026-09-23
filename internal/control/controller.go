// Package control implements the per-cycle closed-loop joint controller:
// feed-forward + PID tracking, convergence detection, safe stopping and an
// explicit recoverable/fatal fault state machine.
//
// Cycle law (per axis i, period dt fixed):
//
//	e      = q_target - q_measured
//	integral_+= clamp(ki*e*dt, -Ilim, +Ilim)
//	tau    = tau_ff + kp*e + integral + kd*(v_target - v_measured)
//	sat    = |tau| > tau_max  (then command is clamped)
//
// Phases: idle -> running -> (stopping | faulted | done).
// Faults:
//
//   - feedback dropout         -> faulted/recoverable; persists beyond
//     DropoutMaxTicks -> fatal;
//   - |e| > MaxPositionError   -> faulted/recoverable; |e| > DivergeError
//     -> fatal;
//   - sustained saturation     -> faulted/recoverable (drive cannot keep up);
//   - non-finite feedback/torque -> fatal immediately.
//
// While a fault is RECOVERABLE the controller applies a safe hold (P/D park
// at the current reference); a FATAL fault removes torque entirely.
//
// Recovery is permitted only with feedback healthy, |e| within half the
// position threshold, speed below the velocity tolerance, and the condition
// held for RecoveryHoldTicks.
package control

import (
	"fmt"
	"sync"
	"time"

	"github.com/h3yow/robot-motion-controller-go/internal/config"
	"github.com/h3yow/robot-motion-controller-go/internal/mcerr"
	"github.com/h3yow/robot-motion-controller-go/internal/numeric"
	"github.com/h3yow/robot-motion-controller-go/internal/trajectory"
)

// Phase is the controller lifecycle state.
type Phase string

const (
	PhaseIdle     Phase = "idle"
	PhaseRunning  Phase = "running"
	PhaseStopping Phase = "stopping"
	PhaseFaulted  Phase = "faulted"
	PhaseDone     Phase = "done"
)

// FaultCode classifies controller faults.
type FaultCode string

const (
	FaultFeedbackLost     FaultCode = "feedback_lost"      // recoverable dropout
	FaultFeedbackDead     FaultCode = "feedback_dead"      // fatal: dropout persisted
	FaultTracking         FaultCode = "tracking_error"     // recoverable threshold
	FaultTrackingDiverged FaultCode = "tracking_diverged"  // fatal
	FaultDriveSaturated   FaultCode = "drive_saturated"    // recoverable
	FaultCommandNonFinite FaultCode = "command_non_finite" // fatal
	FaultNotConverged     FaultCode = "not_converged"      // fatal at end of move
)

// Fault is a structured controller fault.
type Fault struct {
	Code        FaultCode
	Axis        int
	Tick        int64
	Recoverable bool
	Message     string
}

// AxisStatus is a point-in-time snapshot of one axis.
type AxisStatus struct {
	TargetPosition float64
	TargetVelocity float64
	Position       float64
	Velocity       float64
	Error          float64
	TorqueCommand  float64
	Saturated      bool
	FeedbackOK     bool
}

type axisState struct {
	q, v          float64 // measured
	integral      float64
	err           float64
	targetPos     float64
	targetVel     float64
	tauCmd        float64
	saturated     bool
	feedbackOK    bool
	dropoutRun    int // consecutive dropouts
	saturationRun int // consecutive saturated cycles
}

// Controller runs fixed-period control cycles.
type Controller struct {
	mu     sync.Mutex
	cfg    *config.Config
	n      int
	dt     float64
	traj   *trajectory.Trajectory
	idx    int
	phase  Phase
	fault  *Fault
	states []axisState
	ticks  int64

	// safe-stop ramp state
	stopV       []float64 // current reference velocity while stopping
	stopQ       []float64 // reference position while stopping
	stopSettled bool
	endTicks    int // cycles past trajectory end
	holdTicks   int // convergence hold counter
	recTicks    int // recovery precondition hold counter
}

// New constructs a controller from validated configuration.
func New(cfg *config.Config) *Controller {
	n := len(cfg.Axes)
	c := &Controller{
		cfg:    cfg,
		n:      n,
		dt:     cfg.Period.Seconds(),
		phase:  PhaseIdle,
		states: make([]axisState, n),
		stopV:  make([]float64, n),
		stopQ:  make([]float64, n),
	}
	return c
}

// Reset clears state and sets the measured home configuration.
func (c *Controller) Reset(position []float64) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if len(position) != c.n {
		panic(fmt.Sprintf("control.Reset: expected %d positions, got %d", c.n, len(position)))
	}
	c.traj = nil
	c.idx = 0
	c.phase = PhaseIdle
	c.fault = nil
	c.ticks = 0
	c.endTicks = 0
	c.holdTicks = 0
	c.recTicks = 0
	c.stopSettled = false
	for i := 0; i < c.n; i++ {
		c.states[i] = axisState{q: position[i], targetPos: position[i], feedbackOK: true}
		c.stopV[i] = 0
		c.stopQ[i] = position[i]
	}
}

// Load arms a trajectory starting from the current measured configuration.
func (c *Controller) Load(tr *trajectory.Trajectory) error {
	c.mu.Lock()
	defer c.mu.Unlock()
	if tr == nil || tr.Len() == 0 {
		return mcerr.New(mcerr.CodeTargetContradictory, "cannot load an empty trajectory")
	}
	if len(tr.At(0).Position) != c.n {
		return mcerr.New(mcerr.CodeTargetContradictory,
			fmt.Sprintf("trajectory has %d axes, controller has %d", len(tr.At(0).Position), c.n))
	}
	if c.phase == PhaseFaulted {
		return mcerr.New(mcerr.CodeFaultActive, "cannot load trajectory while a fault is active")
	}
	c.traj = tr
	c.idx = 0
	c.phase = PhaseRunning
	c.fault = nil
	c.endTicks = 0
	c.holdTicks = 0
	c.recTicks = 0
	c.stopSettled = false
	for i := range c.states {
		s := &c.states[i]
		s.integral = 0
		s.targetPos = tr.At(0).Position[i]
		s.targetVel = tr.At(0).Velocity[i]
		s.err = s.targetPos - s.q
	}
	return nil
}

// Cancel requests a controlled safe stop of the current move.
func (c *Controller) Cancel() {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.phase == PhaseRunning {
		c.beginStopLocked("canceled")
	}
}

func (c *Controller) beginStopLocked(_ string) {
	c.phase = PhaseStopping
	c.stopSettled = false
	for i := range c.states {
		// Ramp starts from measured state (last healthy reading), never
		// from the trajectory reference, so no position jump is commanded.
		c.stopV[i] = c.states[i].v
		c.stopQ[i] = c.states[i].q
	}
}

// Tick executes one control cycle with encoder feedback. feedbackOK[i]
// == false means the encoder reading for axis i was lost this cycle; in that
// case the position/velocity slices for that axis are ignored.
func (c *Controller) Tick(_ time.Time, measuredPosition, measuredVelocity []float64, feedbackOK []bool) (Phase, *Fault) {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.ticks++
	if len(measuredPosition) != c.n || len(measuredVelocity) != c.n || len(feedbackOK) != c.n {
		c.enterFatalLocked(-1, FaultCommandNonFinite, "feedback cardinality mismatch")
		return c.phase, c.fault
	}
	for i := 0; i < c.n; i++ {
		s := &c.states[i]
		if !feedbackOK[i] {
			s.feedbackOK = false
			s.dropoutRun++
			continue
		}
		if !numeric.Finite(measuredPosition[i]) || !numeric.Finite(measuredVelocity[i]) {
			c.enterFatalLocked(i, FaultCommandNonFinite,
				fmt.Sprintf("non-finite feedback (q=%v, v=%v)", measuredPosition[i], measuredVelocity[i]))
			return c.phase, c.fault
		}
		s.feedbackOK = true
		s.dropoutRun = 0
		s.q = measuredPosition[i]
		s.v = measuredVelocity[i]
	}

	switch c.phase {
	case PhaseIdle, PhaseDone:
		c.zeroTorqueLocked()
		return c.phase, nil
	case PhaseFaulted:
		// Keep feeding measurements into the state even while faulted.
		// A recoverable fault applies a SAFE HOLD: position P/D torque
		// toward the current reference brakes the axes and parks them at
		// the reference (no integral, no feed-forward), so motion does not
		// drift away during the fault window. A fatal fault removes all
		// torque instead. Healthy feedback is still required for recovery,
		// and a fault that worsens while faulted escalates to fatal.
		escalated := false
		for i := range c.states {
			s := &c.states[i]
			s.err = s.targetPos - s.q
			if !s.feedbackOK {
				if s.dropoutRun > c.cfg.Fault.DropoutMaxTicks {
					c.enterFatalLocked(i, FaultFeedbackDead,
						fmt.Sprintf("feedback lost for %d cycles (> %d) while faulted",
							s.dropoutRun, c.cfg.Fault.DropoutMaxTicks))
					return c.phase, c.fault
				}
			} else if abs(s.err) > c.cfg.Fault.DivergeError {
				c.enterFatalLocked(i, FaultTrackingDiverged,
					fmt.Sprintf("tracking error %.4f rad exceeds divergence threshold %.4f while faulted",
						s.err, c.cfg.Fault.DivergeError))
				escalated = true
			}
		}
		if c.fault != nil && c.fault.Recoverable && !escalated {
			c.safeHoldTorqueLocked()
		} else {
			c.zeroTorqueLocked()
		}
		c.trackRecoveryLocked()
		return c.phase, c.fault
	}

	// Update dropout counters into faults before anything else. Use the
	// current-cycle argument; state.feedbackOK is also refreshed above.
	for i := range c.states {
		s := &c.states[i]
		if !feedbackOK[i] {
			if s.dropoutRun > c.cfg.Fault.DropoutMaxTicks {
				c.enterFatalLocked(i, FaultFeedbackDead,
					fmt.Sprintf("feedback lost for %d cycles (> %d)", s.dropoutRun, c.cfg.Fault.DropoutMaxTicks))
				return c.phase, c.fault
			}
			c.enterRecoverableLocked(i, FaultFeedbackLost,
				fmt.Sprintf("feedback lost at tick %d", c.ticks))
			return c.phase, c.fault
		}
	}

	if c.phase == PhaseStopping {
		c.tickSafeStopLocked()
		c.computeTorquesLocked()
		c.checkSaturationLocked()
		if c.fault != nil {
			return c.phase, c.fault
		}
		if c.stopSettled {
			c.phase = PhaseDone
		}
		return c.phase, nil
	}

	// Running: advance reference.
	c.idx++
	if c.idx >= c.traj.Len() {
		c.idx = c.traj.Len() - 1
		c.endTicks++
	}
	pt := c.traj.At(c.idx)
	allConverged := true
	for i := range c.states {
		s := &c.states[i]
		s.targetPos = pt.Position[i]
		s.targetVel = pt.Velocity[i]
		s.err = s.targetPos - s.q
		if abs(s.err) > c.cfg.Fault.DivergeError {
			c.enterFatalLocked(i, FaultTrackingDiverged,
				fmt.Sprintf("tracking error %.4f rad exceeds divergence threshold %.4f",
					s.err, c.cfg.Fault.DivergeError))
			return c.phase, c.fault
		}
		if abs(s.err) > c.cfg.Fault.MaxPositionError {
			c.enterRecoverableLocked(i, FaultTracking,
				fmt.Sprintf("tracking error %.4f rad exceeds threshold %.4f",
					s.err, c.cfg.Fault.MaxPositionError))
			return c.phase, c.fault
		}
		if abs(s.err) > c.cfg.Tolerance.Position || abs(s.v) > c.cfg.Tolerance.Velocity {
			allConverged = false
		}
	}
	c.computeTorquesLocked()
	c.checkSaturationLocked()
	if c.fault != nil {
		return c.phase, c.fault
	}
	if c.idx == c.traj.Len()-1 {
		if allConverged {
			c.holdTicks++
			if c.holdTicks >= c.cfg.Tolerance.HoldTicks {
				c.phase = PhaseDone
				c.zeroTorqueLocked()
				return c.phase, nil
			}
		} else {
			c.holdTicks = 0
			if c.endTicks > c.cfg.Fault.EndGraceTicks {
				c.enterFatalLocked(-1, FaultNotConverged,
					fmt.Sprintf("move ended but tracking did not converge within %d grace ticks",
						c.cfg.Fault.EndGraceTicks))
				return c.phase, c.fault
			}
		}
	}
	return c.phase, nil
}

// computeTorquesLocked runs feed-forward + PID and records saturation.
func (c *Controller) computeTorquesLocked() {
	var pt trajectory.Point
	if c.traj != nil {
		pt = c.traj.At(c.idx)
	} else {
		pt = trajectory.Point{
			Position: make([]float64, c.n), Velocity: make([]float64, c.n),
			Accel: make([]float64, c.n), Torque: make([]float64, c.n),
		}
	}
	tauFF := pt.Torque
	if len(tauFF) != c.n {
		tauFF = make([]float64, c.n)
	}
	for i := 0; i < c.n; i++ {
		a := &c.cfg.Axes[i]
		s := &c.states[i]
		if c.phase == PhaseStopping {
			s.targetPos = c.stopQ[i]
			s.targetVel = c.stopV[i]
			s.err = s.targetPos - s.q
			// No trajectory feed-forward while ramping to rest.
			s.integral += numeric.Clamp(a.Gains.Ki*s.err*c.dt, -a.Gains.IntegralLimit, a.Gains.IntegralLimit)
			tau := a.Gains.Kp*s.err + s.integral + a.Gains.Kd*(s.targetVel-s.v)
			s.saturated = false
			if tau > a.MaxTorque {
				tau = a.MaxTorque
				s.saturated = true
			} else if tau < -a.MaxTorque {
				tau = -a.MaxTorque
				s.saturated = true
			}
			if !numeric.Finite(tau) {
				c.enterFatalLocked(i, FaultCommandNonFinite, "controller produced non-finite torque")
				return
			}
			s.tauCmd = tau
			continue
		}
		s.integral += numeric.Clamp(a.Gains.Ki*s.err*c.dt, -a.Gains.IntegralLimit, a.Gains.IntegralLimit)
		tau := tauFF[i] + a.Gains.Kp*s.err + s.integral + a.Gains.Kd*(s.targetVel-s.v)
		s.saturated = false
		if tau > a.MaxTorque {
			tau = a.MaxTorque
			s.saturated = true
		} else if tau < -a.MaxTorque {
			tau = -a.MaxTorque
			s.saturated = true
		}
		if !numeric.Finite(tau) {
			c.enterFatalLocked(i, FaultCommandNonFinite, "controller produced non-finite torque")
			return
		}
		s.tauCmd = tau
	}
}

// checkSaturationLocked promotes sustained saturation to a recoverable fault.
func (c *Controller) checkSaturationLocked() {
	for i := range c.states {
		s := &c.states[i]
		if s.saturated {
			s.saturationRun++
			if s.saturationRun > c.cfg.Fault.SaturationMaxTicks {
				c.enterRecoverableLocked(i, FaultDriveSaturated,
					fmt.Sprintf("drive saturated for %d consecutive cycles", s.saturationRun))
				return
			}
		} else {
			s.saturationRun = 0
		}
	}
}

// tickSafeStopLocked integrates a per-axis ramp-to-zero reference at the
// configured safe deceleration.
func (c *Controller) tickSafeStopLocked() {
	allSettled := true
	for i := range c.states {
		v := c.stopV[i]
		if abs(v) < 1e-12 {
			c.stopV[i] = 0
			continue
		}
		allSettled = false
		dv := c.cfg.Fault.SafeDecel * c.dt
		if abs(v) <= dv {
			c.stopV[i] = 0
		} else {
			c.stopV[i] -= sign(v) * dv
		}
		c.stopQ[i] += 0.5 * (v + c.stopV[i]) * c.dt
		lim := &c.cfg.Axes[i]
		c.stopQ[i] = numeric.Clamp(c.stopQ[i], lim.MinPosition, lim.MaxPosition)
	}
	if allSettled {
		for i := range c.states {
			if abs(c.states[i].v) > c.cfg.Tolerance.Velocity {
				allSettled = false
				break
			}
		}
	}
	c.stopSettled = allSettled
}

func (c *Controller) zeroTorqueLocked() {
	for i := range c.states {
		c.states[i].tauCmd = 0
		c.states[i].saturated = false
	}
}

// safeHoldTorqueLocked commands pure proportional + derivative torque that
// parks each healthy axis at its current reference and brakes its velocity.
// It uses neither the integral nor trajectory feed-forward, and clips to
// the torque ceiling (sustained saturation during hold is not promoted to a
// fault because the machine is already in a fault state).
func (c *Controller) safeHoldTorqueLocked() {
	for i := 0; i < c.n; i++ {
		a := &c.cfg.Axes[i]
		s := &c.states[i]
		if !s.feedbackOK {
			// Never act on a missing encoder reading.
			s.tauCmd = 0
			s.saturated = false
			continue
		}
		s.err = s.targetPos - s.q
		tau := a.Gains.Kp*s.err - a.Gains.Kd*s.v
		s.saturated = false
		if tau > a.MaxTorque {
			tau, s.saturated = a.MaxTorque, true
		} else if tau < -a.MaxTorque {
			tau, s.saturated = -a.MaxTorque, true
		}
		if !numeric.Finite(tau) {
			c.enterFatalLocked(i, FaultCommandNonFinite, "safe hold produced non-finite torque")
			return
		}
		s.tauCmd = tau
	}
}

func (c *Controller) enterRecoverableLocked(axis int, code FaultCode, msg string) {
	c.phase = PhaseFaulted
	c.zeroTorqueLocked()
	c.fault = &Fault{Code: code, Axis: axis, Tick: c.ticks, Recoverable: true, Message: msg}
}

func (c *Controller) enterFatalLocked(axis int, code FaultCode, msg string) {
	c.phase = PhaseFaulted
	c.zeroTorqueLocked()
	c.fault = &Fault{Code: code, Axis: axis, Tick: c.ticks, Recoverable: false, Message: msg}
}

// trackRecoveryLocked counts cycles in which every recovery precondition
// holds; the caller must still invoke Recover explicitly.
func (c *Controller) trackRecoveryLocked() {
	if c.fault == nil || !c.fault.Recoverable {
		c.recTicks = 0
		return
	}
	for i := range c.states {
		s := &c.states[i]
		if !s.feedbackOK {
			c.recTicks = 0
			return
		}
		if abs(s.err) > 0.5*c.cfg.Fault.MaxPositionError || abs(s.v) > c.cfg.Tolerance.Velocity {
			c.recTicks = 0
			return
		}
	}
	c.recTicks++
}

// LastTorque returns the most recent commanded torque per axis (N·m).
func (c *Controller) LastTorque() []float64 {
	c.mu.Lock()
	defer c.mu.Unlock()
	out := make([]float64, c.n)
	for i := range c.states {
		out[i] = c.states[i].tauCmd
	}
	return out
}

// Phase returns the current phase.
func (c *Controller) Phase() Phase {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.phase
}

// Fault returns a copy of the active fault or nil.
func (c *Controller) Fault() *Fault {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.fault == nil {
		return nil
	}
	f := *c.fault
	return &f
}

// Status returns a point-in-time consistent copy of all axis statuses.
func (c *Controller) Status() []AxisStatus {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.statusLocked()
}

// Snapshot returns the phase, active fault (copy) and all axis statuses in a
// single critical section, so concurrent readers can never see a phase that
// disagrees with the statuses.
func (c *Controller) Snapshot() (Phase, *Fault, []AxisStatus) {
	c.mu.Lock()
	defer c.mu.Unlock()
	var f *Fault
	if c.fault != nil {
		fc := *c.fault
		f = &fc
	}
	return c.phase, f, c.statusLocked()
}

func (c *Controller) statusLocked() []AxisStatus {
	out := make([]AxisStatus, c.n)
	for i := range c.states {
		s := &c.states[i]
		out[i] = AxisStatus{
			TargetPosition: s.targetPos,
			TargetVelocity: s.targetVel,
			Position:       s.q,
			Velocity:       s.v,
			Error:          s.err,
			TorqueCommand:  s.tauCmd,
			Saturated:      s.saturated,
			FeedbackOK:     s.feedbackOK,
		}
	}
	return out
}

// Progress returns consumed sample count and total samples.
func (c *Controller) Progress() (index, total int) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.traj == nil {
		return 0, 0
	}
	return c.idx, c.traj.Len()
}

// Recover clears a recoverable fault only when all recovery preconditions
// hold; fatal faults and unmet preconditions are rejected with a classified
// error and the controller never silently resumes motion.
func (c *Controller) Recover() error {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.phase != PhaseFaulted || c.fault == nil {
		return mcerr.New(mcerr.CodeFaultActive, "no active fault to recover from")
	}
	if !c.fault.Recoverable {
		return mcerr.New(mcerr.CodeFaultActive,
			fmt.Sprintf("fault %s is not recoverable; reset/re-home is required", c.fault.Code))
	}
	if c.recTicks < c.cfg.Fault.RecoveryHoldTicks {
		return mcerr.New(mcerr.CodeRecoveryPrecondition,
			fmt.Sprintf("recovery conditions held for %d ticks, require %d (feedback healthy, |e|<=%.4f rad, |v|<=%.4f rad/s)",
				c.recTicks, c.cfg.Fault.RecoveryHoldTicks,
				0.5*c.cfg.Fault.MaxPositionError, c.cfg.Tolerance.Velocity))
	}
	// Resume the remaining portion of the move from the current reference.
	c.fault = nil
	c.recTicks = 0
	if c.traj != nil && c.idx < c.traj.Len()-1 {
		c.phase = PhaseRunning
	} else {
		c.phase = PhaseIdle
	}
	for i := range c.states {
		c.states[i].integral = 0
		c.states[i].saturationRun = 0
	}
	return nil
}

func abs(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}

func sign(x float64) float64 {
	if x > 0 {
		return 1
	}
	if x < 0 {
		return -1
	}
	return 0
}
