// Package engine ties planner, simulated plant, feedback controller and
// command arbiter behind a single mutex. It provides deterministic, fixed
// clock stepping and guarantees that externally observed axis state and
// execution progress never appear half-updated under concurrent entries.
//
// Lifecycle:
//
//	IDLE -> RUNNING -> IDLE            (normal completion)
//	IDLE -> RUNNING -> STOPPING -> IDLE(preempt/cancel, safe stop executed)
//	*    -> FAULT (recoverable fault) -> RUNNING stopping ramp -> IDLE
//	                                   -> Recover() possible once stopped
//	*    -> FATAL (unrecoverable)      -> Reconfigure() is the only exit
//
// A recoverable fault is not silently cleared: Recover() is required and is
// refused with RESET_PRECONDITIONS_NOT_MET until the physical condition is
// gone, velocity is zero and tracking error is back inside tolerance.
package engine

import (
	"fmt"
	"sync"

	"github.com/h3yow/robot-motion-controller-go/internal/motion/arbiter"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/classify"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/config"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/controller"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/kinematics"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/plant"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/trajectory"
)

// State is the engine lifecycle state.
type State string

const (
	StateIdle     State = "IDLE"
	StateRunning  State = "RUNNING"
	StateStopping State = "STOPPING"
	StateFault    State = "FAULT"
	StateFatal    State = "FATAL"
)

// AxisSnapshot is one axis' externally visible state at one control cycle.
type AxisSnapshot struct {
	PositionCommand float64
	VelocityCommand float64
	PositionActual  float64
	VelocityActual  float64
	TorqueCommand   float64
	Saturated       bool
}

// Snapshot is a consistent (single-lock) view of the whole engine.
type Snapshot struct {
	State      State
	ActiveID   arbiter.ID
	Cycle      int64
	Fault      *controller.Fault
	LastReason classify.Reason
	Axes       []AxisSnapshot
}

// Result reports how one Step/Run resolved.
type Result struct {
	State    State
	ID       arbiter.ID
	Done     bool
	Rejected classify.Reason
	Fault    *controller.Fault
}

type activeMotion struct {
	id       arbiter.ID
	priority int
	traj     *trajectory.Trajectory
	idx      int
	// terminal hold (settle) bookkeeping
	holding      bool
	settleStreak int
	holdCycles   int
}

// stopRamp decelerates the axes to rest after a cancel/preempt/fault.
type stopRamp struct {
	traj *trajectory.Trajectory
	idx  int
	// reason is why the stop was initiated.
	reason classify.Reason
}

// pendingMotion is an accepted higher-priority command armed to start the
// instant the incumbent's safe stop completes.
type pendingMotion struct {
	id       arbiter.ID
	priority int
	traj     *trajectory.Trajectory
}

// Engine is the motion execution engine.
type Engine struct {
	mu  sync.Mutex
	cfg *config.Config

	planner    *trajectory.Planner
	plant      *plant.Sim
	controller *controller.Controller
	arb        *arbiter.Arbiter

	state       State
	cycle       int64
	active      *activeMotion
	stopping    *stopRamp
	pending     *pendingMotion
	pendingOp   arbiter.Op
	pendingGoal []float64
	pendingPath [][]float64
	fault       *controller.Fault

	lastReason classify.Reason
	lastAxes   []AxisSnapshot
	torque     []float64

	notify chan struct{}
	quit   chan struct{}
	ticked bool
	wg     sync.WaitGroup

	disturbance []float64
}

// New constructs an Engine from validated configuration.
func New(cfg *config.Config) *Engine {
	if err := cfg.Validate(); err != nil {
		panic("engine.New: invalid configuration: " + err.Error())
	}
	n := cfg.AxisCount()
	inertia := make([]float64, n)
	gravity := make([]float64, n)
	for i, a := range cfg.Axes {
		inertia[i] = a.Inertia
		gravity[i] = a.GravityTorque
	}
	return &Engine{
		cfg:         cfg,
		planner:     trajectory.NewPlanner(cfg),
		plant:       plant.New(cfg.ControlPeriod, inertia, gravity),
		controller:  controller.New(cfg),
		arb:         arbiter.New(),
		state:       StateIdle,
		lastAxes:    make([]AxisSnapshot, n),
		torque:      make([]float64, n),
		notify:      make(chan struct{}, 1),
		quit:        make(chan struct{}),
		disturbance: make([]float64, n),
	}
}

func (e *Engine) reject(reason classify.Reason, format string, args ...any) (arbiter.Decision, error) {
	e.lastReason = reason
	return arbiter.Decision{Reason: reason}, classify.New(reason, fmt.Sprintf(format, args...))
}

// Submit arbitrates and, if accepted, plans the trajectory synchronously
// (under the engine lock) and arms it. Planning rejection is reported with
// the same distinguishable reasons as the planner.
func (e *Engine) Submit(cmd arbiter.Command) (arbiter.Decision, error) {
	e.mu.Lock()
	defer e.mu.Unlock()

	active := e.active != nil && (e.state == StateRunning || e.state == StateStopping)
	var activeID arbiter.ID
	activePrio := 0
	if active {
		activeID = e.active.id
		activePrio = e.active.priority
	}
	faultActive := e.state == StateFault || e.state == StateFatal
	d := e.arb.Submit(cmd, active, activeID, activePrio, faultActive)
	if !d.Accepted {
		e.lastReason = d.Reason
		return d, classify.New(d.Reason, d.Message)
	}
	if cmd.Op == arbiter.OpReset {
		e.lastReason = classify.ReasonOK
		return d, nil // recovery action handled by Recover()
	}
	if cmd.Op == arbiter.OpCancel {
		e.beginStopLocked(classify.ReasonCancelled)
		e.lastReason = classify.ReasonCancelled
		return d, nil
	}

	// Motion: build goal trajectory from the CURRENT commanded/actual state.
	start := e.currentJointPositionLocked()
	var tr *trajectory.Trajectory
	var err error
	vel := e.velocityEnvelopeLocked()
	acc := e.accelEnvelopeLocked()
	if cmd.Op == arbiter.OpMoveTo {
		if len(cmd.Goal) != e.cfg.AxisCount() {
			return e.reject(classify.ReasonContradictoryTarget,
				"goal has %d components, want %d", len(cmd.Goal), e.cfg.AxisCount())
		}
		tr, err = e.planner.PointToPoint(start, append([]float64(nil), cmd.Goal...), vel, acc)
	} else {
		tr, err = e.planner.Path(start, cmd.Waypoints, vel, acc)
	}
	if err != nil {
		// Planning failure: the accepted command cannot run; record it so it
		// cannot be silently retried, keep incumbent untouched.
		e.arb.Complete(d.ID)
		e.lastReason, _ = reasonOf(err)
		r, _ := classify.AsReject(err)
		return arbiter.Decision{Reason: r.Reason}, err
	}

	// Atomic arbitration result handling.
	if d.Preempted {
		// Incumbent must come to a safe stop first. The new trajectory was
		// planned from the current command point; re-base it onto the actual
		// position by remembering the goal and re-planning at stop end.
		e.beginStopLocked(classify.ReasonCancelled)
		e.pending = &pendingMotion{id: d.ID, priority: cmd.Priority}
		if cmd.Op == arbiter.OpMoveTo {
			e.pending.traj = nil
			e.pendingGoal = append([]float64(nil), cmd.Goal...)
			e.pendingPath = nil
			e.pendingOp = arbiter.OpMoveTo
		} else {
			e.pendingGoal = nil
			e.pendingPath = cmd.Waypoints
			e.pendingOp = arbiter.OpPath
		}
	} else {
		e.armLocked(d.ID, cmd.Priority, tr)
	}
	return d, nil
}

// armLocked installs a fully planned trajectory as the running motion.
func (e *Engine) armLocked(id arbiter.ID, prio int, tr *trajectory.Trajectory) {
	e.active = &activeMotion{id: id, priority: prio, traj: tr, idx: 0}
	e.stopping = nil
	e.pending = nil
	e.state = StateRunning
	e.fault = nil
	e.controller.Reset()
	e.lastReason = classify.ReasonOK
}

func reasonOf(err error) (classify.Reason, bool) {
	r, ok := classify.AsReject(err)
	if !ok {
		return classify.ReasonInvalidCommand, false
	}
	return r.Reason, true
}

// currentJointPositionLocked returns the position the next move must start
// from: the executed trajectory command while running, actual otherwise.
func (e *Engine) currentJointPositionLocked() []float64 {
	n := e.cfg.AxisCount()
	pos := make([]float64, n)
	if e.active != nil && e.active.idx < len(e.active.traj.Points) {
		copy(pos, e.active.traj.Points[e.active.idx].Position)
	} else {
		copy(pos, e.plant.Position())
	}
	return pos
}

func (e *Engine) velocityEnvelopeLocked() []float64 {
	v := make([]float64, e.cfg.AxisCount())
	for i, a := range e.cfg.Axes {
		v[i] = a.MaxVelocity
	}
	return v
}

func (e *Engine) accelEnvelopeLocked() []float64 {
	a := make([]float64, e.cfg.AxisCount())
	for i, ax := range e.cfg.Axes {
		a[i] = ax.MaxAccel
	}
	return a
}

// beginStopLocked builds a safe deceleration ramp from the current commanded
// point to the current encoder position, using configured limits. When
// feedback is unavailable the engine instead commands zero torque and waits
// for a fault/stop (no blind motion without measurements).
func (e *Engine) beginStopLocked(reason classify.Reason) {
	if e.active == nil {
		e.state = StateIdle
		return
	}
	if !e.plant.FeedbackValid() {
		// Cannot plan a measured stop: hold zero torque; the FAULT state and
		// recovery preconditions guard re-entry.
		for i := range e.torque {
			e.torque[i] = 0
		}
		e.stopping = &stopRamp{reason: reason}
		e.state = StateStopping
		return
	}
	cur := e.active.traj.Points[e.active.idx]
	goal := e.plant.Position()
	tr, err := e.planner.PointToPoint(append([]float64(nil), cur.Position...), goal,
		e.velocityEnvelopeLocked(), e.accelEnvelopeLocked())
	if err != nil {
		// Actual position is always within travel (the plant cannot leave
		// it); if planning still fails, freeze at zero torque.
		for i := range e.torque {
			e.torque[i] = 0
		}
		e.stopping = &stopRamp{reason: reason}
		e.state = StateStopping
		return
	}
	e.stopping = &stopRamp{traj: tr, idx: 0, reason: reason}
	e.state = StateStopping
}

// signal wakes a RunTicked loop / waiters; non-blocking.
func (e *Engine) signal() {
	select {
	case e.notify <- struct{}{}:
	default:
	}
}

// Step advances exactly one fixed control period. It is safe to call
// concurrently with Submit/Snapshot; everything is serialized by one mutex.
func (e *Engine) Step() Result {
	e.mu.Lock()
	defer e.mu.Unlock()
	return e.stepLocked()
}

func (e *Engine) stepLocked() Result {
	n := e.cfg.AxisCount()

	// Even in IDLE/FATAL, update the plant with zero torque so snapshots of
	// a coasting axis keep moving consistently.
	applyZero := func() {
		fb := e.plant.Step(make([]float64, n), e.disturbance)
		e.publishLocked(trajectory.Point{
			Position: e.plant.Position(), Velocity: make([]float64, n),
			Accel: make([]float64, n)}, make([]float64, n), make([]bool, n), fb)
		e.cycle++
	}

	switch e.state {
	case StateIdle:
		applyZero()
		return Result{State: e.state}
	case StateFatal:
		// Power off: no torque is applied ever again before reconfiguration.
		applyZero()
		return Result{State: e.state}
	case StateFault:
		// Safe stop: read current velocity, compute a deterministic damping
		// brake from it, then apply it this same cycle. With feedback lost the
		// only safe output is zero torque. Command position is frozen at the
		// last trajectory point so the fault-induced deviation stays visible.
		frozen := e.frozenCommandLocked(n)
		fb := e.plant.Read()
		tau := e.brakeTorqueFromReadingLocked(fb)
		e.plant.Apply(tau, e.disturbance)
		fb = e.plant.Read() // post-apply measurement for a consistent snapshot
		e.publishLocked(frozen, tau, make([]bool, n), fb)
		e.cycle++
		return Result{State: StateFault, ID: e.safeActiveID(), Fault: e.fault}
	}

	if e.state == StateStopping {
		return e.stepStoppingLocked()
	}
	if e.active == nil {
		applyZero()
		return Result{State: e.state}
	}

	cmd := e.active.traj.Points[e.active.idx]
	fb := e.plant.Read()
	tau, sat, f := e.controller.Step(cmd, fb)
	if f != nil {
		// A fault this cycle: do not actuate the computed torque, power off.
		e.plant.Apply(make([]float64, n), e.disturbance)
		e.publishLocked(cmd, make([]float64, n), make([]bool, n), e.plant.Read())
		e.cycle++
		return e.latchFaultLocked(f)
	}
	e.plant.Apply(tau, e.disturbance)
	e.torque = tau
	e.publishLocked(cmd, tau, sat, e.plant.Read())
	e.cycle++

	last := len(e.active.traj.Points) - 1
	if e.active.idx >= last {
		// Terminal hold: keep applying the final setpoint until the axes
		// have actually converged, so a motion is never reported Done with a
		// residual error. Bounded; failure to converge is an unrecoverable
		// CONTROL_DIVERGED rather than a false "complete".
		e.active.idx = last
		e.active.holding = true
		goal := e.active.traj.Points[last].Position
		posTol := settlePosTolFor(e.cfg.Controller.TrackingTolerance)
		velTol := settleVelTolFor(e.cfg.ControlPeriod)
		converged := true
		for i := 0; i < e.cfg.AxisCount(); i++ {
			if absF(fb.Position[i]-goal[i]) > posTol ||
				absF(fb.Velocity[i]) > velTol {
				converged = false
				break
			}
		}
		if converged {
			e.active.settleStreak++
		} else {
			e.active.settleStreak = 0
		}
		e.active.holdCycles++
		if e.active.settleStreak >= settleStreakCycles {
			return e.finishLocked(classify.ReasonOK)
		}
		budget := int(settleBudgetSeconds/e.cfg.ControlPeriod + 0.5)
		if e.active.holdCycles > budget {
			return e.latchFaultLocked(&controller.Fault{
				Kind: controller.FaultControlDiverged, Axis: -1,
				Message: fmt.Sprintf("motion did not converge within %.1f settle budget (residual vs goal %v)",
					settleBudgetSeconds, residualString(fb.Position, goal)),
			})
		}
		return Result{State: StateRunning, ID: e.active.id}
	}
	e.active.idx++
	return Result{State: StateRunning, ID: e.active.id}
}

func residualString(actual, goal []float64) string {
	parts := ""
	for i := range actual {
		if i > 0 {
			parts += ","
		}
		parts += fmt.Sprintf("axis%d=%v", i, actual[i]-goal[i])
	}
	return parts
}

// stepStoppingLocked executes the safe-stop ramp, or a zero-torque coast
// when no measured ramp was possible, until the axes are at rest, then
// either arms the pending (preempting) command or returns to IDLE.
func (e *Engine) stepStoppingLocked() Result {
	n := e.cfg.AxisCount()
	ramp := e.stopping
	reason := classify.ReasonCancelled
	if ramp != nil {
		reason = ramp.reason
	}

	var cmd trajectory.Point
	if ramp != nil && ramp.traj != nil {
		cmd = ramp.traj.Points[ramp.idx]
	} else {
		// Coast under zero torque; hold position command at last known value.
		cmd = trajectory.Point{
			Position: make([]float64, n), Velocity: make([]float64, n),
			Accel: make([]float64, n),
		}
		for i := range cmd.Position {
			cmd.Position[i] = e.lastAxes[i].PositionCommand
		}
	}
	fb := e.plant.Read()
	var tau []float64
	var sat []bool
	var f *controller.Fault
	if ramp != nil && ramp.traj != nil {
		tau, sat, f = e.controller.Step(cmd, fb)
	} else {
		tau = make([]float64, n)
		sat = make([]bool, n)
	}
	if f != nil {
		e.plant.Apply(make([]float64, n), e.disturbance)
		e.publishLocked(cmd, make([]float64, n), make([]bool, n), e.plant.Read())
		e.cycle++
		return e.latchFaultLocked(f)
	}
	e.plant.Apply(tau, e.disturbance)
	e.torque = tau
	fb = e.plant.Read()
	e.publishLocked(cmd, tau, sat, fb)
	e.cycle++

	finished := false
	if ramp != nil && ramp.traj != nil {
		ramp.idx++
		finished = ramp.idx >= len(ramp.traj.Points)
	} else {
		// Coast stop completes once feedback is valid and velocity is ~0.
		if fb.Valid {
			finished = true
			for _, v := range fb.Velocity {
				if absF(v) > stopVelocityEps {
					finished = false
					break
				}
			}
		}
	}
	if !finished {
		return Result{State: StateStopping, ID: e.safeActiveID()}
	}
	return e.completeStopLocked(reason)
}

func (e *Engine) safeActiveID() (id arbiter.ID) {
	if e.active != nil {
		id = e.active.id
	}
	return id
}

// completeStopLocked closes the incumbent after a safe stop and arms the
// pending preempting command (re-planned from the stopped position) or
// returns to IDLE for a plain cancel.
func (e *Engine) completeStopLocked(reason classify.Reason) Result {
	id := e.safeActiveID()
	if e.active != nil {
		e.arb.Complete(e.active.id)
	}
	e.active = nil
	e.stopping = nil
	for i := range e.torque {
		e.torque[i] = 0
	}
	e.controller.Reset()

	if e.pending != nil {
		p := e.pending
		e.pending = nil
		start := e.plant.Position()
		var tr *trajectory.Trajectory
		var err error
		if e.pendingOp == arbiter.OpMoveTo {
			tr, err = e.planner.PointToPoint(start, e.pendingGoal,
				e.velocityEnvelopeLocked(), e.accelEnvelopeLocked())
		} else {
			tr, err = e.planner.Path(start, e.pendingPath,
				e.velocityEnvelopeLocked(), e.accelEnvelopeLocked())
		}
		e.pendingGoal = nil
		e.pendingPath = nil
		if err != nil {
			// Target became invalid relative to the stopped position: reject
			// distinctly, stay safe in IDLE rather than moving.
			r, _ := classify.AsReject(err)
			e.state = StateIdle
			e.lastReason = r.Reason
			return Result{State: StateIdle, ID: p.id, Rejected: r.Reason}
		}
		e.armLocked(p.id, p.priority, tr)
		return Result{State: StateRunning, ID: p.id}
	}

	e.state = StateIdle
	e.lastReason = reason
	return Result{State: StateIdle, ID: id, Done: false, Rejected: reason}
}

// publishLocked stores a consistent per-axis view for Snapshot.
func (e *Engine) publishLocked(cmd trajectory.Point, tau []float64, sat []bool, fb plant.EncoderReading) {
	for i := 0; i < e.cfg.AxisCount(); i++ {
		ax := AxisSnapshot{TorqueCommand: tau[i], Saturated: i < len(sat) && sat[i]}
		if i < len(cmd.Position) {
			ax.PositionCommand = cmd.Position[i]
			ax.VelocityCommand = cmd.Velocity[i]
		}
		if fb.Valid {
			ax.PositionActual = fb.Position[i]
			ax.VelocityActual = fb.Velocity[i]
		} else {
			// Keep last known actuals but mark nothing new; consumers see the
			// FAULT state as authoritative.
			ax.PositionActual = e.lastAxes[i].PositionActual
			ax.VelocityActual = e.lastAxes[i].VelocityActual
		}
		e.lastAxes[i] = ax
	}
}

// frozenCommandLocked returns the command point at the moment execution was
// interrupted (zero velocity/accel), so snapshots during FAULT show the
// last command versus the actual stopped position.
func (e *Engine) frozenCommandLocked(n int) trajectory.Point {
	pt := trajectory.Point{
		Position: make([]float64, n), Velocity: make([]float64, n), Accel: make([]float64, n)}
	if e.active != nil {
		idx := e.active.idx
		if idx >= len(e.active.traj.Points) {
			idx = len(e.active.traj.Points) - 1
		}
		copy(pt.Position, e.active.traj.Points[idx].Position)
	} else {
		copy(pt.Position, e.plant.Position())
	}
	return pt
}

// brakeTorqueFromReadingLocked computes the damping brake from a specific
// reading (zero torque when the reading is invalid).
func (e *Engine) brakeTorqueFromReadingLocked(fb plant.EncoderReading) []float64 {
	n := e.cfg.AxisCount()
	tau := make([]float64, n)
	if !fb.Valid {
		return tau
	}
	for i, a := range e.cfg.Axes {
		b := -brakeGain * fb.Velocity[i]
		if b > a.MaxTorque {
			b = a.MaxTorque
		} else if b < -a.MaxTorque {
			b = -a.MaxTorque
		}
		tau[i] = b
	}
	return tau
}

// brakeTorqueLocked returns a per-axis damping torque opposing velocity,
// clamped to MaxTorque. Returns zero torque when feedback is invalid.
func (e *Engine) brakeTorqueLocked() []float64 {
	n := e.cfg.AxisCount()
	tau := make([]float64, n)
	if !e.plant.FeedbackValid() {
		return tau
	}
	vel := e.plant.Velocity()
	for i, a := range e.cfg.Axes {
		b := -brakeGain * vel[i]
		if b > a.MaxTorque {
			b = a.MaxTorque
		} else if b < -a.MaxTorque {
			b = -a.MaxTorque
		}
		tau[i] = b
	}
	return tau
}

// latchFaultLocked transitions to FAULT or FATAL and commands safe torque.
func (e *Engine) latchFaultLocked(f *controller.Fault) Result {
	// The controller already returned zero torque this cycle; freeze the
	// command index. A recoverable fault remains safe-stopped in FAULT; an
	// unrecoverable one powers off into FATAL pending reconfiguration.
	for i := range e.torque {
		e.torque[i] = 0
	}
	if f.Kind.Recoverable() {
		e.state = StateFault
	} else {
		e.state = StateFatal
	}
	e.fault = f
	e.lastReason = classify.Reason(f.Kind)
	return Result{State: e.state, ID: e.safeActiveID(), Fault: f}
}

// finishLocked completes the active motion (normal end or, if terminal, a
// preempt/cancel after its stop ramp).
func (e *Engine) finishLocked(reason classify.Reason) Result {
	id := arbiter.ID{}
	if e.active != nil {
		id = e.active.id
		e.arb.Complete(id)
	}
	e.active = nil
	e.stopping = nil
	e.fault = nil
	e.state = StateIdle
	e.lastReason = reason
	for i := range e.torque {
		e.torque[i] = 0
	}
	return Result{State: StateIdle, ID: id, Done: reason == classify.ReasonOK, Rejected: reason}
}

// Run steps deterministically until reaching a quiescent state (IDLE,
// FAULT or FATAL). No wall clock is used, so identical runs are reproducible.
func (e *Engine) Run() Result {
	for {
		r := e.Step()
		if r.State == StateIdle || r.State == StateFault || r.State == StateFatal {
			return r
		}
	}
}

// Recover requests recovery from a recoverable fault. It is refused unless
// ALL preconditions hold:
//  1. state is FAULT (never FATAL; that needs Reconfigure),
//  2. encoder feedback is valid again,
//  3. every axis is finite and numerically stopped (|v| <= 1e-6 rad/s),
//     i.e. the safe-stop brake has actually converged.
//
// Recovery never silently resumes the interrupted trajectory; it returns to
// IDLE and the client must issue a fresh command from the actual position.
func (e *Engine) Recover() error {
	e.mu.Lock()
	defer e.mu.Unlock()
	if e.state != StateFault {
		return classify.New(classify.ReasonResetNotSatisfied,
			"recover requires a recoverable FAULT state, current state "+string(e.state))
	}
	if !e.plant.FeedbackValid() {
		return classify.New(classify.ReasonResetNotSatisfied,
			"feedback still invalid; restore encoder before recovery")
	}
	vel := e.plant.Velocity()
	pos := e.plant.Position()
	const velEps = 1e-6
	for i := range vel {
		if !kinematics.IsFinite(pos[i], vel[i]) {
			return classify.New(classify.ReasonResetNotSatisfied,
				"non-finite axis state during recovery precondition check")
		}
		if absF(vel[i]) > velEps {
			return classify.New(classify.ReasonResetNotSatisfied,
				fmt.Sprintf("axis[%d] still moving at %v rad/s; safe stop must complete first", i, vel[i]))
		}
	}
	// Clear fault and go idle; interrupted motion is NOT auto-resumed.
	e.fault = nil
	e.state = StateIdle
	e.active = nil
	e.stopping = nil
	e.controller.Reset()
	e.lastReason = classify.ReasonOK
	return nil
}

// Reconfigure is the only exit from FATAL: it validates and installs a new
// configuration, resets plant/controller/arbiter and returns to IDLE. An
// invalid configuration is rejected and leaves all state unchanged.
func (e *Engine) Reconfigure(cfg *config.Config) error {
	e.mu.Lock()
	defer e.mu.Unlock()
	if e.state != StateFatal {
		return classify.New(classify.ReasonResetNotSatisfied,
			"reconfigure is only valid after an unrecoverable FATAL fault (state "+string(e.state)+")")
	}
	if err := cfg.Validate(); err != nil {
		return err // distinguishable reason, state untouched
	}
	n := cfg.AxisCount()
	inertia := make([]float64, n)
	gravity := make([]float64, n)
	for i, a := range cfg.Axes {
		inertia[i], gravity[i] = a.Inertia, a.GravityTorque
	}
	e.cfg = cfg
	e.planner = trajectory.NewPlanner(cfg)
	e.plant = plant.New(cfg.ControlPeriod, inertia, gravity)
	e.controller = controller.New(cfg)
	e.arb.Reset()
	e.active = nil
	e.stopping = nil
	e.fault = nil
	e.state = StateIdle
	e.cycle = 0
	e.lastAxes = make([]AxisSnapshot, n)
	e.torque = make([]float64, n)
	e.disturbance = make([]float64, n)
	e.lastReason = classify.ReasonOK
	return nil
}

// Snapshot returns a consistent deep copy of engine state taken under lock.
func (e *Engine) Snapshot() Snapshot {
	e.mu.Lock()
	defer e.mu.Unlock()
	snap := Snapshot{
		State:      e.state,
		Cycle:      e.cycle,
		Fault:      e.fault,
		LastReason: e.lastReason,
		Axes:       make([]AxisSnapshot, len(e.lastAxes)),
	}
	if e.active != nil {
		snap.ActiveID = e.active.id
	}
	copy(snap.Axes, e.lastAxes)
	return snap
}

// State returns the lifecycle state (convenience accessor).
func (e *Engine) State() State {
	e.mu.Lock()
	defer e.mu.Unlock()
	return e.state
}

// SetFeedbackValid injects encoder connection state for fault injection.
func (e *Engine) SetFeedbackValid(ok bool) {
	e.mu.Lock()
	defer e.mu.Unlock()
	e.plant.SetFeedbackValid(ok)
}

// SetDisturbance installs a constant per-axis disturbance torque (N*m).
func (e *Engine) SetDisturbance(d []float64) {
	e.mu.Lock()
	defer e.mu.Unlock()
	e.disturbance = append(make([]float64, 0, e.cfg.AxisCount()), d...)
}

// RunTicked advances one control period in a background goroutine using a
// fixed-period ticker, until Shutdown. Deterministic logic still lives in
// Step; the ticker only supplies timing for live demos.
func (e *Engine) RunTicked() {
	e.mu.Lock()
	if e.ticked {
		e.mu.Unlock()
		return
	}
	e.ticked = true
	e.mu.Unlock()
	e.wg.Add(1)
	go func() {
		defer e.wg.Done()
		ticker := newTicker(e.cfg.ControlPeriod)
		defer ticker.Stop()
		for {
			select {
			case <-e.quit:
				return
			case <-ticker.C():
				e.Step()
			}
		}
	}()
}

// Shutdown stops the background ticker and waits for it.
func (e *Engine) Shutdown() {
	e.mu.Lock()
	if !e.ticked {
		e.mu.Unlock()
		return
	}
	e.ticked = false
	e.mu.Unlock()
	close(e.quit)
	e.wg.Wait()
}

// stopVelocityEps is the speed below which a zero-torque coast is considered
// stopped.
const stopVelocityEps = 1e-6

// brakeGain is the safe-stop damping coefficient (N*m per rad/s).
const brakeGain = 50.0

// Terminal settling criteria: position within 1/10 of the tracking
// tolerance and essentially zero velocity, sustained for settleStreakCycles
// control cycles, bounded by settleBudgetSeconds.
const settleStreakCycles = 20

func settlePosTolFor(tol float64) float64 { return tol * 0.1 }
func settleVelTolFor(period float64) float64 {
	// Near-zero speed for "at target": tight enough that residual motion over
	// one cycle is far below the position tolerance, but above the discrete
	// integral-action tail. (At 5e-4 rad/s the per-cycle displacement is
	// period*5e-4, orders of magnitude below the 0.1*tolerance position test.)
	return 5e-4
}

const settleBudgetSeconds = 5.0

func absF(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}
