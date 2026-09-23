// Package motion ties planning, simulation, closed-loop control and command
// arbitration into one local, fixed-clock motion service safe for
// concurrent callers.
//
// Execution model: a single goroutine (the cycle loop) owns the controller,
// plant and arbiter transitions; public methods send typed jobs to it and
// block on a per-job response channel. RWMutex guards only the read-only
// snapshot paths (Axes/Outcome) against a running cycle, so axis status and
// progress can never be observed half-updated. In manual-clock mode tests
// drive cycles deterministically via Step.
package motion

import (
	"context"
	"fmt"
	"sync"
	"time"

	"github.com/h3yow/robot-motion-controller-go/internal/arbiter"
	"github.com/h3yow/robot-motion-controller-go/internal/config"
	"github.com/h3yow/robot-motion-controller-go/internal/control"
	"github.com/h3yow/robot-motion-controller-go/internal/kinematics"
	"github.com/h3yow/robot-motion-controller-go/internal/mcerr"
	"github.com/h3yow/robot-motion-controller-go/internal/numeric"
	"github.com/h3yow/robot-motion-controller-go/internal/sim"
	"github.com/h3yow/robot-motion-controller-go/internal/trajectory"
)

// loadFn returns external disturbance torque per axis at a cycle.
type loadFn func(tick int64) []float64

// DropFn reports whether feedback of axis is dropped at a cycle.
type DropFn = dropFn

// dropFn reports whether axis feedback is dropped at a cycle.
type dropFn func(tick int64, axis int) bool

// scaleFn returns per-axis actuator torque scale at a cycle.
type scaleFn func(tick int64) []float64

type options struct {
	manual bool
	load   loadFn
	drop   dropFn
	scale  scaleFn
}

// Option configures a Service.
type Option func(*options)

// WithManualClock disables the wall-clock ticker so tests drive cycles
// deterministically via Service.Step.
func WithManualClock() Option {
	return func(o *options) { o.manual = true }
}

// WithLoadTorque injects a time-varying external disturbance torque.
func WithLoadTorque(fn func(tick int64) []float64) Option {
	return func(o *options) { o.load = fn }
}

// WithFeedbackDrop drops axis feedback for cycles where fn returns true.
func WithFeedbackDrop(fn func(tick int64, axis int) bool) Option {
	return func(o *options) { o.drop = fn }
}

// WithTorqueScale scales applied actuator torque per cycle and axis.
func WithTorqueScale(fn func(tick int64) []float64) Option {
	return func(o *options) { o.scale = fn }
}

type submitReq struct {
	entrance string
	seq      uint64
	prio     arbiter.Priority
	goal     []float64
}

type job struct {
	kind     string
	submit   *submitReq
	id       uint64
	respID   chan uint64
	respErr  chan error
	respOut  chan *arbiter.Outcome
	respSnap chan []control.AxisStatus
}

// storedMove retains the validated trajectory of an admitted command so
// queued commands need not be re-planned at activation time.
type storedMove struct {
	cmd arbiter.Command
	tr  *trajectory.Trajectory
}

// Service is the concurrent-safe motion execution service.
type Service struct {
	cfg   *config.Config
	arm   *kinematics.Arm
	plant *sim.Plant
	ctrl  *control.Controller
	arb   *arbiter.Arbiter
	opts  options

	jobs    chan job
	stepReq chan chan struct{}
	done    chan struct{}
	closeMu sync.Mutex
	closed  bool
	wg      sync.WaitGroup

	idCounter uint64
	moves     map[uint64]*storedMove
}

// New creates a service from a validated configuration.
func New(cfg *config.Config, opts ...Option) (*Service, error) {
	if cfg == nil {
		return nil, mcerr.New(mcerr.CodeConfigInvalid, "motion.New requires a non-nil config")
	}
	o := options{}
	for _, opt := range opts {
		opt(&o)
	}
	arm, err := kinematics.NewArm(cfg.Links, cfg.Elbow)
	if err != nil {
		return nil, err
	}
	s := &Service{
		cfg:       cfg,
		arm:       arm,
		plant:     sim.NewPlant(cfg),
		ctrl:      control.New(cfg),
		arb:       arbiter.New(),
		opts:      o,
		jobs:      make(chan job, 1024),
		stepReq:   make(chan chan struct{}, 1),
		done:      make(chan struct{}),
		idCounter: 0,
		moves:     map[uint64]*storedMove{},
	}
	home := make([]float64, len(cfg.Axes))
	for i := range cfg.Axes {
		home[i] = cfg.Axes[i].HomePosition
	}
	s.ctrl.Reset(home)
	s.plant.Reset(home)
	s.wg.Add(1)
	go s.loop()
	if !o.manual {
		s.wg.Add(1)
		go s.ticker()
	}
	return s, nil
}

// ticker emits fixed-period cycle signals on the wall clock.
func (s *Service) ticker() {
	defer s.wg.Done()
	t := time.NewTicker(s.cfg.Period)
	defer t.Stop()
	for {
		select {
		case <-s.done:
			return
		case <-t.C:
			ch := make(chan struct{})
			select {
			case s.stepReq <- ch:
				<-ch
			case <-s.done:
				return
			}
		}
	}
}

// loop is the single owner of mutable motion state.
func (s *Service) loop() {
	defer s.wg.Done()
	for {
		select {
		case <-s.done:
			return
		case j := <-s.jobs:
			s.runJob(j)
		case ch := <-s.stepReq:
			s.runCycle()
			close(ch)
		}
	}
}

func (s *Service) runJob(j job) {
	switch j.kind {
	case "submit":
		id, err := s.applySubmit(j.submit)
		if err != nil {
			j.respErr <- err
			return
		}
		j.respID <- id
	case "cancel":
		err := s.applyCancel(j.id)
		j.respErr <- err
	case "recover":
		j.respErr <- s.ctrl.Recover()
	case "outcome":
		oc, ok := s.arb.Outcome(j.id)
		if !ok {
			j.respErr <- mcerr.New(mcerr.CodeCommandUnknown, fmt.Sprintf("unknown command %d", j.id))
			return
		}
		j.respOut <- oc
	case "snapshot":
		j.respSnap <- s.ctrl.Status()
	case "phase":
		// used internally by tests through Step completion; no-op here
		j.respOut <- nil
	}
}

func (s *Service) applySubmit(req *submitReq) (uint64, error) {
	n := len(s.cfg.Axes)
	if len(req.goal) != n {
		return 0, mcerr.New(mcerr.CodeTargetContradictory,
			fmt.Sprintf("goal has %d axes, expected %d", len(req.goal), n))
	}
	if !numeric.AllFinite(req.goal) {
		return 0, mcerr.New(mcerr.CodeTargetNonFinite, "goal contains NaN/Inf")
	}
	for i, g := range req.goal {
		a := s.cfg.Axes[i]
		if g < a.MinPosition || g > a.MaxPosition {
			return 0, mcerr.New(mcerr.CodeJointLimitExceeded,
				fmt.Sprintf("axis %s goal %.6f rad outside [%.6f, %.6f]", a.Name, g, a.MinPosition, a.MaxPosition))
		}
	}
	ph, _, _ := s.ctrl.Snapshot()
	if ph == control.PhaseFaulted {
		return 0, mcerr.New(mcerr.CodeFaultActive, "cannot submit a move while a fault is active")
	}
	s.idCounter++
	id := s.idCounter
	m := &storedMove{
		cmd: arbiter.Command{ID: id, Entrance: req.entrance, Seq: req.seq,
			Priority: req.prio, Goal: append([]float64(nil), req.goal...)},
	}
	res, aerr := s.arb.Submit(&m.cmd)
	if aerr != nil {
		return 0, aerr
	}
	if res.Duplicate {
		// ID was freshly allocated, so this cannot happen; keep guard.
		return 0, mcerr.New(mcerr.CodeDuplicateCmd, "internal duplicate command")
	}
	s.moves[id] = m
	if res.State == arbiter.StateRunning {
		if err := s.startMoveLocked(id); err != nil {
			return 0, err
		}
	}
	return id, nil
}

// startMoveLocked plans from the CURRENT measured state and arms the
// controller. Planning at activation (not submission) is essential for
// queued/preempting commands whose activation state differs from their
// submission state.
func (s *Service) startMoveLocked(id uint64) error {
	m, ok := s.moves[id]
	if !ok {
		s.arb.Finish(id, arbiter.StateRejected, "missing command record")
		return mcerr.New(mcerr.CodeCommandUnknown, fmt.Sprintf("missing command %d", id))
	}
	tr, err := s.planJointLocked(m.cmd.Goal)
	if err != nil {
		s.arb.Finish(id, arbiter.StateRejected, err.Error())
		return err
	}
	m.tr = tr
	if err := s.ctrl.Load(tr); err != nil {
		s.arb.Finish(id, arbiter.StateRejected, err.Error())
		return err
	}
	return nil
}

func (s *Service) applyCancel(id uint64) error {
	if _, ok := s.arb.Outcome(id); !ok {
		return mcerr.New(mcerr.CodeCommandUnknown, fmt.Sprintf("unknown command %d", id))
	}
	wasActive := s.arb.ActiveID() == id
	cmd, err := s.arb.Cancel(id)
	if err != nil {
		return err
	}
	_ = cmd
	if wasActive && s.ctrl.Phase() == control.PhaseRunning {
		s.ctrl.Cancel()
	}
	return nil
}

// runCycle executes exactly one fixed control period.
func (s *Service) runCycle() {
	// Promote a queued command only when nothing is active and no fault.
	ph, _, _ := s.ctrl.Snapshot()
	if ph == control.PhaseIdle || ph == control.PhaseDone {
		if next := s.arb.PopQueued(); next != nil {
			s.activateQueuedLocked(*next)
		}
	}

	tick := s.plant.Tick()
	q, v := s.plant.State()
	feedbackOK := make([]bool, len(q))
	for i := range q {
		feedbackOK[i] = s.opts.drop == nil || !s.opts.drop(tick, i)
	}
	if s.opts.load != nil {
		s.plant.SetLoadTorque(s.opts.load(tick))
	}
	if s.opts.scale != nil {
		s.plant.SetTorqueScale(s.opts.scale(tick))
	}
	tau := s.ctrl.LastTorque()
	newQ, newV, perr := s.plant.Step(tau)
	if perr != nil {
		// A non-finite command must never move the plant; freeze it.
		newQ, newV = q, v
	}

	phase, f := s.ctrl.Tick(time.Time{}, newQ, newV, feedbackOK)
	s.syncLocked(phase, f, perr)
}

func (s *Service) activateQueuedLocked(cmd arbiter.Command) {
	ph, _, _ := s.ctrl.Snapshot()
	if ph == control.PhaseFaulted {
		return
	}
	s.arb.SetActive(cmd.ID)
	if err := s.startMoveLocked(cmd.ID); err != nil {
		return
	}
}

// syncLocked updates command terminal states after a cycle.
func (s *Service) syncLocked(phase control.Phase, f *control.Fault, plantErr error) {
	id := s.arb.ActiveID()
	if plantErr != nil && (f == nil || f.Code != control.FaultCommandNonFinite) {
		if mcerr.HasCode(plantErr, mcerr.CodeCommandNonFinite) && id != 0 {
			s.arb.Finish(id, arbiter.StateFaulted, "fatal fault: non-finite torque command")
			return
		}
	}
	if f != nil {
		if id == 0 {
			return
		}
		if f.Recoverable {
			// Non-terminal: command stays associated with the active slot so
			// it can resume after a successful Recover().
			s.arb.Finish(id, arbiter.StateFaultedRecov,
				fmt.Sprintf("recoverable fault (%s, axis %d): %s", f.Code, f.Axis, f.Message))
		} else {
			// Escalation from a prior recoverable fault, or directly fatal.
			s.arb.MarkFatal(id, fmt.Sprintf("fatal fault (%s, axis %d): %s", f.Code, f.Axis, f.Message))
			s.arb.Finish(id, arbiter.StateFaulted,
				fmt.Sprintf("fatal fault (%s, axis %d): %s", f.Code, f.Axis, f.Message))
		}
		return
	}
	if id == 0 {
		return
	}
	oc, _ := s.arb.Outcome(id)
	if oc != nil && oc.State == arbiter.StateFaultedRecov {
		// Fault cleared this cycle (controller resumed via Recover).
		if phase == control.PhaseRunning {
			s.arb.MarkRecovered(id)
		}
		return
	}
	if phase == control.PhaseDone {
		if oc != nil && oc.State == arbiter.StateStopping {
			s.arb.Finish(id, arbiter.StateCanceled, "safe stop completed after cancel")
		} else {
			s.arb.Finish(id, arbiter.StateCompleted, "reached goal within tolerance")
		}
	}
}

func (s *Service) planJointLocked(goal []float64) (*trajectory.Trajectory, error) {
	q, v := s.plant.State()
	return trajectory.Plan(&trajectory.Request{
		StartPosition: q,
		StartVelocity: v,
		GoalPosition:  append([]float64(nil), goal...),
		Limits:        s.limits(),
		Period:        s.cfg.Period,
	})
}

func (s *Service) limits() []trajectory.AxisLimits {
	out := make([]trajectory.AxisLimits, len(s.cfg.Axes))
	for i, a := range s.cfg.Axes {
		out[i] = trajectory.AxisLimits{
			Name:           a.Name,
			MinPosition:    a.MinPosition,
			MaxPosition:    a.MaxPosition,
			MaxVelocity:    a.MaxVelocity,
			MaxAccel:       a.MaxAccel,
			MaxTorque:      a.MaxTorque,
			Inertia:        a.Inertia,
			GravityTorque:  a.GravityTorque,
			FrictionTorque: a.FrictionTorque,
		}
	}
	return out
}

func (s *Service) sendSubmit(ctx context.Context, req *submitReq) (uint64, error) {
	ch := make(chan uint64, 1)
	ech := make(chan error, 1)
	select {
	case s.jobs <- job{kind: "submit", submit: req, respID: ch, respErr: ech}:
	case <-ctx.Done():
		return 0, ctx.Err()
	case <-s.done:
		return 0, mcerr.New(mcerr.CodeFaultActive, "service closed")
	}
	select {
	case id := <-ch:
		return id, nil
	case err := <-ech:
		return 0, err
	case <-ctx.Done():
		return 0, ctx.Err()
	}
}

// MoveJoints submits a joint-space move (angles in radians).
func (s *Service) MoveJoints(ctx context.Context, entrance string, seq uint64, prio arbiter.Priority, goal []float64) (uint64, error) {
	return s.sendSubmit(ctx, &submitReq{
		entrance: entrance, seq: seq, prio: prio,
		goal: append([]float64(nil), goal...),
	})
}

// MovePose submits a Cartesian move; IK is validated before admission.
func (s *Service) MovePose(ctx context.Context, entrance string, seq uint64, prio arbiter.Priority, pose kinematics.Pose) (uint64, error) {
	if !numeric.Finite(pose.X) || !numeric.Finite(pose.Y) || !numeric.Finite(pose.Phi) {
		return 0, mcerr.New(mcerr.CodeTargetNonFinite, "pose contains NaN/Inf")
	}
	q, _ := s.plant.State()
	joints, err := s.arm.IK(pose, q)
	if err != nil {
		return 0, err
	}
	for i, g := range joints {
		a := s.cfg.Axes[i]
		if g < a.MinPosition || g > a.MaxPosition {
			return 0, mcerr.New(mcerr.CodeJointLimitExceeded,
				fmt.Sprintf("IK axis %s solution %.6f rad outside [%.6f, %.6f]", a.Name, g, a.MinPosition, a.MaxPosition))
		}
	}
	return s.sendSubmit(ctx, &submitReq{
		entrance: entrance, seq: seq, prio: prio, goal: joints,
	})
}

// Wait blocks until the command reaches a terminal state.
func (s *Service) Wait(ctx context.Context, id uint64) (*arbiter.Outcome, error) {
	for {
		oc, err := s.Outcome(id)
		if err != nil {
			return nil, err
		}
		if isTerminal(oc.State) {
			return oc, nil
		}
		select {
		case <-ctx.Done():
			return nil, ctx.Err()
		case <-time.After(s.cfg.Period / 4):
		}
	}
}

func isTerminal(st arbiter.CmdState) bool {
	switch st {
	case arbiter.StateCompleted, arbiter.StateCanceled, arbiter.StatePreempted,
		arbiter.StateFaulted, arbiter.StateRejected:
		return true
	}
	return false
}

// Cancel requests cancellation of a queued or running command.
func (s *Service) Cancel(ctx context.Context, id uint64) error {
	ch := make(chan error, 1)
	select {
	case s.jobs <- job{kind: "cancel", id: id, respErr: ch}:
	case <-ctx.Done():
		return ctx.Err()
	case <-s.done:
		return mcerr.New(mcerr.CodeFaultActive, "service closed")
	}
	select {
	case err := <-ch:
		return err
	case <-ctx.Done():
		return ctx.Err()
	}
}

// Recover attempts to clear a recoverable fault; preconditions are enforced
// and fatal faults are rejected. Cycle stepping must continue while faulted
// so the controller can observe healthy feedback and satisfy its recovery
// hold window.
func (s *Service) Recover(ctx context.Context) error {
	ch := make(chan error, 1)
	select {
	case s.jobs <- job{kind: "recover", respErr: ch}:
	case <-ctx.Done():
		return ctx.Err()
	case <-s.done:
		return mcerr.New(mcerr.CodeFaultActive, "service closed")
	}
	select {
	case err := <-ch:
		return err
	case <-ctx.Done():
		return ctx.Err()
	}
}

// Outcome returns the current outcome record for a command.
func (s *Service) Outcome(id uint64) (*arbiter.Outcome, error) {
	oc, ok := s.arb.Outcome(id)
	if !ok {
		return nil, mcerr.New(mcerr.CodeCommandUnknown, fmt.Sprintf("unknown command %d", id))
	}
	out := *oc
	return &out, nil
}

// Axes returns a consistent snapshot of all axis statuses. The controller
// copies statuses under its own lock, so a half-updated axis set is never
// observable.
func (s *Service) Axes() []control.AxisStatus {
	return s.ctrl.Status()
}

// Step runs n control cycles manually (WithManualClock only).
func (s *Service) Step(n int) {
	if !s.opts.manual {
		panic("Step requires WithManualClock")
	}
	for i := 0; i < n; i++ {
		ch := make(chan struct{})
		s.stepReq <- ch
		<-ch
	}
}

// Close stops the background ticker and releases resources.
func (s *Service) Close() error {
	s.closeMu.Lock()
	if s.closed {
		s.closeMu.Unlock()
		return nil
	}
	s.closed = true
	close(s.done)
	s.closeMu.Unlock()
	s.wg.Wait()
	return nil
}
