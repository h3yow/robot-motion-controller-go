// Package motion ties planning, simulation, closed-loop control and command
// arbitration into one local, fixed-clock motion service safe for
// concurrent callers.
//
// Execution model: a single goroutine (the cycle loop) owns the controller,
// plant and arbiter transitions; public methods send typed jobs to it and
// block on a per-job response channel. Submission, waypoint append,
// pause/resume, cancellation and queue-status queries are all serialized by
// that goroutine, so status reads can never observe a half-updated queue.
// In manual-clock mode tests drive cycles deterministically via Step.
package motion

import (
	"context"
	"fmt"
	"sync"
	"sync/atomic"
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

// DropFn reports whether feedback of an axis is dropped at a cycle.
type DropFn = dropFn

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

// Waypoint is one ordered point of a path. Exactly one of Joints or Pose
// must be set: Joints gives joint angles (radians) directly; Pose gives an
// end-effector pose resolved through inverse kinematics. Joint-space and
// Cartesian waypoints may be mixed freely within one path.
type Waypoint struct {
	Joints []float64
	Pose   *kinematics.Pose
}

// resolvedWaypoint retains the validated joint-space goal of an admitted
// waypoint. Pose waypoints are resolved exactly once, at admission, so
// execution is deterministic and independent of later plant state.
type resolvedWaypoint struct {
	source Waypoint
	goal   []float64
}

type submitReq struct {
	entrance  string
	seq       uint64
	prio      arbiter.Priority
	waypoints []Waypoint
}

type job struct {
	kind      string
	submit    *submitReq
	appendReq *appendReq
	pause     *pauseReq
	id        uint64
	respID    chan uint64
	respErr   chan error
	respOut   chan *arbiter.Outcome
	respSnap  chan []control.AxisStatus
}

type appendReq struct {
	id        uint64
	waypoints []Waypoint
}

type pauseReq struct {
	id     uint64
	resume bool
}

// pathState retains the validated waypoints of an admitted command/path.
// seg is the index of the segment currently in effect (0-based); it stays
// valid while the command is active and is advanced at each segment end.
type pathState struct {
	cmd arbiter.Command
	wps []resolvedWaypoint
	seg int
}

// Service is the concurrent-safe motion execution service.
type Service struct {
	cfg        *config.Config
	arm        *kinematics.Arm
	plant      *sim.Plant
	ctrl       *control.Controller
	arb        *arbiter.Arbiter
	opts       options
	maxPending int

	jobs    chan job
	stepReq chan chan struct{}
	done    chan struct{}
	closeMu sync.Mutex
	closed  bool
	wg      sync.WaitGroup

	idCounter uint64
	paths     map[uint64]*pathState

	// published holds the latest complete queue/backlog snapshot. The loop
	// goroutine is the only writer; Queue reads it without entering the job
	// channel, so a flood of status reads cannot starve cycle stepping and
	// readers always observe one complete, internally consistent snapshot.
	published atomic.Pointer[QueueStatus]
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
		cfg:        cfg,
		arm:        arm,
		plant:      sim.NewPlant(cfg),
		ctrl:       control.New(cfg),
		arb:        arbiter.New(),
		opts:       o,
		maxPending: cfg.EffectiveMaxPending(),
		jobs:       make(chan job, 1024),
		stepReq:    make(chan chan struct{}, 1),
		done:       make(chan struct{}),
		paths:      map[uint64]*pathState{},
	}
	home := make([]float64, len(cfg.Axes))
	for i := range cfg.Axes {
		home[i] = cfg.Axes[i].HomePosition
	}
	s.ctrl.Reset(home)
	s.plant.Reset(home)
	s.publishLocked()
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
	// For every mutating job the new snapshot is published BEFORE the
	// response is sent. Response channels are buffered, so publishing after
	// the send could let the caller wake up and read the previous snapshot;
	// publish-then-respond makes the observed ordering exactly commit order.
	switch j.kind {
	case "submit":
		id, err := s.applySubmit(j.submit)
		if err != nil {
			s.publishLocked()
			j.respErr <- err
			return
		}
		s.publishLocked()
		j.respID <- id
	case "append":
		err := s.applyAppend(j.appendReq)
		s.publishLocked()
		j.respErr <- err
	case "pause":
		err := s.applyPause(j.pause)
		s.publishLocked()
		j.respErr <- err
	case "cancel":
		err := s.applyCancel(j.id)
		s.publishLocked()
		j.respErr <- err
	case "recover":
		err := s.ctrl.Recover()
		s.publishLocked()
		j.respErr <- err
	case "outcome":
		oc, ok := s.arb.Outcome(j.id)
		if !ok {
			j.respErr <- mcerr.New(mcerr.CodeCommandUnknown, fmt.Sprintf("unknown command %d", j.id))
			return
		}
		j.respOut <- oc
	case "snapshot":
		j.respSnap <- s.ctrl.Status()
	}
}

// publishLocked snapshots the complete queue/backlog state atomically. It
// runs only on the loop goroutine after each cycle and each mutating job,
// which is exactly when the committed state changes; therefore every
// published snapshot is complete and self-consistent.
func (s *Service) publishLocked() {
	st := s.queueStatusLocked()
	s.published.Store(&st)
}

// resolveWaypointLocked validates one waypoint and returns its joint-space
// goal. The seed (current joints for the first waypoint, the previously
// resolved goal thereafter) selects the closest IK branch. Validation is
// exhaustive: cardinality, finiteness, reachability and joint travel are
// all checked at admission; no default value is ever substituted.
func (s *Service) resolveWaypointLocked(wp Waypoint, seed []float64) (resolvedWaypoint, error) {
	n := len(s.cfg.Axes)
	joints, poseSet := wp.Joints != nil, wp.Pose != nil
	if joints == poseSet {
		return resolvedWaypoint{}, mcerr.New(mcerr.CodeTargetContradictory,
			"waypoint must set exactly one of Joints or Pose")
	}
	var goal []float64
	if joints {
		if len(wp.Joints) != n {
			return resolvedWaypoint{}, mcerr.New(mcerr.CodeTargetContradictory,
				fmt.Sprintf("waypoint has %d joint angles, expected %d", len(wp.Joints), n))
		}
		if !numeric.AllFinite(wp.Joints) {
			return resolvedWaypoint{}, mcerr.New(mcerr.CodeTargetNonFinite, "waypoint joints contain NaN/Inf")
		}
		goal = append([]float64(nil), wp.Joints...)
	} else {
		p := *wp.Pose
		if !numeric.Finite(p.X) || !numeric.Finite(p.Y) || !numeric.Finite(p.Phi) {
			return resolvedWaypoint{}, mcerr.New(mcerr.CodeTargetNonFinite, "waypoint pose contains NaN/Inf")
		}
		q, err := s.arm.IK(p, seed)
		if err != nil {
			return resolvedWaypoint{}, err
		}
		goal = q
	}
	for i, g := range goal {
		a := s.cfg.Axes[i]
		if g < a.MinPosition || g > a.MaxPosition {
			return resolvedWaypoint{}, mcerr.New(mcerr.CodeJointLimitExceeded,
				fmt.Sprintf("axis %s waypoint %.6f rad outside [%.6f, %.6f]", a.Name, g, a.MinPosition, a.MaxPosition))
		}
	}
	src := Waypoint{Joints: append([]float64(nil), wp.Joints...)}
	if wp.Pose != nil {
		p := *wp.Pose
		src.Pose = &p
	}
	return resolvedWaypoint{source: src, goal: goal}, nil
}

// resolvePathLocked validates every waypoint of a submitted path. Joint and
// pose entries may be mixed; each pose is seeded by the previous resolved
// goal (the first is seeded by the measured configuration).
func (s *Service) resolvePathLocked(wps []Waypoint) ([]resolvedWaypoint, error) {
	if len(wps) == 0 {
		return nil, mcerr.New(mcerr.CodeTargetContradictory, "a path requires at least one waypoint")
	}
	q, _ := s.plant.State()
	seed := append([]float64(nil), q...)
	out := make([]resolvedWaypoint, len(wps))
	for i, wp := range wps {
		rw, err := s.resolveWaypointLocked(wp, seed)
		if err != nil {
			return nil, err
		}
		out[i] = rw
		seed = rw.goal
	}
	return out, nil
}

func (s *Service) applySubmit(req *submitReq) (uint64, error) {
	if req.entrance == "" {
		return 0, mcerr.New(mcerr.CodeTargetContradictory, "command requires a non-empty entrance")
	}
	if req.prio > arbiter.PriorityEmergency {
		return 0, mcerr.New(mcerr.CodeConfigInvalid, fmt.Sprintf("unsupported priority %d", req.prio))
	}
	// All content validation runs here, on the loop goroutine, before any
	// state is committed.
	resolved, err := s.resolvePathLocked(req.waypoints)
	if err != nil {
		return 0, err
	}
	ph, _, _ := s.ctrl.Snapshot()
	if ph == control.PhaseFaulted {
		return 0, mcerr.New(mcerr.CodeFaultActive, "cannot submit a move while a fault is active")
	}

	// Capacity: a command that becomes effective immediately (nothing
	// active, or strict preemption) does not add pending items. A command
	// that enters the FIFO queue costs its whole waypoint count.
	queues := false
	if activeID := s.arb.ActiveID(); activeID != 0 {
		if active, ok := s.arb.Command(activeID); ok {
			queues = req.prio <= active.Priority
		}
	}
	if queues {
		pending := s.pendingLocked()
		if pending+len(resolved) > s.maxPending {
			return 0, mcerr.New(mcerr.CodeQueueFull,
				fmt.Sprintf("pending backlog %d plus %d waypoint(s) exceeds limit %d",
					pending, len(resolved), s.maxPending))
		}
	}

	s.idCounter++
	id := s.idCounter
	st := &pathState{
		cmd: arbiter.Command{ID: id, Entrance: req.entrance, Seq: req.seq,
			Priority: req.prio, Goal: append([]float64(nil), resolved[0].goal...)},
		wps: resolved,
	}
	res, aerr := s.arb.Submit(&st.cmd)
	if aerr != nil {
		return 0, aerr
	}
	if res.Duplicate {
		// ID was freshly allocated, so this cannot happen; keep guard.
		return 0, mcerr.New(mcerr.CodeDuplicateCmd, "internal duplicate command")
	}
	s.paths[id] = st
	if res.State == arbiter.StateRunning {
		if err := s.startCommandLocked(id); err != nil {
			return 0, err
		}
	}
	return id, nil
}

// pendingLocked counts admitted-but-not-started motion items: every
// waypoint of every queued command, plus the not-yet-started tail of the
// active path. The currently executing segment is not "pending".
func (s *Service) pendingLocked() int {
	total := 0
	if activeID := s.arb.ActiveID(); activeID != 0 {
		if st := s.paths[activeID]; st != nil {
			if rem := len(st.wps) - st.seg - 1; rem > 0 {
				total += rem
			}
		}
	}
	for _, qid := range s.arb.QueuedIDs() {
		if st := s.paths[qid]; st != nil {
			total += len(st.wps)
		}
	}
	return total
}

// startCommandLocked plans the first segment from the CURRENT measured
// state and arms the controller. Planning at activation (not submission)
// is essential for queued/preempting commands whose activation state
// differs from their submission state. Subsequent segments are planned the
// same way when the previous one ends.
func (s *Service) startCommandLocked(id uint64) error {
	st, ok := s.paths[id]
	if !ok {
		s.arb.Finish(id, arbiter.StateRejected, "missing command record")
		return mcerr.New(mcerr.CodeCommandUnknown, fmt.Sprintf("missing command %d", id))
	}
	st.seg = 0
	tr, err := s.planJointLocked(st.wps[0].goal)
	if err != nil {
		s.arb.Finish(id, arbiter.StateRejected, err.Error())
		delete(s.paths, id)
		return err
	}
	if err := s.ctrl.LoadSegment(tr, len(st.wps) > 1); err != nil {
		s.arb.Finish(id, arbiter.StateRejected, err.Error())
		delete(s.paths, id)
		return err
	}
	return nil
}

// advanceSegmentLocked hands the controller from a finished segment to the
// next waypoint. Called once per cycle while the controller parks in
// PhaseSegmentEnd. The new segment is planned from the measured (settled)
// state, so consecutive segments meet without a jump.
func (s *Service) advanceSegmentLocked() {
	id := s.arb.ActiveID()
	st := s.paths[id]
	if st == nil {
		s.ctrl.ClearSegment()
		return
	}
	st.seg++
	if st.seg >= len(st.wps) {
		// Controller expected another segment that was never appended;
		// finish normally rather than spinning in segment_end.
		s.ctrl.ClearSegment()
		s.arb.Finish(id, arbiter.StateCompleted, "reached final waypoint")
		return
	}
	tr, err := s.planJointLocked(st.wps[st.seg].goal)
	if err != nil {
		s.ctrl.ClearSegment()
		s.arb.Finish(id, arbiter.StateRejected, err.Error())
		return
	}
	if err := s.ctrl.LoadSegment(tr, st.seg < len(st.wps)-1); err != nil {
		s.ctrl.ClearSegment()
		s.arb.Finish(id, arbiter.StateRejected, err.Error())
	}
}

func (s *Service) applyAppend(req *appendReq) error {
	st, ok := s.paths[req.id]
	if !ok {
		return mcerr.New(mcerr.CodeCommandUnknown, fmt.Sprintf("unknown command %d", req.id))
	}
	oc, exists := s.arb.Outcome(req.id)
	if !exists {
		return mcerr.New(mcerr.CodeCommandUnknown, fmt.Sprintf("unknown command %d", req.id))
	}
	if isTerminal(oc.State) {
		return mcerr.New(mcerr.CodeOperationInvalid,
			fmt.Sprintf("cannot append waypoints to command %d in terminal state %s", req.id, oc.State))
	}
	if len(req.waypoints) == 0 {
		return mcerr.New(mcerr.CodeTargetContradictory, "append requires at least one waypoint")
	}
	ph, _, _ := s.ctrl.Snapshot()
	if ph == control.PhaseFaulted {
		return mcerr.New(mcerr.CodeFaultActive, "cannot append waypoints while a fault is active")
	}
	// Capacity is reserved for the whole append before anything is added.
	pending := s.pendingLocked()
	active := s.arb.ActiveID() == req.id
	// Waypoints appended to the active path are pending immediately
	// (behind the current segment); appends to a queued path add to its
	// whole backlog.
	cost := len(req.waypoints)
	_ = active
	if pending+cost > s.maxPending {
		return mcerr.New(mcerr.CodeQueueFull,
			fmt.Sprintf("pending backlog %d plus %d appended waypoint(s) exceeds limit %d",
				pending, cost, s.maxPending))
	}

	// Resolve against the last validated goal: admission is independent of
	// the exact execution instant, so replayed inputs reproduce bit-for-bit.
	seed := append([]float64(nil), st.wps[len(st.wps)-1].goal...)
	newWps := make([]resolvedWaypoint, len(req.waypoints))
	for i, wp := range req.waypoints {
		rw, err := s.resolveWaypointLocked(wp, seed)
		if err != nil {
			return err
		}
		newWps[i] = rw
		seed = rw.goal
	}

	// Commit only after the whole batch validated and fit the budget.
	st.wps = append(st.wps, newWps...)
	if active {
		// Extend the live path. When the current segment has already
		// parked in segment_end, the flag re-arms the hand-off; when it is
		// still running (or paused), convergence becomes a hand-off.
		if err := s.ctrl.MarkMoreSegments(); err != nil {
			return mcerr.New(mcerr.CodeOperationInvalid,
				fmt.Sprintf("cannot append to active command %d: %v", req.id, err))
		}
	}
	return nil
}

func (s *Service) applyPause(req *pauseReq) error {
	if _, ok := s.paths[req.id]; !ok {
		return mcerr.New(mcerr.CodeCommandUnknown, fmt.Sprintf("unknown command %d", req.id))
	}
	if s.arb.ActiveID() != req.id {
		return mcerr.New(mcerr.CodeOperationInvalid,
			fmt.Sprintf("command %d is not the effective command; only the active command can be paused/resumed", req.id))
	}
	if req.resume {
		if err := s.ctrl.Resume(); err != nil {
			return err
		}
		// Re-plan the remaining segment from the measured settled state to
		// the same waypoint. The cycle loop sees PhaseIdle only inside this
		// job (jobs and cycles are serialized), so no queued command can be
		// promoted in between.
		st := s.paths[req.id]
		if st != nil {
			tr, perr := s.planJointLocked(st.wps[st.seg].goal)
			if perr != nil {
				s.ctrl.ClearSegment()
				s.arb.Finish(req.id, arbiter.StateRejected, perr.Error())
				return perr
			}
			if lerr := s.ctrl.LoadSegment(tr, st.seg < len(st.wps)-1); lerr != nil {
				s.ctrl.ClearSegment()
				s.arb.Finish(req.id, arbiter.StateRejected, lerr.Error())
				return lerr
			}
		}
		return nil
	}
	return s.ctrl.Pause()
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
	if wasActive {
		// Cover running, paused-holding and pause-braking phases.
		ph := s.ctrl.Phase()
		if ph == control.PhaseRunning || ph == control.PhasePaused || ph == control.PhaseStopping {
			s.ctrl.Cancel()
		}
	}
	return nil
}

// runCycle executes exactly one fixed control period.
func (s *Service) runCycle() {
	// Segment hand-off takes precedence; otherwise promote a queued
	// command only when nothing is active and no fault.
	ph, _, _ := s.ctrl.Snapshot()
	switch ph {
	case control.PhaseSegmentEnd:
		// A cancellation delivered during the one-cycle hand-off park is
		// honored here rather than advancing into the next segment.
		if id := s.arb.ActiveID(); id != 0 {
			if oc, _ := s.arb.Outcome(id); oc != nil && oc.State == arbiter.StateStopping {
				s.ctrl.ClearSegment()
				s.arb.Finish(id, arbiter.StateCanceled, "canceled at waypoint boundary")
				return
			}
		}
		s.advanceSegmentLocked()
	case control.PhaseIdle, control.PhaseDone:
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
	s.publishLocked()
}

func (s *Service) activateQueuedLocked(cmd arbiter.Command) {
	ph, _, _ := s.ctrl.Snapshot()
	if ph == control.PhaseFaulted {
		return
	}
	s.arb.SetActive(cmd.ID)
	if err := s.startCommandLocked(cmd.ID); err != nil {
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
		if phase == control.PhaseRunning || phase == control.PhasePaused || phase == control.PhaseSegmentEnd {
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

// MoveJoints submits a single joint-space move (angles in radians). It is
// equivalent to MovePath with one joint-space waypoint.
func (s *Service) MoveJoints(ctx context.Context, entrance string, seq uint64, prio arbiter.Priority, goal []float64) (uint64, error) {
	return s.MovePath(ctx, entrance, seq, prio, []Waypoint{{Joints: append([]float64(nil), goal...)}})
}

// MovePose submits a single Cartesian move; IK is validated before
// admission. It is equivalent to MovePath with one pose waypoint.
func (s *Service) MovePose(ctx context.Context, entrance string, seq uint64, prio arbiter.Priority, pose kinematics.Pose) (uint64, error) {
	p := pose
	return s.MovePath(ctx, entrance, seq, prio, []Waypoint{{Pose: &p}})
}

// MovePath submits an ordered sequence of waypoints. The path executes in
// submission order, coming to rest at every waypoint; joint-space and
// Cartesian waypoints may be mixed. The returned id identifies the whole
// path for Wait/Outcome/Cancel/Pause/AppendWaypoints.
func (s *Service) MovePath(ctx context.Context, entrance string, seq uint64, prio arbiter.Priority, wps []Waypoint) (uint64, error) {
	cp := make([]Waypoint, len(wps))
	for i, wp := range wps {
		cp[i] = Waypoint{Joints: append([]float64(nil), wp.Joints...)}
		if wp.Pose != nil {
			p := *wp.Pose
			cp[i].Pose = &p
		}
	}
	return s.sendSubmit(ctx, &submitReq{
		entrance: entrance, seq: seq, prio: prio, waypoints: cp,
	})
}

// AppendWaypoints extends an existing non-terminal path with additional
// waypoints. They execute after the currently queued tail in the given
// order; joint and pose entries may be mixed with the existing ones.
// Appending is rejected distinctly (queue_full) when the pending-command
// budget would be exceeded, and (operation_invalid) once the path ended.
func (s *Service) AppendWaypoints(ctx context.Context, id uint64, wps []Waypoint) error {
	cp := make([]Waypoint, len(wps))
	for i, wp := range wps {
		cp[i] = Waypoint{Joints: append([]float64(nil), wp.Joints...)}
		if wp.Pose != nil {
			p := *wp.Pose
			cp[i].Pose = &p
		}
	}
	ch := make(chan error, 1)
	select {
	case s.jobs <- job{kind: "append", appendReq: &appendReq{id: id, waypoints: cp}, respErr: ch}:
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

// Pause holds the currently active path at its current reference with the
// closed loop active. Queued commands keep their place and are unaffected;
// cancellation remains effective while paused.
func (s *Service) Pause(ctx context.Context, id uint64) error {
	return s.sendPause(ctx, &pauseReq{id: id, resume: false})
}

// Resume continues a paused path from the frozen reference toward the same
// final waypoint. Pause duration does not change the final result.
func (s *Service) Resume(ctx context.Context, id uint64) error {
	return s.sendPause(ctx, &pauseReq{id: id, resume: true})
}

func (s *Service) sendPause(ctx context.Context, req *pauseReq) error {
	ch := make(chan error, 1)
	select {
	case s.jobs <- job{kind: "pause", pause: req, respErr: ch}:
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

// PathProgress describes how far one path has executed.
type PathProgress struct {
	// WaypointTotal is the number of admitted waypoints in the path.
	WaypointTotal int
	// WaypointIndex is the 0-based index of the waypoint currently in
	// effect; it is -1 for a queued path and equals WaypointTotal once the
	// path has finished.
	WaypointIndex int
	// SampleIndex/SampleTotal are the consumed/total samples of the
	// trajectory of the current segment (0/0 while queued).
	SampleIndex int
	SampleTotal int
	// Goal is the resolved joint-space target of the current waypoint.
	Goal []float64
}

// QueuedCommand describes one command waiting in the FIFO queue.
type QueuedCommand struct {
	ID        uint64
	Entrance  string
	Seq       uint64
	Priority  arbiter.Priority
	Waypoints int
	// Goals are the resolved joint goals of all queued waypoints, in order.
	Goals [][]float64
}

// QueueStatus is a consistent snapshot of command backlog and progress,
// produced atomically on the cycle goroutine: concurrent submits, steps,
// cancels, appends and queries can never interleave inside it.
type QueueStatus struct {
	// MaxPending is the configured budget; Pending is the current count of
	// admitted-but-not-started motion items.
	MaxPending int
	Pending    int
	// ActiveID is the effective command (0 when idle).
	ActiveID uint64
	// Paused reports whether the active command is currently paused.
	Paused bool
	// Phase is the controller phase of the active command.
	Phase control.Phase
	// Active reports the active path's progress (zero value when idle).
	Active PathProgress
	// Queued lists the FIFO backlog in effective (priority, then arrival)
	// order.
	Queued []QueuedCommand
}

func (s *Service) queueStatusLocked() QueueStatus {
	st := QueueStatus{MaxPending: s.maxPending, Pending: s.pendingLocked()}
	ph, _, _ := s.ctrl.Snapshot()
	st.Phase = ph
	st.Paused = s.ctrl.IsPaused()
	if activeID := s.arb.ActiveID(); activeID != 0 {
		st.ActiveID = activeID
		if p := s.paths[activeID]; p != nil && p.seg < len(p.wps) {
			st.Active = PathProgress{
				WaypointTotal: len(p.wps),
				WaypointIndex: p.seg,
				Goal:          append([]float64(nil), p.wps[p.seg].goal...),
			}
			st.Active.SampleIndex, st.Active.SampleTotal = s.ctrl.Progress()
		}
	}
	for _, qid := range s.arb.QueuedIDs() {
		p := s.paths[qid]
		if p == nil {
			continue
		}
		qc := QueuedCommand{
			ID: p.cmd.ID, Entrance: p.cmd.Entrance, Seq: p.cmd.Seq,
			Priority: p.cmd.Priority, Waypoints: len(p.wps),
		}
		for _, wp := range p.wps {
			qc.Goals = append(qc.Goals, append([]float64(nil), wp.goal...))
		}
		st.Queued = append(st.Queued, qc)
	}
	return st
}

// Queue returns the latest consistent snapshot of the pending backlog, the
// active command and its progress. It is served from an atomically
// published snapshot (refreshed at every committed state change), so it
// never blocks cycle stepping and concurrent callers always read one
// complete snapshot, never a half-updated one. The context is accepted for
// API uniformity but does not cancel an in-memory read.
func (s *Service) Queue(ctx context.Context) (QueueStatus, error) {
	st := s.published.Load()
	if st == nil {
		return QueueStatus{}, mcerr.New(mcerr.CodeFaultActive, "service not initialized")
	}
	// Deep-copy so callers cannot mutate the shared published snapshot.
	out := *st
	if st.Active.Goal != nil {
		out.Active.Goal = append([]float64(nil), st.Active.Goal...)
	}
	if st.Queued != nil {
		out.Queued = make([]QueuedCommand, len(st.Queued))
		for i, q := range st.Queued {
			out.Queued[i] = q
			if q.Goals != nil {
				gs := make([][]float64, len(q.Goals))
				for j, g := range q.Goals {
					gs[j] = append([]float64(nil), g...)
				}
				out.Queued[i].Goals = gs
			}
		}
	}
	return out, nil
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
