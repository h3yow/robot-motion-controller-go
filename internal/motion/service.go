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
	// pose interprets goal as (x, y, phi) and resolves it with IK on the
	// cycle loop instead of treating it as joint angles.
	pose bool
}

// WaypointKind selects how one path point is interpreted.
type WaypointKind uint8

const (
	// WaypointJoint is a joint-angle target (radians).
	WaypointJoint WaypointKind = iota
	// WaypointPose is a Cartesian end-effector target, resolved with IK at
	// admission time (joints and poses may be mixed in one path).
	WaypointPose
)

// Waypoint is one point of a multi-point path. Exactly one of Joints/Pose is
// meaningful according to Kind.
type Waypoint struct {
	Kind   WaypointKind
	Joints []float64
	Pose   kinematics.Pose
}

type pathReq struct {
	entrance string
	seq      uint64
	prio     arbiter.Priority
	points   []Waypoint
}

type appendReq struct {
	id       uint64
	entrance string
	seq      uint64
	points   []Waypoint
}

// PathInfo summarizes one admitted multi-point path.
type PathInfo struct {
	ID            uint64
	TotalPoints   int // waypoints admitted so far
	ReachedPoints int // waypoints already reached (valid for active/done)
}

// Progress reports where the currently effective command has advanced to.
type Progress struct {
	Index int
	Total int
}

// QueueStatus is a single-consistent-snapshot view of the pending command set.
type QueueStatus struct {
	Capacity       int
	Pending        int      // active (0/1) + queued
	ActiveID       uint64   // 0 when idle
	Paused         bool     // execution is held (ramp settled or ramping to it)
	QueuedIDs      []uint64 // effective execution order
	ActiveProgress Progress // controller samples of the running leg/command
	ActivePath     *PathInfo
}

type job struct {
	kind      string
	submit    *submitReq
	path      *pathReq
	appended  *appendReq
	id        uint64
	respID    chan uint64
	respErr   chan error
	respOut   chan *arbiter.Outcome
	respSnap  chan []control.AxisStatus
	respPath  chan *PathInfo
	respQueue chan QueueStatus
	respBool  chan bool
}

// storedMove retains the validated trajectory of an admitted command so
// queued commands need not be re-planned at activation time. For multi-point
// paths it additionally keeps every validated joint target and how many have
// already been reached.
type storedMove struct {
	cmd arbiter.Command
	tr  *trajectory.Trajectory

	isPath  bool
	points  [][]float64 // validated joint targets in admission order
	reached int         // number of path points already reached
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

	// paused is the service-wide hold latch: when true, any command that
	// becomes active (including a preempting one) is ramped to rest until
	// Resume. It is owned by the cycle loop only.
	paused bool
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
		arb:       arbiter.NewWithCapacity(cfg.QueueCapacity()),
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
	case "path":
		id, info, err := s.applyPath(j.path)
		if err != nil {
			j.respErr <- err
			return
		}
		j.respID <- id
		j.respPath <- info
	case "append":
		info, err := s.applyAppend(j.appended)
		if err != nil {
			j.respErr <- err
			return
		}
		j.respPath <- info
	case "pause":
		j.respBool <- s.applyPause()
	case "resume":
		j.respErr <- s.applyResume()
	case "cancel":
		err := s.applyCancel(j.id)
		j.respErr <- err
	case "recover":
		err := s.ctrl.Recover()
		if err == nil && s.paused {
			// Keep the service-wide hold after a fault cleared during pause.
			s.ctrl.Pause()
		}
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
	case "queue":
		j.respQueue <- s.queueStatusLocked()
	}
}

func (s *Service) applySubmit(req *submitReq) (uint64, error) {
	n := len(s.cfg.Axes)
	goal := req.goal
	if req.pose {
		// Cartesian single target: resolve on the loop from measured state so
		// the seed is deterministic and never races the cycle writer.
		if len(goal) != n {
			return 0, mcerr.New(mcerr.CodeTargetContradictory,
				fmt.Sprintf("pose has %d components, expected %d", len(goal), n))
		}
		if !numeric.AllFinite(goal) {
			return 0, mcerr.New(mcerr.CodeTargetNonFinite, "pose contains NaN/Inf")
		}
		q, _ := s.plant.State()
		joints, err := s.arm.IK(kinematics.Pose{X: goal[0], Y: goal[1], Phi: goal[2]}, q)
		if err != nil {
			return 0, err
		}
		if err := s.checkJointLimits(joints); err != nil {
			return 0, err
		}
		goal = joints
	}
	if len(goal) != n {
		return 0, mcerr.New(mcerr.CodeTargetContradictory,
			fmt.Sprintf("goal has %d axes, expected %d", len(goal), n))
	}
	if !numeric.AllFinite(goal) {
		return 0, mcerr.New(mcerr.CodeTargetNonFinite, "goal contains NaN/Inf")
	}
	if err := s.checkJointLimits(goal); err != nil {
		return 0, err
	}
	ph, _, _ := s.ctrl.Snapshot()
	if ph == control.PhaseFaulted {
		return 0, mcerr.New(mcerr.CodeFaultActive, "cannot submit a move while a fault is active")
	}
	s.idCounter++
	id := s.idCounter
	m := &storedMove{
		cmd: arbiter.Command{ID: id, Entrance: req.entrance, Seq: req.seq,
			Priority: req.prio, Goal: append([]float64(nil), goal...)},
	}
	res, aerr := s.arb.Submit(&m.cmd)
	if aerr != nil {
		s.idCounter--
		return 0, aerr
	}
	if res.Duplicate {
		s.idCounter--
		// ID was freshly allocated, so this cannot happen; keep guard.
		return 0, mcerr.New(mcerr.CodeDuplicateCmd, "internal duplicate command")
	}
	s.moves[id] = m
	if res.State == arbiter.StateRunning {
		if err := s.startMoveLocked(id, true); err != nil {
			return 0, err
		}
	}
	return id, nil
}

// checkJointLimits rejects a joint target outside configured travel.
func (s *Service) checkJointLimits(goal []float64) error {
	for i, g := range goal {
		a := s.cfg.Axes[i]
		if g < a.MinPosition || g > a.MaxPosition {
			return mcerr.New(mcerr.CodeJointLimitExceeded,
				fmt.Sprintf("axis %s goal %.6f rad outside [%.6f, %.6f]", a.Name, g, a.MinPosition, a.MaxPosition))
		}
	}
	return nil
}

// resolveWaypoint validates one path point and returns its joint target.
// Joint points are checked for cardinality, finiteness and travel directly;
// pose points are solved with IK against seed (the current measured state)
// and then travel-checked. Mixed kinds are allowed.
func (s *Service) resolveWaypoint(wp Waypoint, seed []float64, at int) ([]float64, error) {
	n := len(s.cfg.Axes)
	switch wp.Kind {
	case WaypointJoint:
		if len(wp.Joints) != n {
			return nil, mcerr.New(mcerr.CodeTargetContradictory,
				fmt.Sprintf("waypoint %d has %d axes, expected %d", at, len(wp.Joints), n))
		}
		if !numeric.AllFinite(wp.Joints) {
			return nil, mcerr.New(mcerr.CodeTargetNonFinite,
				fmt.Sprintf("waypoint %d contains NaN/Inf", at))
		}
		if err := s.checkJointLimits(wp.Joints); err != nil {
			return nil, err
		}
		return append([]float64(nil), wp.Joints...), nil
	case WaypointPose:
		if !numeric.Finite(wp.Pose.X) || !numeric.Finite(wp.Pose.Y) || !numeric.Finite(wp.Pose.Phi) {
			return nil, mcerr.New(mcerr.CodeTargetNonFinite,
				fmt.Sprintf("waypoint %d pose contains NaN/Inf", at))
		}
		joints, err := s.arm.IK(wp.Pose, seed)
		if err != nil {
			return nil, err
		}
		if err := s.checkJointLimits(joints); err != nil {
			return nil, mcerr.New(mcerr.CodeJointLimitExceeded,
				fmt.Sprintf("waypoint %d: %v", at, err.Error()))
		}
		return joints, nil
	default:
		return nil, mcerr.New(mcerr.CodeTargetContradictory,
			fmt.Sprintf("waypoint %d has unknown kind %d", at, wp.Kind))
	}
}

// resolvePoints validates a batch of points in order; each pose is seeded at
// the previous resolved point (the first from seed), so mixed joint/pose
// paths resolve deterministically and continuously.
func (s *Service) resolvePoints(points []Waypoint, seed []float64) ([][]float64, error) {
	if len(points) == 0 {
		return nil, mcerr.New(mcerr.CodeTargetContradictory, "path requires at least one waypoint")
	}
	out := make([][]float64, len(points))
	cur := append([]float64(nil), seed...)
	for i, wp := range points {
		joints, err := s.resolveWaypoint(wp, cur, i)
		if err != nil {
			return nil, err
		}
		out[i] = joints
		cur = joints
	}
	return out, nil
}

func (s *Service) applyPath(req *pathReq) (uint64, *PathInfo, error) {
	ph, _, _ := s.ctrl.Snapshot()
	if ph == control.PhaseFaulted {
		return 0, nil, mcerr.New(mcerr.CodeFaultActive, "cannot submit a path while a fault is active")
	}
	seed, _ := s.plant.State()
	points, err := s.resolvePoints(req.points, seed)
	if err != nil {
		return 0, nil, err
	}
	s.idCounter++
	id := s.idCounter
	m := &storedMove{
		cmd: arbiter.Command{ID: id, Entrance: req.entrance, Seq: req.seq,
			Priority: req.prio, Goal: append([]float64(nil), points[len(points)-1]...)},
		isPath: true,
		points: points,
	}
	res, aerr := s.arb.Submit(&m.cmd)
	if aerr != nil {
		s.idCounter--
		return 0, nil, aerr
	}
	if res.Duplicate {
		s.idCounter--
		return 0, nil, mcerr.New(mcerr.CodeDuplicateCmd, "internal duplicate command")
	}
	s.moves[id] = m
	if res.State == arbiter.StateRunning {
		if err := s.startMoveLocked(id, true); err != nil {
			return 0, nil, err
		}
	}
	return id, m.pathInfoLocked(), nil
}

func (s *Service) applyAppend(req *appendReq) (*PathInfo, error) {
	m, ok := s.moves[req.id]
	if !ok {
		return nil, mcerr.New(mcerr.CodeCommandUnknown, fmt.Sprintf("unknown path command %d", req.id))
	}
	if !m.isPath {
		return nil, mcerr.New(mcerr.CodeTargetContradictory,
			fmt.Sprintf("command %d is a single-target move, not a path", req.id))
	}
	oc, exists := s.arb.Outcome(req.id)
	if !exists {
		return nil, mcerr.New(mcerr.CodeCommandUnknown, fmt.Sprintf("unknown path command %d", req.id))
	}
	// Appending is allowed while the path is queued, running (incl. stopping
	// after cancel) or recoverably faulted; a terminal path cannot gain new
	// points. A path mid-cancel stop is already committed to terminal.
	if isTerminal(oc.State) || oc.State == arbiter.StateStopping {
		return nil, mcerr.New(mcerr.CodePathClosed,
			fmt.Sprintf("cannot append to path %d in state %s: it has already finished", req.id, oc.State))
	}
	if err := s.arb.CheckSeq(req.entrance, req.seq); err != nil {
		return nil, err
	}
	// Seed poses at the current tail (already validated), never at measured
	// state: an appended pose must be consistent with the path as planned.
	seed := m.points[len(m.points)-1]
	points, err := s.resolvePoints(req.points, seed)
	if err != nil {
		return nil, err
	}
	m.points = append(m.points, points...)
	m.cmd.Goal = append([]float64(nil), m.points[len(m.points)-1]...)
	return m.pathInfoLocked(), nil
}

func (m *storedMove) pathInfoLocked() *PathInfo {
	return &PathInfo{
		ID:            m.cmd.ID,
		TotalPoints:   len(m.points),
		ReachedPoints: m.reached,
	}
}

// applyPause latches the service-wide hold and ramps the active move to rest
// if it is currently moving. A command already ramping down because of a
// cancel cannot be paused.
func (s *Service) applyPause() bool {
	if id := s.arb.ActiveID(); id != 0 {
		if oc, ok := s.arb.Outcome(id); ok && oc.State == arbiter.StateStopping {
			return false
		}
	}
	s.paused = true
	return s.ctrl.Pause()
}

// applyResume clears the hold latch and resumes from the measured state. The
// continuation (remaining path points or the single goal) is re-planned from
// the measured state, so the pause duration cannot change the final result.
func (s *Service) applyResume() error {
	ph := s.ctrl.Phase()
	switch ph {
	case control.PhasePaused:
		if !s.ctrl.Resume() {
			return mcerr.New(mcerr.CodeTargetContradictory, "controller refused resume")
		}
	case control.PhaseStopping:
		// Pause ramp still in progress: just leave the latch; the stop will
		// settle into the paused hold, so resume has to wait for settlement.
		return mcerr.New(mcerr.CodeTargetContradictory, "pause ramp has not settled yet")
	default:
		return mcerr.New(mcerr.CodeTargetContradictory, "no paused motion to resume")
	}
	id := s.arb.ActiveID()
	if id != 0 {
		// Replanning happens while the latch is still set; only release it once
		// the continuation armed successfully, so a failed replan keeps the
		// machine safely paused. honorPause=false because this arming IS the
		// explicit resume.
		if err := s.startMoveLocked(id, false); err != nil {
			return err
		}
	}
	s.paused = false
	return nil
}

// startMoveLocked plans the command's CURRENT leg from the CURRENT measured
// state and arms the controller. Planning at activation (not submission) is
// essential for queued/preempting commands whose activation state differs
// from their submission state; the same property makes pause/resume exact
// (the continuation is replanned from the measured stop point). For paths
// exactly ONE leg is planned — from the measured state to the next unreached
// waypoint — so each controller completion marks exactly one reached point
// and the axes never turn back toward an already visited one.
func (s *Service) startMoveLocked(id uint64, honorPause bool) error {
	m, ok := s.moves[id]
	if !ok {
		s.arb.Finish(id, arbiter.StateRejected, "missing command record")
		return mcerr.New(mcerr.CodeCommandUnknown, fmt.Sprintf("missing command %d", id))
	}
	var tr *trajectory.Trajectory
	var err error
	if m.isPath {
		if m.reached >= len(m.points) {
			s.arb.Finish(id, arbiter.StateRejected, "path has no unreached waypoint left")
			return mcerr.New(mcerr.CodeTargetContradictory, fmt.Sprintf("path %d has no unreached waypoint left", id))
		}
		tr, err = s.planTailLocked(m.points[m.reached : m.reached+1])
	} else {
		tr, err = s.planJointLocked(m.cmd.Goal)
	}
	if err != nil {
		s.arb.Finish(id, arbiter.StateRejected, err.Error())
		return err
	}
	m.tr = tr
	if err := s.ctrl.Load(tr); err != nil {
		s.arb.Finish(id, arbiter.StateRejected, err.Error())
		return err
	}
	// A command that activates (queued promotion, preemption replacement,
	// path-leg continuation) while the hold latch is set must not move: ramp
	// it to rest immediately and hold until Resume. Resume itself replans
	// with honorPause=false so the continuation actually runs.
	if honorPause && s.paused && s.ctrl.Phase() == control.PhaseRunning {
		s.ctrl.Pause()
	}
	return nil
}

// planTailLocked plans one rest-to-rest leg through goals (a single waypoint
// for path execution) from the current measured state.
func (s *Service) planTailLocked(goals [][]float64) (*trajectory.Trajectory, error) {
	q, v := s.plant.State()
	return trajectory.PlanPath(&trajectory.PathRequest{
		StartPosition: q,
		StartVelocity: v,
		Waypoints:     goals,
		Limits:        s.limits(),
		Period:        s.cfg.Period,
	})
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
		ph := s.ctrl.Phase()
		switch ph {
		case control.PhaseRunning:
			s.ctrl.Cancel()
		case control.PhasePaused:
			// Cancel takes precedence over the pause hold and releases it so
			// queued successors are not parked forever.
			s.ctrl.CancelHold()
			s.paused = false
		case control.PhaseStopping:
			// A pause ramp still settling becomes a cancel stop.
			if s.ctrl.AbortPauseRamp() {
				s.paused = false
			}
		}
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

	// A path reaches one waypoint each time its current leg completes while
	// it is still running (a canceled/stopping leg completing is handled by
	// syncLocked below). Advance immediately so queued commands cannot
	// interleave between path points.
	id := s.arb.ActiveID()
	if phase == control.PhaseDone && id != 0 {
		if m, ok := s.moves[id]; ok && m.isPath {
			oc, _ := s.arb.Outcome(id)
			if oc != nil && oc.State != arbiter.StateStopping {
				m.reached++
				if m.reached < len(m.points) {
					if err := s.startMoveLocked(id, true); err == nil {
						phase = control.PhaseRunning
					}
					// On planning failure the active command has just been
					// finished as rejected by startMoveLocked; fall through to
					// syncLocked with the done phase (it ignores unknown/
					// terminal ids).
				}
			}
		}
	}

	s.syncLocked(phase, f, perr)
}

func (s *Service) activateQueuedLocked(cmd arbiter.Command) {
	ph, _, _ := s.ctrl.Snapshot()
	if ph == control.PhaseFaulted {
		return
	}
	s.arb.SetActive(cmd.ID)
	if err := s.startMoveLocked(cmd.ID, true); err != nil {
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

// queueStatusLocked assembles the pending-set snapshot on the cycle loop, so
// capacity/count/active/progress always describe one and the same instant.
func (s *Service) queueStatusLocked() QueueStatus {
	active, queued := s.arb.PendingCount()
	ids := s.arb.QueuedIDs()
	st := QueueStatus{
		Capacity:  s.cfg.QueueCapacity(),
		Pending:   active + queued,
		ActiveID:  s.arb.ActiveID(),
		Paused:    s.paused,
		QueuedIDs: ids,
	}
	idx, total := s.ctrl.Progress()
	st.ActiveProgress = Progress{Index: idx, Total: total}
	if st.ActiveID != 0 {
		if m, ok := s.moves[st.ActiveID]; ok && m.isPath {
			st.ActivePath = m.pathInfoLocked()
		}
	}
	return st
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
	j := job{kind: "submit", submit: req,
		respID: make(chan uint64, 1), respErr: make(chan error, 1)}
	select {
	case s.jobs <- j:
	case <-ctx.Done():
		return 0, ctx.Err()
	case <-s.done:
		return 0, mcerr.New(mcerr.CodeFaultActive, "service closed")
	}
	select {
	case err := <-j.respErr:
		return 0, err
	case id := <-j.respID:
		return id, nil
	case <-ctx.Done():
		return 0, ctx.Err()
	case <-s.done:
		return 0, mcerr.New(mcerr.CodeFaultActive, "service closed")
	}
}

// sendJob enqueues an error-returning job and waits for its response. Every
// job answers its error channel exactly once with a buffered send, so the
// cycle loop never blocks on a caller that gave up.
func (s *Service) sendJob(ctx context.Context, j job) error {
	select {
	case s.jobs <- j:
	case <-ctx.Done():
		return ctx.Err()
	case <-s.done:
		return mcerr.New(mcerr.CodeFaultActive, "service closed")
	}
	select {
	case err := <-j.respErr:
		return err
	case <-ctx.Done():
		return ctx.Err()
	case <-s.done:
		return mcerr.New(mcerr.CodeFaultActive, "service closed")
	}
}

// MoveJoints submits a joint-space move (angles in radians).
func (s *Service) MoveJoints(ctx context.Context, entrance string, seq uint64, prio arbiter.Priority, goal []float64) (uint64, error) {
	return s.sendSubmit(ctx, &submitReq{
		entrance: entrance, seq: seq, prio: prio,
		goal: append([]float64(nil), goal...),
	})
}

// MovePose submits a Cartesian move; IK is resolved and validated on the
// cycle loop from the measured state (deterministic and race-free).
func (s *Service) MovePose(ctx context.Context, entrance string, seq uint64, prio arbiter.Priority, pose kinematics.Pose) (uint64, error) {
	if !numeric.Finite(pose.X) || !numeric.Finite(pose.Y) || !numeric.Finite(pose.Phi) {
		return 0, mcerr.New(mcerr.CodeTargetNonFinite, "pose contains NaN/Inf")
	}
	return s.sendSubmit(ctx, &submitReq{
		entrance: entrance, seq: seq, prio: prio,
		goal: []float64{pose.X, pose.Y, pose.Phi},
		pose: true,
	})
}

// PathJoints submits an ordered multi-point path given in joint angles
// (radians). Points are executed in order with a full stop at each one; the
// returned id tracks the whole path in Outcome/Wait/Cancel.
func (s *Service) PathJoints(ctx context.Context, entrance string, seq uint64, prio arbiter.Priority, goals [][]float64) (uint64, *PathInfo, error) {
	points := make([]Waypoint, len(goals))
	for i, g := range goals {
		points[i] = Waypoint{Kind: WaypointJoint, Joints: append([]float64(nil), g...)}
	}
	return s.sendPath(ctx, &pathReq{entrance: entrance, seq: seq, prio: prio, points: points})
}

// PathPoses submits an ordered multi-point path given in Cartesian poses.
func (s *Service) PathPoses(ctx context.Context, entrance string, seq uint64, prio arbiter.Priority, poses []kinematics.Pose) (uint64, *PathInfo, error) {
	points := make([]Waypoint, len(poses))
	for i, p := range poses {
		points[i] = Waypoint{Kind: WaypointPose, Pose: p}
	}
	return s.sendPath(ctx, &pathReq{entrance: entrance, seq: seq, prio: prio, points: points})
}

// PathMixed submits an ordered multi-point path whose points may freely mix
// joint-angle and Cartesian targets. Poses are seeded at the preceding
// resolved point, so the resolved joint path is deterministic.
func (s *Service) PathMixed(ctx context.Context, entrance string, seq uint64, prio arbiter.Priority, points []Waypoint) (uint64, *PathInfo, error) {
	cp := make([]Waypoint, len(points))
	copy(cp, points)
	for i := range cp {
		if cp[i].Kind == WaypointJoint {
			cp[i].Joints = append([]float64(nil), cp[i].Joints...)
		}
	}
	return s.sendPath(ctx, &pathReq{entrance: entrance, seq: seq, prio: prio, points: cp})
}

func (s *Service) sendPath(ctx context.Context, req *pathReq) (uint64, *PathInfo, error) {
	j := job{kind: "path", path: req,
		respID: make(chan uint64, 1), respPath: make(chan *PathInfo, 1), respErr: make(chan error, 1)}
	select {
	case s.jobs <- j:
	case <-ctx.Done():
		return 0, nil, ctx.Err()
	case <-s.done:
		return 0, nil, mcerr.New(mcerr.CodeFaultActive, "service closed")
	}
	select {
	case err := <-j.respErr:
		return 0, nil, err
	case id := <-j.respID:
		return id, <-j.respPath, nil
	case <-ctx.Done():
		return 0, nil, ctx.Err()
	case <-s.done:
		return 0, nil, mcerr.New(mcerr.CodeFaultActive, "service closed")
	}
}

// AppendJoints appends joint-angle waypoints to a path that has not finished.
// It may be used while the path is queued, running or paused.
func (s *Service) AppendJoints(ctx context.Context, entrance string, seq uint64, pathID uint64, goals [][]float64) (*PathInfo, error) {
	points := make([]Waypoint, len(goals))
	for i, g := range goals {
		points[i] = Waypoint{Kind: WaypointJoint, Joints: append([]float64(nil), g...)}
	}
	return s.appendPoints(ctx, entrance, seq, pathID, points)
}

// AppendPoses appends Cartesian waypoints to a path that has not finished.
func (s *Service) AppendPoses(ctx context.Context, entrance string, seq uint64, pathID uint64, poses []kinematics.Pose) (*PathInfo, error) {
	points := make([]Waypoint, len(poses))
	for i, p := range poses {
		points[i] = Waypoint{Kind: WaypointPose, Pose: p}
	}
	return s.appendPoints(ctx, entrance, seq, pathID, points)
}

// AppendMixed appends possibly mixed waypoints to a path that has not
// finished.
func (s *Service) AppendMixed(ctx context.Context, entrance string, seq uint64, pathID uint64, points []Waypoint) (*PathInfo, error) {
	cp := make([]Waypoint, len(points))
	copy(cp, points)
	for i := range cp {
		if cp[i].Kind == WaypointJoint {
			cp[i].Joints = append([]float64(nil), cp[i].Joints...)
		}
	}
	return s.appendPoints(ctx, entrance, seq, pathID, cp)
}

func (s *Service) appendPoints(ctx context.Context, entrance string, seq uint64, pathID uint64, points []Waypoint) (*PathInfo, error) {
	req := &appendReq{id: pathID, entrance: entrance, seq: seq, points: points}
	j := job{kind: "append", appended: req,
		respPath: make(chan *PathInfo, 1), respErr: make(chan error, 1)}
	select {
	case s.jobs <- j:
	case <-ctx.Done():
		return nil, ctx.Err()
	case <-s.done:
		return nil, mcerr.New(mcerr.CodeFaultActive, "service closed")
	}
	select {
	case err := <-j.respErr:
		return nil, err
	case info := <-j.respPath:
		return info, nil
	case <-ctx.Done():
		return nil, ctx.Err()
	case <-s.done:
		return nil, mcerr.New(mcerr.CodeFaultActive, "service closed")
	}
}

// Pause ramps the active motion to rest and holds it closed-loop. Queued
// commands are retained. It returns false when no motion is active/paused.
func (s *Service) Pause(ctx context.Context) (bool, error) {
	j := job{kind: "pause", respBool: make(chan bool, 1)}
	select {
	case s.jobs <- j:
	case <-ctx.Done():
		return false, ctx.Err()
	case <-s.done:
		return false, mcerr.New(mcerr.CodeFaultActive, "service closed")
	}
	select {
	case ok := <-j.respBool:
		return ok, nil
	case <-ctx.Done():
		return false, ctx.Err()
	case <-s.done:
		return false, mcerr.New(mcerr.CodeFaultActive, "service closed")
	}
}

// Resume continues a paused motion from its stop point and returns it to the
// original target; pending queued commands are retained. Resuming while the
// pause ramp has not yet settled, or with no paused motion, is an error.
func (s *Service) Resume(ctx context.Context) error {
	j := job{kind: "resume", respErr: make(chan error, 1)}
	return s.sendJob(ctx, j)
}

// Queue returns a single-consistent snapshot of capacity usage, the active
// command, pause state, execution order and progress.
func (s *Service) Queue(ctx context.Context) (QueueStatus, error) {
	j := job{kind: "queue", respQueue: make(chan QueueStatus, 1), respErr: make(chan error, 1)}
	select {
	case s.jobs <- j:
	case <-ctx.Done():
		return QueueStatus{}, ctx.Err()
	case <-s.done:
		return QueueStatus{}, mcerr.New(mcerr.CodeFaultActive, "service closed")
	}
	select {
	case err := <-j.respErr:
		return QueueStatus{}, err
	case st := <-j.respQueue:
		return st, nil
	case <-ctx.Done():
		return QueueStatus{}, ctx.Err()
	case <-s.done:
		return QueueStatus{}, mcerr.New(mcerr.CodeFaultActive, "service closed")
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
