package motion

import (
	"context"
	"fmt"
	"math"
	"runtime"
	"sync"
	"testing"
	"time"

	"github.com/h3yow/robot-motion-controller-go/internal/arbiter"
	"github.com/h3yow/robot-motion-controller-go/internal/config"
	"github.com/h3yow/robot-motion-controller-go/internal/kinematics"
	"github.com/h3yow/robot-motion-controller-go/internal/mcerr"
)

func testCfg() *config.Config {
	return &config.Config{
		Version: "1.0.0", Period: 2 * time.Millisecond,
		Links: []float64{0.4, 0.3, 0.1}, Elbow: "up",
		Axes: []config.AxisConfig{
			{Name: "j1", HomePosition: 0, MinPosition: -3, MaxPosition: 3,
				MaxVelocity: 3, MaxAccel: 8, MaxTorque: 25, Inertia: 1,
				GravityTorque: 0, FrictionTorque: 0.2,
				Gains: config.PIDGains{Kp: 40, Ki: 8, Kd: 10, IntegralLimit: 8}},
			{Name: "j2", HomePosition: 0, MinPosition: -2.5, MaxPosition: 2.5,
				MaxVelocity: 3, MaxAccel: 8, MaxTorque: 25, Inertia: 1,
				GravityTorque: 0, FrictionTorque: 0.2,
				Gains: config.PIDGains{Kp: 40, Ki: 8, Kd: 10, IntegralLimit: 8}},
			{Name: "j3", HomePosition: 0, MinPosition: -3, MaxPosition: 3,
				MaxVelocity: 4, MaxAccel: 10, MaxTorque: 20, Inertia: 0.6,
				GravityTorque: 0, FrictionTorque: 0.2,
				Gains: config.PIDGains{Kp: 30, Ki: 6, Kd: 8, IntegralLimit: 6}},
		},
		Tolerance: config.Tolerance{Position: 0.005, Velocity: 0.02, HoldTicks: 3},
		Fault: config.FaultPolicy{MaxPositionError: 0.2, DivergeError: 0.6,
			DropoutMaxTicks: 2, SaturationMaxTicks: 15, EndGraceTicks: 60,
			RecoveryHoldTicks: 2, SafeDecel: 12},
	}
}

func newManual(t *testing.T, opts ...Option) *Service {
	t.Helper()
	opts = append([]Option{WithManualClock()}, opts...)
	s, err := New(testCfg(), opts...)
	if err != nil {
		t.Fatalf("New: %v", err)
	}
	return s
}

func runUntil(t *testing.T, s *Service, id uint64, maxTicks int) *arbiter.Outcome {
	t.Helper()
	for k := 0; k < maxTicks; k++ {
		s.Step(1)
		oc, err := s.Outcome(id)
		if err != nil {
			t.Fatal(err)
		}
		if isTerminal(oc.State) {
			return oc
		}
	}
	oc, _ := s.Outcome(id)
	t.Fatalf("command %d did not terminate within %d ticks, state=%s axes=%+v", id, maxTicks, oc.State, s.Axes())
	return nil
}

func TestEndToEndJointMove(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	goal := []float64{0.8, -0.6, 0.4}
	id, err := s.MoveJoints(context.Background(), "hmi", 1, arbiter.PriorityNormal, goal)
	if err != nil {
		t.Fatalf("MoveJoints rejected: %v (goal=%v)", err, goal)
	}
	oc := runUntil(t, s, id, 20000)
	if oc.State != arbiter.StateCompleted {
		t.Fatalf("state=%s reason=%q", oc.State, oc.Reason)
	}
	axes := s.Axes()
	for i, g := range goal {
		if d := axes[i].Position - g; absF(d) > testCfg().Tolerance.Position {
			t.Fatalf("axis %s final=%.5f goal=%.5f residual=%.5f", testCfg().Axes[i].Name,
				axes[i].Position, g, d)
		}
	}
}

func TestEndToEndPoseMove(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	// A pose known reachable: full-ish extension.
	pose := kinematics.Pose{X: 0.6, Y: 0.2, Phi: 0.3}
	id, err := s.MovePose(context.Background(), "hmi", 1, arbiter.PriorityNormal, pose)
	if err != nil {
		t.Fatalf("MovePose: %v", err)
	}
	oc := runUntil(t, s, id, 20000)
	if oc.State != arbiter.StateCompleted {
		t.Fatalf("pose move state=%s reason=%s", oc.State, oc.Reason)
	}
}

func TestSubmissionRejections(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	ctx := context.Background()
	cases := []struct {
		name string
		pose *kinematics.Pose
		goal []float64
		code mcerr.Code
	}{
		{"joint beyond travel", nil, []float64{9, 0, 0}, mcerr.CodeJointLimitExceeded},
		{"joint non-finite", nil, []float64{0, math.NaN(), 0}, mcerr.CodeTargetNonFinite},
		{"cardinality", nil, []float64{1, 2}, mcerr.CodeTargetContradictory},
		{"pose unreachable", &kinematics.Pose{X: 50, Y: 50, Phi: 0}, nil, mcerr.CodeTargetUnreachable},
		{"pose non-finite", &kinematics.Pose{X: math.NaN(), Y: 0, Phi: 0}, nil, mcerr.CodeTargetNonFinite},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			var err error
			if tc.pose != nil {
				_, err = s.MovePose(ctx, "hmi", uint64(0), arbiter.PriorityNormal, *tc.pose)
			} else {
				_, err = s.MoveJoints(ctx, "hmi", uint64(0), arbiter.PriorityNormal, tc.goal)
			}
			if err == nil {
				t.Fatalf("expected %s, accepted (input pose=%v goal=%v)", tc.code, tc.pose, tc.goal)
			}
			if !mcerr.HasCode(err, tc.code) {
				t.Fatalf("want %s got %v (input pose=%v goal=%v)", tc.code, err, tc.pose, tc.goal)
			}
		})
	}
}

func TestStaleSequenceFromSameEntrance(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	ctx := context.Background()
	if _, err := s.MoveJoints(ctx, "a", 5, arbiter.PriorityNormal, []float64{0.2, 0, 0}); err != nil {
		t.Fatal(err)
	}
	if _, err := s.MoveJoints(ctx, "a", 4, arbiter.PriorityHigh, []float64{0.3, 0, 0}); !mcerr.HasCode(err, mcerr.CodeStaleSequence) {
		t.Fatalf("stale seq: %v", err)
	}
}

func TestPreemption(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	ctx := context.Background()
	id1, err := s.MoveJoints(ctx, "a", 1, arbiter.PriorityNormal, []float64{1.5, 0, 0})
	if err != nil {
		t.Fatal(err)
	}
	// Let it start moving.
	s.Step(50)
	id2, err := s.MoveJoints(ctx, "b", 1, arbiter.PriorityEmergency, []float64{-0.2, 0, 0})
	if err != nil {
		t.Fatalf("emergency submit: %v", err)
	}
	oc1 := runUntil(t, s, id1, 200)
	if oc1.State != arbiter.StatePreempted {
		t.Fatalf("cmd1 state=%s want preempted", oc1.State)
	}
	oc2 := runUntil(t, s, id2, 20000)
	if oc2.State != arbiter.StateCompleted {
		t.Fatalf("emergency cmd state=%s reason=%s", oc2.State, oc2.Reason)
	}
}

func TestCancellation(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	ctx := context.Background()
	id, err := s.MoveJoints(ctx, "a", 1, arbiter.PriorityNormal, []float64{1.5, 0, 0})
	if err != nil {
		t.Fatal(err)
	}
	s.Step(50)
	if err := s.Cancel(ctx, id); err != nil {
		t.Fatal(err)
	}
	oc := runUntil(t, s, id, 5000)
	if oc.State != arbiter.StateCanceled {
		t.Fatalf("state=%s want canceled, reason=%q", oc.State, oc.Reason)
	}
	if v := s.Axes()[0].Velocity; absF(v) > testCfg().Tolerance.Velocity {
		t.Fatalf("residual speed after cancel %.4f", v)
	}
	if err := s.Cancel(ctx, 9999); !mcerr.HasCode(err, mcerr.CodeCommandUnknown) {
		t.Fatalf("unknown cancel: %v", err)
	}
}

func TestFeedbackFaultAndRecovery(t *testing.T) {
	// First service runs the move almost to completion (no faults); a
	// second service replays and drops exactly one feedback cycle near the
	// end, when the axis is at rest, so the recovery error stays bounded.
	driveNearEnd := func(t *testing.T, s *Service, id uint64) int {
		t.Helper()
		for k := 0; k < 20000; k++ {
			s.Step(1)
			idx, total := progressOf(s)
			if total > 0 && idx >= total-2 {
				return k
			}
			oc, _ := s.Outcome(id)
			if isTerminal(oc.State) {
				t.Fatalf("move ended too early: %s %s", oc.State, oc.Reason)
			}
		}
		t.Fatal("never approached end")
		return 0
	}
	// Probe the near-end sample count with a fault-free run.
	probe := newManual(t)
	pid, _ := probe.MoveJoints(context.Background(), "a", 1, arbiter.PriorityNormal,
		[]float64{0.8, 0, 0})
	nearEnd := driveNearEnd(t, probe, pid)
	probe.Close()

	drop := func(tick int64, axis int) bool {
		return axis == 0 && tick == int64(nearEnd+1)
	}
	s := newManual(t, WithFeedbackDrop(drop))
	defer s.Close()
	id, err := s.MoveJoints(context.Background(), "a", 1, arbiter.PriorityNormal,
		[]float64{0.8, 0, 0})
	if err != nil {
		t.Fatal(err)
	}
	var sawRecov bool
	for k := 0; k < nearEnd+20; k++ {
		s.Step(1)
		oc, _ := s.Outcome(id)
		if oc.State == arbiter.StateFaultedRecov {
			sawRecov = true
			break
		}
		if isTerminal(oc.State) {
			t.Fatalf("unexpected terminal %s before fault: %s", oc.State, oc.Reason)
		}
	}
	if !sawRecov {
		t.Fatal("never entered recoverable fault state")
	}
	// Immediate Recover must be rejected: preconditions not yet held.
	if err := s.Recover(context.Background()); !mcerr.HasCode(err, mcerr.CodeRecoveryPrecondition) {
		t.Fatalf("premature recover: %v", err)
	}
	// Step with healthy feedback, then recover, and finish the move.
	for k := 0; k < 500; k++ {
		s.Step(1)
		if s.Recover(context.Background()) == nil {
			break
		}
	}
	oc := runUntil(t, s, id, 20000)
	if oc.State != arbiter.StateCompleted {
		t.Fatalf("post-recovery state=%s reason=%s", oc.State, oc.Reason)
	}
}

func TestPersistentFeedbackLossIsFatal(t *testing.T) {
	drop := func(tick int64, axis int) bool {
		return axis == 0 && tick >= 50
	}
	s := newManual(t, WithFeedbackDrop(drop))
	defer s.Close()
	id, err := s.MoveJoints(context.Background(), "a", 1, arbiter.PriorityNormal,
		[]float64{0.8, 0, 0})
	if err != nil {
		t.Fatal(err)
	}
	oc := runUntil(t, s, id, 5000)
	if oc.State != arbiter.StateFaulted {
		t.Fatalf("want fatal faulted, got %s (%s)", oc.State, oc.Reason)
	}
	// Any new move must be refused while fault active.
	if _, err := s.MoveJoints(context.Background(), "b", 1, arbiter.PriorityEmergency,
		[]float64{0, 0, 0}); !mcerr.HasCode(err, mcerr.CodeFaultActive) {
		t.Fatalf("move during fatal fault should be refused, got %v", err)
	}
}

func TestConcurrentSubmissionsAndSnapshots(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	const n = 20
	var subWg, readerWg sync.WaitGroup
	subDone := make(chan struct{})
	errs := make(chan error, 128)
	ids := make(chan uint64, n)

	// Submitters race concurrently.
	for g := 0; g < n; g++ {
		subWg.Add(1)
		go func(g int) {
			defer subWg.Done()
			entrance := fmt.Sprintf("ui%d", g)
			goal := 0.1 + 0.01*float64(g)
			id, err := s.MoveJoints(context.Background(), entrance, 1, arbiter.PriorityNormal,
				[]float64{goal, 0, 0})
			if err != nil {
				errs <- fmt.Errorf("submit ui%d: %w", g, err)
				return
			}
			ids <- id
		}(g)
	}
	go func() { subWg.Wait(); close(subDone) }()

	// Concurrent snapshot readers that poll and yield (they never lock out
	// the single cycle writer for long).
	stopReaders := make(chan struct{})
	for g := 0; g < 4; g++ {
		readerWg.Add(1)
		go func() {
			defer readerWg.Done()
			for {
				select {
				case <-stopReaders:
					return
				default:
					st := s.Axes()
					if len(st) != 3 {
						errs <- fmt.Errorf("half-updated snapshot len=%d", len(st))
						return
					}
					runtime.Gosched()
				}
			}
		}()
	}

	// Collect admitted ids while pumping cycles on THIS goroutine (the
	// fixed clock permits one driver).
	seen := map[uint64]bool{}
	collecting := true
	for collecting {
		select {
		case <-subDone:
			collecting = false
		case id := <-ids:
			seen[id] = true
		default:
			s.Step(1)
		}
	}
	// Drain any ids emitted with the very last submissions.
	for {
		select {
		case id := <-ids:
			seen[id] = true
		default:
			goto drained
		}
	}
drained:
	// Drive every admitted command to a terminal state while readers run.
	// Queue order is priority/FIFO, so sweep the whole set each cycle
	// instead of waiting on one possibly-queued id.
	pending := func() bool {
		for id := range seen {
			oc, err := s.Outcome(id)
			if err != nil {
				errs <- err
				return false
			}
			if !isTerminal(oc.State) {
				return true
			}
		}
		return false
	}
	for k := 0; k < 200000 && pending(); k++ {
		s.Step(1)
	}
	for id := range seen {
		oc, _ := s.Outcome(id)
		if oc.State != arbiter.StateCompleted {
			errs <- fmt.Errorf("id %d ended %s: %s", id, oc.State, oc.Reason)
		}
	}
	close(stopReaders)
	readerWg.Wait()
	close(errs)
	for e := range errs {
		t.Fatal(e)
	}
	if len(seen) != n {
		t.Fatalf("admitted %d commands, want %d", len(seen), n)
	}
}

func TestDeterministicReplay(t *testing.T) {
	// Two fresh services, identical inputs -> identical final positions.
	s1 := newManual(t)
	defer s1.Close()
	id1, _ := s1.MoveJoints(context.Background(), "a", 1, arbiter.PriorityNormal,
		[]float64{0.7, 0.3, -0.2})
	runUntil(t, s1, id1, 20000)
	p1 := make([]float64, 3)
	for i, a := range s1.Axes() {
		p1[i] = a.Position
	}
	s2 := newManual(t)
	defer s2.Close()
	id2, _ := s2.MoveJoints(context.Background(), "a", 1, arbiter.PriorityNormal,
		[]float64{0.7, 0.3, -0.2})
	runUntil(t, s2, id2, 20000)
	for i, a := range s2.Axes() {
		if a.Position != p1[i] {
			t.Fatalf("non-deterministic replay axis %d: %.12f vs %.12f", i, a.Position, p1[i])
		}
	}
}

func absF(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}

// progressOf reads controller progress through the status snapshot; the
// service exposes progress indirectly, so derive from axes target/position.
func progressOf(s *Service) (int, int) {
	// Use the controller Progress via the service's controller field.
	return s.ctrl.Progress()
}

// TestPoseReachableButOutsideJointTravel distinguishes an IK solution that
// exists geometrically yet violates the configured joint range.
func TestPoseReachableButOutsideJointTravel(t *testing.T) {
	cfg := testCfg()
	// Restrict joint 3 to a tiny band so a reachable pose's wrist rotation
	// cannot be satisfied inside the travel.
	cfg.Axes[2].MinPosition = -0.01
	cfg.Axes[2].MaxPosition = 0.01
	s, err := New(cfg, WithManualClock())
	if err != nil {
		t.Fatal(err)
	}
	defer s.Close()
	// A reachable pose requiring substantial q3 rotation.
	_, err = s.MovePose(context.Background(), "hmi", 1, arbiter.PriorityNormal,
		kinematics.Pose{X: 0.45, Y: 0.35, Phi: 1.0})
	if err == nil {
		t.Fatal("pose requiring out-of-range q3 must be rejected")
	}
	if !mcerr.HasCode(err, mcerr.CodeJointLimitExceeded) {
		t.Fatalf("want joint_limit_exceeded (reachable pose, joint range), got %v", err)
	}
}

// TestNoSilentResumeAfterFatal asserts a fatal fault never lets queued
// commands start and that recovery is refused.
func TestNoSilentResumeAfterFatal(t *testing.T) {
	drop := func(tick int64, axis int) bool { return axis == 0 && tick >= 30 }
	s := newManual(t, WithFeedbackDrop(drop))
	defer s.Close()
	// Queue a second command behind the first.
	id1, err := s.MoveJoints(context.Background(), "a", 1, arbiter.PriorityNormal, []float64{0.5, 0, 0})
	if err != nil {
		t.Fatal(err)
	}
	id2, err := s.MoveJoints(context.Background(), "b", 1, arbiter.PriorityLow, []float64{-0.5, 0, 0})
	if err != nil {
		t.Fatal(err)
	}
	oc1 := runUntil(t, s, id1, 5000)
	if oc1.State != arbiter.StateFaulted {
		t.Fatalf("id1 want fatal faulted, got %s", oc1.State)
	}
	// The queued command must remain queued (never silently started) after
	// a fatal fault.
	oc2, _ := s.Outcome(id2)
	if oc2.State != arbiter.StateQueued {
		t.Fatalf("queued cmd must stay queued after fatal fault, got %s", oc2.State)
	}
	if err := s.Recover(context.Background()); !mcerr.HasCode(err, mcerr.CodeFaultActive) {
		t.Fatalf("fatal recover refused expected, got %v", err)
	}
}
