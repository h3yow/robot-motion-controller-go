package motion

import (
	"context"
	"fmt"
	"runtime"
	"sync"
	"testing"

	"github.com/h3yow/robot-motion-controller-go/internal/arbiter"
	"github.com/h3yow/robot-motion-controller-go/internal/config"
	"github.com/h3yow/robot-motion-controller-go/internal/control"
	"github.com/h3yow/robot-motion-controller-go/internal/kinematics"
	"github.com/h3yow/robot-motion-controller-go/internal/mcerr"
)

// collectVisited drives until id terminates and records the ordered sequence
// of active-segment goals (consecutive duplicates collapse).
func collectVisited(t *testing.T, s *Service, id uint64, maxTicks int) ([][]float64, *arbiter.Outcome) {
	t.Helper()
	var visited [][]float64
	eq := func(a, b []float64) bool {
		if len(a) != len(b) {
			return false
		}
		for i := range a {
			if a[i] != b[i] {
				return false
			}
		}
		return true
	}
	for k := 0; k < maxTicks; k++ {
		s.Step(1)
		st, err := s.Queue(context.Background())
		if err != nil {
			t.Fatal(err)
		}
		if len(st.Active.Goal) == 3 {
			g := append([]float64(nil), st.Active.Goal...)
			if len(visited) == 0 || !eq(visited[len(visited)-1], g) {
				visited = append(visited, g)
			}
		}
		oc, err := s.Outcome(id)
		if err != nil {
			t.Fatal(err)
		}
		if isTerminal(oc.State) {
			return visited, oc
		}
	}
	t.Fatalf("command %d did not terminate within %d ticks", id, maxTicks)
	return nil, nil
}

// settleAfterPause steps until the controller has parked (phase paused) and
// the hold itself is at rest for run consecutive cycles; it returns the
// stationary joint positions.
func settleAfterPause(t *testing.T, s *Service, run int) []float64 {
	t.Helper()
	parked := false
	rest := 0
	for k := 0; k < 40000; k++ {
		s.Step(1)
		st, err := s.Queue(context.Background())
		if err != nil {
			t.Fatal(err)
		}
		if st.Phase == control.PhasePaused {
			parked = true
		}
		atRest := true
		for _, a := range s.Axes() {
			if absF(a.Velocity) > testCfg().Tolerance.Velocity {
				atRest = false
			}
		}
		if parked && atRest {
			rest++
		} else {
			rest = 0
		}
		if rest >= run {
			q := make([]float64, 3)
			for i, a := range s.Axes() {
				q[i] = a.Position
			}
			return q
		}
	}
	t.Fatalf("controller never settled after pause (parked=%v)", parked)
	return nil
}

// TestPathMixedJointAndPoseWaypoints covers MovePath with joint-space and
// Cartesian waypoints mixed in one command: every waypoint is reached in
// order and the final pose matches the last waypoint.
func TestPathMixedJointAndPoseWaypoints(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	ctx := context.Background()

	jointA := []float64{0.5, -0.3, 0.2}
	poseB := kinematics.Pose{X: 0.55, Y: 0.25, Phi: 0.4}
	jointC := []float64{-0.3, 0.4, -0.5}
	poseD := kinematics.Pose{X: 0.4, Y: -0.2, Phi: -0.2}

	goalA := jointA
	goalB, err := s.arm.IK(poseB, goalA)
	if err != nil {
		t.Fatal(err)
	}
	goalC := jointC
	goalD, err := s.arm.IK(poseD, goalC)
	if err != nil {
		t.Fatal(err)
	}

	id, err := s.MovePath(ctx, "hmi", 1, arbiter.PriorityNormal, []Waypoint{
		{Joints: jointA},
		{Pose: &poseB},
		{Joints: jointC},
		{Pose: &poseD},
	})
	if err != nil {
		t.Fatalf("MovePath rejected: %v", err)
	}
	visited, oc := collectVisited(t, s, id, 60000)
	if oc.State != arbiter.StateCompleted {
		t.Fatalf("path state=%s reason=%q", oc.State, oc.Reason)
	}
	want := [][]float64{goalA, goalB, goalC, goalD}
	if len(visited) != len(want) {
		t.Fatalf("visited %d waypoints, want %d, visited=%v", len(visited), len(want), visited)
	}
	for i := range want {
		for ax := range want[i] {
			if visited[i][ax] != want[i][ax] {
				t.Fatalf("waypoint %d axis %d executed %.8f != admitted %.8f (order/skip regression)",
					i, ax, visited[i][ax], want[i][ax])
			}
		}
	}
	for ax, g := range goalD {
		if d := absF(s.Axes()[ax].Position - g); d > testCfg().Tolerance.Position {
			t.Fatalf("final axis %d residual %.5f", ax, d)
		}
	}
}

// TestPathAppendWhileRunning appends waypoints both while a segment runs
// and at a segment boundary; all points execute in exact order.
func TestPathAppendWhileRunning(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	ctx := context.Background()
	id, err := s.MovePath(ctx, "hmi", 1, arbiter.PriorityNormal, []Waypoint{
		{Joints: []float64{0.4, 0, 0}},
		{Joints: []float64{0.7, 0.1, 0}},
	})
	if err != nil {
		t.Fatal(err)
	}
	firstAppend := []Waypoint{
		{Joints: []float64{0.2, 0.4, 0}},
		{Pose: &kinematics.Pose{X: 0.5, Y: 0.1, Phi: 0.3}},
	}
	secondAppend := []Waypoint{
		{Joints: []float64{-0.2, -0.2, 0.3}},
		{Joints: []float64{0.1, 0.1, 0.1}},
	}
	var visited [][]float64
	eq := func(a, b []float64) bool {
		if len(a) != len(b) {
			return false
		}
		for i := range a {
			if a[i] != b[i] {
				return false
			}
		}
		return true
	}
	record := func(g []float64) {
		if len(visited) == 0 || !eq(visited[len(visited)-1], g) {
			visited = append(visited, append([]float64(nil), g...))
		}
	}
	var oc *arbiter.Outcome
	secondDone := false
	for k := 0; k < 90000; k++ {
		s.Step(1)
		st, qerr := s.Queue(ctx)
		if qerr != nil {
			t.Fatal(qerr)
		}
		if len(st.Active.Goal) == 3 {
			record(st.Active.Goal)
		}
		if k == 20 {
			if err := s.AppendWaypoints(ctx, id, firstAppend); err != nil {
				t.Fatalf("first append: %v", err)
			}
		}
		if !secondDone && len(visited) == 3 && st.Active.WaypointIndex >= 2 {
			if err := s.AppendWaypoints(ctx, id, secondAppend); err != nil {
				t.Fatalf("second append: %v", err)
			}
			secondDone = true
		}
		oc, _ = s.Outcome(id)
		if isTerminal(oc.State) {
			break
		}
	}
	if oc.State != arbiter.StateCompleted {
		t.Fatalf("state=%s reason=%q", oc.State, oc.Reason)
	}
	goalPose, err := s.arm.IK(*firstAppend[1].Pose, firstAppend[0].Joints)
	if err != nil {
		t.Fatal(err)
	}
	want := [][]float64{
		{0.4, 0, 0}, {0.7, 0.1, 0},
		{0.2, 0.4, 0}, goalPose,
		{-0.2, -0.2, 0.3}, {0.1, 0.1, 0.1},
	}
	if len(visited) != len(want) {
		t.Fatalf("visited %d goals want %d: %v", len(visited), len(want), visited)
	}
	for i := range want {
		for ax := range want[i] {
			if visited[i][ax] != want[i][ax] {
				t.Fatalf("goal %d axis %d got %.8f want %.8f", i, ax, visited[i][ax], want[i][ax])
			}
		}
	}
}

// TestPathAppendRejections distinguishes append failure reasons.
func TestPathAppendRejections(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	ctx := context.Background()
	id, err := s.MoveJoints(ctx, "hmi", 1, arbiter.PriorityNormal, []float64{0.5, 0, 0})
	if err != nil {
		t.Fatal(err)
	}
	runUntil(t, s, id, 20000)

	err = s.AppendWaypoints(ctx, id, []Waypoint{{Joints: []float64{0.2, 0, 0}}})
	if !mcerr.HasCode(err, mcerr.CodeOperationInvalid) {
		t.Fatalf("append to finished path: want operation_invalid, got %v", err)
	}
	err = s.AppendWaypoints(ctx, 99999, []Waypoint{{Joints: []float64{0.2, 0, 0}}})
	if !mcerr.HasCode(err, mcerr.CodeCommandUnknown) {
		t.Fatalf("append to unknown id: want command_unknown, got %v", err)
	}
	id2, err := s.MoveJoints(ctx, "hmi", 2, arbiter.PriorityNormal, []float64{0.2, 0, 0})
	if err != nil {
		t.Fatal(err)
	}
	s.Step(5)
	if err := s.AppendWaypoints(ctx, id2, []Waypoint{{Joints: []float64{9, 0, 0}}}); !mcerr.HasCode(err, mcerr.CodeJointLimitExceeded) {
		t.Fatalf("append out-of-range: want joint_limit_exceeded, got %v", err)
	}
	if err := s.AppendWaypoints(ctx, id2, []Waypoint{{}}); !mcerr.HasCode(err, mcerr.CodeTargetContradictory) {
		t.Fatalf("append empty waypoint: want target_contradictory, got %v", err)
	}
	runUntil(t, s, id2, 20000)
}

func smallQueueCfg(maxPending int) *config.Config {
	c := testCfg()
	m := maxPending
	c.MaxPendingCommands = &m
	c.Version = "1.1.0"
	return c
}

// TestQueueCapacityRejects verifies the hard pending ceiling and the
// distinct queue_full reason, including for appends.
func TestQueueCapacityRejects(t *testing.T) {
	s, err := New(smallQueueCfg(3), WithManualClock())
	if err != nil {
		t.Fatal(err)
	}
	defer s.Close()
	ctx := context.Background()
	if _, err := s.MoveJoints(ctx, "hmi", 1, arbiter.PriorityNormal, []float64{0.8, 0, 0}); err != nil {
		t.Fatal(err)
	}
	id3, err := s.MovePath(ctx, "hmi", 2, arbiter.PriorityNormal, []Waypoint{
		{Joints: []float64{0.2, 0, 0}},
		{Joints: []float64{0.3, 0, 0}},
		{Joints: []float64{0.4, 0, 0}},
	})
	if err != nil {
		t.Fatalf("budget-exact path rejected: %v", err)
	}
	if _, err := s.MoveJoints(ctx, "hmi", 3, arbiter.PriorityNormal, []float64{0.1, 0, 0}); !mcerr.HasCode(err, mcerr.CodeQueueFull) {
		t.Fatalf("over-capacity submit: want queue_full, got %v", err)
	}
	if err := s.AppendWaypoints(ctx, id3, []Waypoint{{Joints: []float64{0.05, 0, 0}}}); !mcerr.HasCode(err, mcerr.CodeQueueFull) {
		t.Fatalf("over-capacity append: want queue_full, got %v", err)
	}
	// Reported pending count must match the exact budget.
	st, _ := s.Queue(ctx)
	if st.Pending != 3 || st.MaxPending != 3 {
		t.Fatalf("pending=%d max=%d, want 3/3", st.Pending, st.MaxPending)
	}
	// A strictly higher priority command preempts and is not capacity bound.
	emID, err := s.MoveJoints(ctx, "hmi", 4, arbiter.PriorityEmergency, []float64{-0.2, 0, 0})
	if err != nil {
		t.Fatalf("preempting emergency submit: %v", err)
	}
	oc := runUntil(t, s, emID, 20000)
	if oc.State != arbiter.StateCompleted {
		t.Fatalf("emergency move: %s %s", oc.State, oc.Reason)
	}
}

// TestLegacyConfigDefaultCapacity confirms a 1.0 document without the new
// field keeps the documented default budget (no silent downgrade).
func TestLegacyConfigDefaultCapacity(t *testing.T) {
	c := testCfg() // version 1.0.0, MaxPendingCommands nil
	s, err := New(c, WithManualClock())
	if err != nil {
		t.Fatal(err)
	}
	defer s.Close()
	st, _ := s.Queue(context.Background())
	if st.MaxPending != config.DefaultMaxPendingCommands {
		t.Fatalf("legacy default=%d want %d", st.MaxPending, config.DefaultMaxPendingCommands)
	}
}

// TestPauseResumeReachesGoal: pause holds the axes in place with the loop
// active, queued commands survive, resume reaches the original goal, and
// pause duration does not change the final result (within tolerance).
func TestPauseResumeReachesGoal(t *testing.T) {
	cases := []struct {
		name      string
		holdTicks int
	}{
		{"short_pause", 10},
		{"long_pause", 500},
	}
	finalGoals := make([][]float64, len(cases))
	for ci, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			s := newManual(t)
			defer s.Close()
			ctx := context.Background()
			goal := []float64{0.8, -0.5, 0.3}
			id, err := s.MovePath(ctx, "hmi", 1, arbiter.PriorityNormal, []Waypoint{
				{Joints: []float64{0.5, 0, 0}},
				{Joints: goal},
			})
			if err != nil {
				t.Fatal(err)
			}
			followID, err := s.MoveJoints(ctx, "panel", 1, arbiter.PriorityLow, []float64{0.1, 0, 0})
			if err != nil {
				t.Fatal(err)
			}
			s.Step(40)
			if err := s.Pause(ctx, id); err != nil {
				t.Fatalf("pause: %v", err)
			}
			holdQ := settleAfterPause(t, s, 30)
			st, _ := s.Queue(ctx)
			if !st.Paused || st.Phase != control.PhasePaused {
				t.Fatalf("after settle Paused=%v phase=%s", st.Paused, st.Phase)
			}
			if err := s.Pause(ctx, id); err != nil {
				t.Fatalf("idempotent pause: %v", err)
			}
			if q, _ := s.Queue(ctx); len(q.Queued) != 1 || q.Queued[0].ID != followID {
				t.Fatalf("queued follower lost during pause: %+v", q.Queued)
			}
			// From the stationary point, the hold keeps every axis parked
			// and at rest for the whole pause; an isolated stiction micro-slip
			// is tolerated, but sustained drift is not.
			posBreaches, speedBreaches, maxDrift := 0, 0, 0.0
			for k := 0; k < tc.holdTicks; k++ {
				s.Step(1)
				for i, a := range s.Axes() {
					d := absF(a.Position - holdQ[i])
					if d > maxDrift {
						maxDrift = d
					}
					if d > testCfg().Tolerance.Position {
						posBreaches++
					}
					if absF(a.Velocity) > testCfg().Tolerance.Velocity {
						speedBreaches++
					}
				}
			}
			if posBreaches > 4 || speedBreaches > 4 || maxDrift > 2*testCfg().Tolerance.Position {
				t.Fatalf("hold not steady: posBreaches=%d speedBreaches=%d maxDrift=%.5f",
					posBreaches, speedBreaches, maxDrift)
			}
			for i, a := range s.Axes() {
				if absF(a.Position-holdQ[i]) > testCfg().Tolerance.Position {
					t.Fatalf("final hold axis %d drifted %.6f", i, a.Position-holdQ[i])
				}
			}
			if err := s.Resume(ctx, followID); !mcerr.HasCode(err, mcerr.CodeOperationInvalid) {
				t.Fatalf("resume queued cmd: want operation_invalid, got %v", err)
			}
			if err := s.Resume(ctx, id); err != nil {
				t.Fatalf("resume: %v", err)
			}
			oc := runUntil(t, s, id, 40000)
			if oc.State != arbiter.StateCompleted {
				t.Fatalf("path after resume: %s %s", oc.State, oc.Reason)
			}
			fg := make([]float64, 3)
			for i := range fg {
				if d := absF(s.Axes()[i].Position - goal[i]); d > testCfg().Tolerance.Position {
					t.Fatalf("final axis %d %.5f != goal %.5f", i, s.Axes()[i].Position, goal[i])
				}
				fg[i] = s.Axes()[i].Position
			}
			finalGoals[ci] = fg
			ocf := runUntil(t, s, followID, 20000)
			if ocf.State != arbiter.StateCompleted {
				t.Fatalf("follower after resumed path: %s %s", ocf.State, ocf.Reason)
			}
		})
	}
	for i := range finalGoals[0] {
		if absF(finalGoals[0][i]-finalGoals[1][i]) > testCfg().Tolerance.Position {
			t.Fatalf("final result depends on pause duration: %.6f vs %.6f", finalGoals[0][i], finalGoals[1][i])
		}
	}
}

// TestPauseThenCancel verifies cancellation stays effective while paused.
func TestPauseThenCancel(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	ctx := context.Background()
	id, err := s.MoveJoints(ctx, "hmi", 1, arbiter.PriorityNormal, []float64{1.2, 0, 0})
	if err != nil {
		t.Fatal(err)
	}
	s.Step(40)
	if err := s.Pause(ctx, id); err != nil {
		t.Fatal(err)
	}
	settleAfterPause(t, s, 5)
	s.Step(20)
	if err := s.Cancel(ctx, id); err != nil {
		t.Fatal(err)
	}
	oc := runUntil(t, s, id, 5000)
	if oc.State != arbiter.StateCanceled {
		t.Fatalf("want canceled, got %s (%s)", oc.State, oc.Reason)
	}
	if v := s.Axes()[0].Velocity; absF(v) > testCfg().Tolerance.Velocity {
		t.Fatalf("residual speed after pause+cancel %.4f", v)
	}
	id2, _ := s.MoveJoints(ctx, "a", 1, arbiter.PriorityNormal, []float64{0.2, 0, 0})
	id3, _ := s.MoveJoints(ctx, "b", 1, arbiter.PriorityNormal, []float64{0.3, 0, 0})
	if err := s.Pause(ctx, id3); !mcerr.HasCode(err, mcerr.CodeOperationInvalid) {
		t.Fatalf("pause queued: want operation_invalid, got %v", err)
	}
	runUntil(t, s, id2, 20000)
}

// replayPauseOnce runs one fixed scenario and returns final positions.
func replayPauseOnce(t *testing.T, pauseAt, pauseTicks int) []float64 {
	t.Helper()
	s := newManual(t)
	defer s.Close()
	ctx := context.Background()
	id, err := s.MovePath(ctx, "hmi", 1, arbiter.PriorityNormal, []Waypoint{
		{Joints: []float64{0.5, 0, 0}},
		{Pose: &kinematics.Pose{X: 0.55, Y: 0.25, Phi: 0.4}},
		{Joints: []float64{-0.2, 0.3, -0.4}},
	})
	if err != nil {
		t.Fatal(err)
	}
	paused, resumed := false, false
	for k := 0; k < 200000; k++ {
		s.Step(1)
		if !paused && k == pauseAt {
			if err := s.Pause(ctx, id); err != nil {
				t.Fatal(err)
			}
			paused = true
		}
		if paused && !resumed && k == pauseAt+pauseTicks {
			if err := s.Resume(ctx, id); err != nil {
				t.Fatal(err)
			}
			resumed = true
		}
		oc, _ := s.Outcome(id)
		if isTerminal(oc.State) {
			if oc.State != arbiter.StateCompleted {
				t.Fatalf("replay ended %s: %s", oc.State, oc.Reason)
			}
			break
		}
	}
	out := make([]float64, 3)
	for i, a := range s.Axes() {
		out[i] = a.Position
	}
	return out
}

// TestPathPauseDeterminism: identical inputs reproduce bit-identical output.
func TestPathPauseDeterminism(t *testing.T) {
	p1 := replayPauseOnce(t, 40, 120)
	p2 := replayPauseOnce(t, 40, 120)
	for i := range p1 {
		if p1[i] != p2[i] {
			t.Fatalf("non-deterministic paused-path replay axis %d: %.12f vs %.12f", i, p1[i], p2[i])
		}
	}
}

// TestConcurrentPathsAppendsAndQueries: many entrances concurrently submit
// single moves and multi-waypoint paths, append concurrently, and read
// Queue/Axes concurrently while one goroutine drives the clock. Clean under
// -race; every admitted command completes; snapshots stay self-consistent.
func TestConcurrentPathsAppendsAndQueries(t *testing.T) {
	cfg := smallQueueCfg(4096)
	s, err := New(cfg, WithManualClock())
	if err != nil {
		t.Fatal(err)
	}
	defer s.Close()
	ctx := context.Background()

	const entrances, perEntrance = 12, 4
	var wg sync.WaitGroup
	errs := make(chan error, 1024)
	ids := make(chan uint64, entrances*perEntrance)
	submitsDone := make(chan struct{})

	// One goroutine per entrance submits its four commands in strictly
	// increasing seq order; entrances race each other.
	for g := 0; g < entrances; g++ {
		wg.Add(1)
		go func(g int) {
			defer wg.Done()
			entrance := fmt.Sprintf("ui%d", g)
			for k := 0; k < perEntrance; k++ {
				seq := uint64(k + 1)
				base := 0.05 + 0.02*float64(g) + 0.003*float64(k)
				var id uint64
				var e error
				switch k % 3 {
				case 0:
					id, e = s.MoveJoints(ctx, entrance, seq, arbiter.PriorityNormal,
						[]float64{base, 0, 0})
				case 1:
					id, e = s.MovePath(ctx, entrance, seq, arbiter.PriorityNormal, []Waypoint{
						{Joints: []float64{base, 0.1 * float64(k), 0}},
						{Pose: &kinematics.Pose{X: 0.4 + 0.01*float64(g), Y: 0.1 + 0.01*float64(k), Phi: 0.2}},
						{Joints: []float64{base * 0.5, -0.1, 0}},
					})
				default:
					id, e = s.MovePose(ctx, entrance, seq, arbiter.PriorityNormal,
						kinematics.Pose{X: 0.45 + 0.004*float64(g), Y: 0.12 + 0.004*float64(k), Phi: 0.1})
				}
				if e != nil {
					errs <- fmt.Errorf("submit %s#%d: %w", entrance, seq, e)
					return
				}
				ids <- id
				if k%3 == 1 {
					if ae := s.AppendWaypoints(ctx, id, []Waypoint{{Joints: []float64{0, 0, 0}}}); ae != nil &&
						!mcerr.HasCode(ae, mcerr.CodeOperationInvalid) &&
						!mcerr.HasCode(ae, mcerr.CodeQueueFull) {
						errs <- fmt.Errorf("append %s id=%d: %w", entrance, id, ae)
						return
					}
				}
			}
		}(g)
	}

	stopReaders := make(chan struct{})
	var rwg sync.WaitGroup
	for r := 0; r < 6; r++ {
		rwg.Add(1)
		go func() {
			defer rwg.Done()
			for {
				select {
				case <-stopReaders:
					return
				default:
					st, qerr := s.Queue(ctx)
					if qerr == nil {
						if st.Pending < 0 || st.Pending > st.MaxPending {
							errs <- fmt.Errorf("impossible pending=%d max=%d", st.Pending, st.MaxPending)
						}
						sum := 0
						for _, q := range st.Queued {
							sum += q.Waypoints
							if len(q.Goals) != q.Waypoints {
								errs <- fmt.Errorf("half-updated queued cmd: waypoints=%d goals=%d",
									q.Waypoints, len(q.Goals))
							}
						}
						if st.ActiveID != 0 {
							sum += st.Active.WaypointTotal - st.Active.WaypointIndex - 1
						}
						if sum != st.Pending {
							errs <- fmt.Errorf("inconsistent snapshot: pending=%d recomputed=%d", st.Pending, sum)
						}
					}
					if ax := s.Axes(); len(ax) != 3 {
						errs <- fmt.Errorf("half-updated axes len=%d", len(ax))
					}
					runtime.Gosched()
				}
			}
		}()
	}

	go func() {
		wg.Wait()
		close(submitsDone)
		close(ids)
	}()
	seen := map[uint64]bool{}
	var seenMu sync.Mutex
	collectDone := make(chan struct{})
	go func() {
		for id := range ids {
			seenMu.Lock()
			seen[id] = true
			seenMu.Unlock()
		}
		close(collectDone)
	}()
	for {
		select {
		case <-submitsDone:
			goto collected
		default:
			s.Step(1)
		}
	}
collected:
	<-collectDone
	seenMu.Lock()
	admitted := len(seen)
	seenMu.Unlock()
	if admitted != entrances*perEntrance {
		select {
		case e := <-errs:
			t.Fatalf("admitted %d, want %d; submitter error: %v", admitted, entrances*perEntrance, e)
		default:
		}
		t.Fatalf("admitted %d, want %d", admitted, entrances*perEntrance)
	}

	pending := func() bool {
		for id := range seen {
			oc, e := s.Outcome(id)
			if e != nil {
				errs <- e
				return false
			}
			if !isTerminal(oc.State) {
				return true
			}
		}
		return false
	}
	for k := 0; k < 500000 && pending(); k++ {
		s.Step(1)
	}
	for id := range seen {
		oc, _ := s.Outcome(id)
		if oc.State != arbiter.StateCompleted {
			errs <- fmt.Errorf("id %d ended %s: %s", id, oc.State, oc.Reason)
		}
	}
	close(stopReaders)
	rwg.Wait()
	close(errs)
	for e := range errs {
		t.Error(e)
	}
}

// TestPauseSurvivesRecoverableFault: a feedback fault during a pause is
// recoverable; after Recover the command is still paused (queue intact),
// and an explicit Resume then completes the path to the original goal.
func TestPauseSurvivesRecoverableFault(t *testing.T) {
	var mu sync.Mutex
	windowOn := false
	inj := func(tick int64, axis int) bool {
		mu.Lock()
		on := windowOn
		mu.Unlock()
		return on && axis == 0
	}
	s := newManual(t, WithFeedbackDrop(inj))
	defer s.Close()
	ctx := context.Background()
	goal := []float64{0.7, 0.2, -0.2}
	id, err := s.MovePath(ctx, "hmi", 1, arbiter.PriorityNormal, []Waypoint{
		{Joints: []float64{0.5, 0, 0}},
		{Joints: goal},
	})
	if err != nil {
		t.Fatal(err)
	}
	s.Step(40)
	if err := s.Pause(ctx, id); err != nil {
		t.Fatal(err)
	}
	settleAfterPause(t, s, 10)

	// Drop one feedback cycle while paused: expect a recoverable fault.
	mu.Lock()
	windowOn = true
	mu.Unlock()
	sawFault := false
	for k := 0; k < 5; k++ {
		s.Step(1)
		oc, _ := s.Outcome(id)
		if oc.State == arbiter.StateFaultedRecov {
			sawFault = true
			break
		}
	}
	mu.Lock()
	windowOn = false
	mu.Unlock()
	if !sawFault {
		t.Fatal("expected a recoverable feedback fault during pause")
	}
	// Premature recover is refused until healthy hold window passes.
	if err := s.Recover(ctx); err == nil {
		t.Fatal("premature recover unexpectedly accepted")
	}
	recovered := false
	for k := 0; k < 500; k++ {
		s.Step(1)
		if s.Recover(ctx) == nil {
			recovered = true
			break
		}
	}
	if !recovered {
		t.Fatal("recovery never succeeded")
	}
	// After recovery the command must STILL be paused, not silently moving.
	st, _ := s.Queue(ctx)
	if st.Phase != control.PhasePaused {
		t.Fatalf("after recovery phase=%s, want paused", st.Phase)
	}
	// Explicit resume completes to the original final waypoint.
	if err := s.Resume(ctx, id); err != nil {
		t.Fatal(err)
	}
	oc := runUntil(t, s, id, 40000)
	if oc.State != arbiter.StateCompleted {
		t.Fatalf("post-fault resume: %s %s", oc.State, oc.Reason)
	}
	for i, g := range goal {
		if d := absF(s.Axes()[i].Position - g); d > testCfg().Tolerance.Position {
			t.Fatalf("axis %d final residual %.5f", i, d)
		}
	}
}
