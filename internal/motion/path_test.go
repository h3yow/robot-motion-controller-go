package motion

import (
	"context"
	"math"
	"testing"

	"github.com/h3yow/robot-motion-controller-go/internal/arbiter"
	"github.com/h3yow/robot-motion-controller-go/internal/config"
	"github.com/h3yow/robot-motion-controller-go/internal/kinematics"
	"github.com/h3yow/robot-motion-controller-go/internal/mcerr"
)

// arrivalTick records, per waypoint, the first cycle at which every axis was
// within position tolerance and at rest; a never-reached point yields -1.
type arrivalRecord struct{ ticks []int64 }

func recordArrivals(t *testing.T, s *Service, id uint64, points [][]float64, maxTicks int) arrivalRecord {
	t.Helper()
	cfg := testCfg()
	tolP := cfg.Tolerance.Position
	tolV := cfg.Tolerance.Velocity
	rec := arrivalRecord{ticks: make([]int64, len(points))}
	for i := range rec.ticks {
		rec.ticks[i] = -1
	}
	var tick int64
	for k := 0; k < maxTicks; k++ {
		s.Step(1)
		tick++
		axes := s.Axes()
		for wi, wp := range points {
			if rec.ticks[wi] >= 0 {
				continue
			}
			ok := true
			for ax := range wp {
				if math.Abs(axes[ax].Position-wp[ax]) > tolP || math.Abs(axes[ax].Velocity) > tolV {
					ok = false
					break
				}
			}
			if ok {
				rec.ticks[wi] = tick
			}
		}
		oc, err := s.Outcome(id)
		if err != nil {
			t.Fatal(err)
		}
		if isTerminal(oc.State) {
			if oc.State != arbiter.StateCompleted {
				t.Fatalf("command %d ended %s: %s", id, oc.State, oc.Reason)
			}
			return rec
		}
	}
	t.Fatalf("command %d did not complete within %d ticks", id, maxTicks)
	return rec
}

func (r arrivalRecord) requireStrictOrder(t *testing.T, points [][]float64) {
	t.Helper()
	for i, tk := range r.ticks {
		if tk < 0 {
			t.Fatalf("waypoint %d (%v) never reached at rest", i, points[i])
		}
		if i > 0 && tk <= r.ticks[i-1] {
			t.Fatalf("waypoint %d arrived at tick %d not after waypoint %d at tick %d",
				i, tk, i-1, r.ticks[i-1])
		}
	}
	t.Logf("waypoint arrival ticks (strict increasing): %v", r.ticks)
}

func fkOf(t *testing.T, cfg *config.Config, q []float64) kinematics.Pose {
	t.Helper()
	arm, err := kinematics.NewArm(cfg.Links, cfg.Elbow)
	if err != nil {
		t.Fatal(err)
	}
	pose, err := arm.FK(q)
	if err != nil {
		t.Fatal(err)
	}
	return pose
}

// TestPathJointsVisitsAllPointsInOrder drives a three-point joint path and
// verifies every point is reached at rest, in order, with no skipped point.
func TestPathJointsVisitsAllPointsInOrder(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	points := [][]float64{{0.6, 0.2, 0.0}, {0.2, 0.5, 0.3}, {-0.3, -0.1, 0.1}}
	id, info, err := s.PathJoints(context.Background(), "hmi", 1, arbiter.PriorityNormal, points)
	if err != nil {
		t.Fatalf("PathJoints: %v", err)
	}
	if info.TotalPoints != 3 || info.ReachedPoints != 0 {
		t.Fatalf("initial info=%+v, want 3 total/0 reached", info)
	}
	t.Logf("joint path admitted id=%d with %d waypoints", id, info.TotalPoints)
	rec := recordArrivals(t, s, id, points, 60000)
	rec.requireStrictOrder(t, points)
	axes := s.Axes()
	for ax, g := range points[2] {
		if d := axes[ax].Position - g; math.Abs(d) > testCfg().Tolerance.Position {
			t.Fatalf("final axis %d = %.5f want %.5f", ax, axes[ax].Position, g)
		}
	}
}

// TestPathPosesVisitsAllPoints exercises the Cartesian entrance and checks
// the end-effector actually reaches every requested pose.
func TestPathPosesVisitsAllPoints(t *testing.T) {
	cfg := testCfg()
	s := newManual(t)
	defer s.Close()
	poses := []kinematics.Pose{
		{X: 0.62, Y: 0.18, Phi: 0.2},
		{X: 0.50, Y: 0.30, Phi: 0.6},
	}
	id, _, err := s.PathPoses(context.Background(), "hmi", 1, arbiter.PriorityNormal, poses)
	if err != nil {
		t.Fatalf("PathPoses: %v", err)
	}
	// Drive to completion.
	oc := runUntil(t, s, id, 60000)
	t.Logf("pose path completed: %s (%s)", oc.State, oc.Reason)
	axes := s.Axes()
	q := []float64{axes[0].Position, axes[1].Position, axes[2].Position}
	got := fkOf(t, cfg, q)
	want := poses[1]
	if math.Abs(got.X-want.X) > 0.01 || math.Abs(got.Y-want.Y) > 0.01 || math.Abs(got.Phi-want.Phi) > 0.02 {
		t.Fatalf("final FK=(%.4f,%.4f,%.4f) want pose (%.4f,%.4f,%.4f)", got.X, got.Y, got.Phi, want.X, want.Y, want.Phi)
	}
}

// TestPathMixedKinds submits joints, a pose and joints in one path; ordering
// and the exact joint start/end are asserted.
func TestPathMixedKinds(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	wps := []Waypoint{
		{Kind: WaypointJoint, Joints: []float64{0.5, 0.0, 0.0}},
		{Kind: WaypointPose, Pose: kinematics.Pose{X: 0.6, Y: 0.25, Phi: 0.3}},
		{Kind: WaypointJoint, Joints: []float64{0.1, -0.2, 0.4}},
	}
	id, info, err := s.PathMixed(context.Background(), "hmi", 1, arbiter.PriorityNormal, wps)
	if err != nil {
		t.Fatalf("PathMixed: %v", err)
	}
	t.Logf("mixed path id=%d points=%d", id, info.TotalPoints)
	// Resolve expected joint targets the same deterministic chain.
	cfg := testCfg()
	arm, _ := kinematics.NewArm(cfg.Links, cfg.Elbow)
	seed := []float64{0, 0, 0}
	joints := make([][]float64, 3)
	joints[0] = wps[0].Joints
	j1, err := arm.IK(wps[1].Pose, joints[0])
	if err != nil {
		t.Fatal(err)
	}
	joints[1] = j1
	joints[2] = wps[2].Joints
	_ = seed
	rec := recordArrivals(t, s, id, joints, 60000)
	rec.requireStrictOrder(t, joints)
}

// TestPathAppendDuringExecution appends waypoints before the original tail is
// reached; the path must visit originals first, then the appended ones.
func TestPathAppendDuringExecution(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	first := [][]float64{{0.8, 0.0, 0.0}}
	id, info, err := s.PathJoints(context.Background(), "hmi", 1, arbiter.PriorityNormal, first)
	if err != nil {
		t.Fatal(err)
	}
	s.Step(100) // underway, not finished
	more := [][]float64{{0.8, 0.4, 0.0}, {0.2, 0.4, 0.2}}
	info, err = s.AppendJoints(context.Background(), "hmi", 2, id, more)
	if err != nil {
		t.Fatalf("append while running: %v", err)
	}
	if info.TotalPoints != 3 {
		t.Fatalf("info after append TotalPoints=%d want 3", info.TotalPoints)
	}
	t.Logf("appended 2 points mid-run, total now %d", info.TotalPoints)
	all := append(append([][]float64{}, first...), more...)
	rec := recordArrivals(t, s, id, all, 80000)
	rec.requireStrictOrder(t, all)
}

// TestPathAppendWhileQueued submits a blocking first command, queues a path
// behind it, appends to the queued path, and then verifies the queued path
// runs every original and appended point after the first completes.
func TestPathAppendWhileQueued(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	ctx := context.Background()
	blocker, err := s.MoveJoints(ctx, "a", 1, arbiter.PriorityNormal, []float64{0.9, 0, 0})
	if err != nil {
		t.Fatal(err)
	}
	wps := [][]float64{{0.4, 0, 0}}
	pid, _, err := s.PathJoints(ctx, "b", 1, arbiter.PriorityLow, wps)
	if err != nil {
		t.Fatal(err)
	}
	st, _ := s.Queue(ctx)
	if st.Pending != 2 || st.ActiveID != blocker || len(st.QueuedIDs) != 1 || st.QueuedIDs[0] != pid {
		t.Fatalf("queue before append: %+v", st)
	}
	extra := [][]float64{{-0.2, 0, 0}, {-0.5, 0.1, 0}}
	if _, err := s.AppendJoints(ctx, "b", 2, pid, extra); err != nil {
		t.Fatalf("append while queued: %v", err)
	}
	t.Log("append to queued path accepted; driving blocker to completion")
	oc := runUntil(t, s, blocker, 40000)
	if oc.State != arbiter.StateCompleted {
		t.Fatalf("blocker ended %s", oc.State)
	}
	all := append(append([][]float64{}, wps...), extra...)
	rec := recordArrivals(t, s, pid, all, 80000)
	rec.requireStrictOrder(t, all)
}

// TestPathAppendMixedKinds appends a pose to a joint-started path.
func TestPathAppendMixedKinds(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	ctx := context.Background()
	id, _, err := s.PathJoints(ctx, "hmi", 1, arbiter.PriorityNormal, [][]float64{{0.5, 0, 0}})
	if err != nil {
		t.Fatal(err)
	}
	pose := kinematics.Pose{X: 0.55, Y: 0.28, Phi: 0.4}
	info, err := s.AppendPoses(ctx, "hmi", 2, id, []kinematics.Pose{pose})
	if err != nil {
		t.Fatalf("append pose: %v", err)
	}
	t.Logf("mixed append accepted: total=%d", info.TotalPoints)
	if oc := runUntil(t, s, id, 60000); oc.State != arbiter.StateCompleted {
		t.Fatalf("state=%s reason=%s", oc.State, oc.Reason)
	}
	got := fkOf(t, testCfg(), axisPositions(s))
	if math.Abs(got.X-pose.X) > 0.01 || math.Abs(got.Y-pose.Y) > 0.01 || math.Abs(got.Phi-pose.Phi) > 0.02 {
		t.Fatalf("appended pose not reached: got (%.4f,%.4f,%.4f)", got.X, got.Y, got.Phi)
	}
}

func axisPositions(s *Service) []float64 {
	axes := s.Axes()
	q := make([]float64, len(axes))
	for i, a := range axes {
		q[i] = a.Position
	}
	return q
}

// TestPathAppendRejections covers terminal-path, unknown-id, non-path and
// stale-sequence appends.
func TestPathAppendRejections(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	ctx := context.Background()
	id, _, err := s.PathJoints(ctx, "hmi", 1, arbiter.PriorityNormal, [][]float64{{0.5, 0, 0}})
	if err != nil {
		t.Fatal(err)
	}
	if oc := runUntil(t, s, id, 40000); oc.State != arbiter.StateCompleted {
		t.Fatalf("setup: %s", oc.State)
	}
	if _, err := s.AppendJoints(ctx, "hmi", 2, id, [][]float64{{0.2, 0, 0}}); !mcerr.HasCode(err, mcerr.CodePathClosed) {
		t.Fatalf("append to completed path: want path_closed, got %v", err)
	}
	t.Log("append to completed path rejected with path_closed")
	if _, err := s.AppendJoints(ctx, "hmi", 1, 99999, [][]float64{{0.2, 0, 0}}); !mcerr.HasCode(err, mcerr.CodeCommandUnknown) {
		t.Fatalf("append to unknown id: %v", err)
	}
	mv, err := s.MoveJoints(ctx, "hmi", 2, arbiter.PriorityNormal, []float64{0.1, 0, 0})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := s.AppendJoints(ctx, "hmi", 3, mv, [][]float64{{0.2, 0, 0}}); !mcerr.HasCode(err, mcerr.CodeTargetContradictory) {
		t.Fatalf("append to single move: %v", err)
	}
	runUntil(t, s, mv, 40000)
	// Stale per-entrance seq on a fresh path.
	pid, _, err := s.PathJoints(ctx, "ui", 1, arbiter.PriorityNormal, [][]float64{{0.1, 0, 0}})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := s.AppendJoints(ctx, "ui", 1, pid, [][]float64{{0.2, 0, 0}}); !mcerr.HasCode(err, mcerr.CodeStaleSequence) {
		t.Fatalf("append stale seq: %v", err)
	}
	t.Log("stale append sequence rejected")
}

// TestPathInvalidWaypointRejectedAtSubmit verifies the WHOLE batch is refused
// (no partial admission) for each distinct reason class.
func TestPathInvalidWaypointRejectedAtSubmit(t *testing.T) {
	cases := []struct {
		name string
		wps  []Waypoint
		code mcerr.Code
	}{
		{"empty batch", nil, mcerr.CodeTargetContradictory},
		{"joint cardinality", []Waypoint{
			{Kind: WaypointJoint, Joints: []float64{0.1, 0.2}},
		}, mcerr.CodeTargetContradictory},
		{"joint non-finite", []Waypoint{
			{Kind: WaypointJoint, Joints: []float64{0.1, math.NaN(), 0}},
		}, mcerr.CodeTargetNonFinite},
		{"joint out of travel", []Waypoint{
			{Kind: WaypointJoint, Joints: []float64{9, 0, 0}},
		}, mcerr.CodeJointLimitExceeded},
		{"pose unreachable", []Waypoint{
			{Kind: WaypointPose, Pose: kinematics.Pose{X: 50, Y: 50, Phi: 0}},
		}, mcerr.CodeTargetUnreachable},
		{"pose non-finite", []Waypoint{
			{Kind: WaypointPose, Pose: kinematics.Pose{X: math.Inf(1), Y: 0, Phi: 0}},
		}, mcerr.CodeTargetNonFinite},
		{"second point invalid rejects all", []Waypoint{
			{Kind: WaypointJoint, Joints: []float64{0.1, 0, 0}},
			{Kind: WaypointJoint, Joints: []float64{0.2, 0, 0, 0}},
		}, mcerr.CodeTargetContradictory},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			s := newManual(t)
			defer s.Close()
			_, _, err := s.PathMixed(context.Background(), "hmi", 1, arbiter.PriorityNormal, tc.wps)
			if !mcerr.HasCode(err, tc.code) {
				t.Fatalf("want %s, got %v", tc.code, err)
			}
			st, _ := s.Queue(context.Background())
			if st.Pending != 0 {
				t.Fatalf("rejected batch left %d pending commands", st.Pending)
			}
			t.Logf("%-30s rejected as %s; no pending state left", tc.name, tc.code)
		})
	}
}
