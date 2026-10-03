package motion

import (
	"context"
	"math"
	"testing"

	"github.com/h3yow/robot-motion-controller-go/internal/arbiter"
)

// pathProbe samples the plant every cycle while a path runs and records, per
// waypoint, the first tick at which every axis sits within tolerance at rest.
// After the LAST waypoint has been reached it keeps sampling and fails if any
// earlier waypoint is ever entered again (the path must not turn back).
type pathProbe struct {
	points   [][]float64
	arrivals []int64 // first at-rest arrival tick per waypoint, -1 if never
	tick     int64
	lastHit  int // index of the last waypoint reached so far (-1 none)
	finished bool
}

func newPathProbe(points [][]float64) *pathProbe {
	p := &pathProbe{points: points, arrivals: make([]int64, len(points)), lastHit: -1}
	for i := range p.arrivals {
		p.arrivals[i] = -1
	}
	return p
}

// atRestWithin reports whether all axes are within tolerance of wp and at rest.
func atRestWithin(axes []control0, wp []float64, tolP, tolV float64) bool {
	for ax := range wp {
		if math.Abs(axes[ax].pos-wp[ax]) > tolP || math.Abs(axes[ax].vel) > tolV {
			return false
		}
	}
	return true
}

type control0 struct{ pos, vel float64 }

// sample ingests one cycle of axis feedback.
func (p *pathProbe) sample(t *testing.T, s *Service) {
	t.Helper()
	p.tick++
	st := s.Axes()
	axes := make([]control0, len(st))
	for i, a := range st {
		axes[i] = control0{pos: a.Position, vel: a.Velocity}
	}
	cfg := testCfg()
	for wi, wp := range p.points {
		if p.arrivals[wi] >= 0 {
			continue
		}
		if atRestWithin(axes, wp, cfg.Tolerance.Position, cfg.Tolerance.Velocity) {
			p.arrivals[wi] = p.tick
			if wi != p.lastHit+1 {
				t.Fatalf("tick %d: waypoint %d reached at rest out of order (previous reached index %d)",
					p.tick, wi, p.lastHit)
			}
			p.lastHit = wi
			if wi == len(p.points)-1 {
				p.finished = true
			}
			t.Logf("tick %d: waypoint %d reached at rest for the first time", p.tick, wi)
		}
	}
	if p.finished {
		// The final point has been reached: no earlier waypoint may be
		// entered again for the rest of the motion.
		for wi := 0; wi < len(p.points)-1; wi++ {
			if atRestWithin(axes, p.points[wi], cfg.Tolerance.Position, cfg.Tolerance.Velocity) {
				t.Fatalf("tick %d: waypoint %d re-entered after the final waypoint was reached (no-turnback violated)",
					p.tick, wi)
			}
		}
	}
}

// requireAllReached asserts every waypoint was reached exactly in index order.
func (p *pathProbe) requireAllReached(t *testing.T) {
	t.Helper()
	for i, tk := range p.arrivals {
		if tk < 0 {
			t.Fatalf("waypoint %d (%v) never reached at rest", i, p.points[i])
		}
		if i > 0 && tk <= p.arrivals[i-1] {
			t.Fatalf("waypoint %d first-arrival tick %d not after waypoint %d tick %d",
				i, tk, i-1, p.arrivals[i-1])
		}
	}
	t.Logf("first-arrival ticks strictly increasing: %v", p.arrivals)
}

// TestPathLongChainKeepsOrder submits four waypoints and verifies the axes
// visit them strictly in order, each exactly once: after the final point is
// reached the motion must stop there and never re-enter an earlier point.
func TestPathLongChainKeepsOrder(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	points := [][]float64{
		{0.4, 0.1, 0.0},
		{0.8, 0.3, 0.1},
		{0.5, 0.6, 0.2},
		{0.9, 0.2, -0.1},
	}
	id, info, err := s.PathJoints(context.Background(), "hmi", 1, arbiter.PriorityNormal, points)
	if err != nil {
		t.Fatalf("PathJoints: %v", err)
	}
	if info.TotalPoints != 4 {
		t.Fatalf("TotalPoints=%d want 4", info.TotalPoints)
	}
	probe := newPathProbe(points)
	for k := 0; k < 80000; k++ {
		s.Step(1)
		probe.sample(t, s)
		oc, err := s.Outcome(id)
		if err != nil {
			t.Fatal(err)
		}
		if isTerminal(oc.State) {
			if oc.State != arbiter.StateCompleted {
				t.Fatalf("path ended %s: %s", oc.State, oc.Reason)
			}
			break
		}
	}
	probe.requireAllReached(t)
	// Keep sampling after completion: the axes must stay parked at the last
	// waypoint and never drift back toward an earlier one.
	for k := 0; k < 2000; k++ {
		s.Step(1)
		probe.sample(t, s)
	}
	axes := s.Axes()
	for ax, g := range points[len(points)-1] {
		if d := math.Abs(axes[ax].Position - g); d > testCfg().Tolerance.Position {
			t.Fatalf("axis %d final position %.5f, want %.5f (parked at last waypoint)", ax, axes[ax].Position, g)
		}
	}
	t.Log("after completion the axes remain parked at the final waypoint")
}

// driveToCompletion steps the service, sampling the probe every cycle, until
// the command reaches a terminal state; it requires completion.
func driveToCompletion(t *testing.T, s *Service, id uint64, probe *pathProbe, maxTicks int) {
	t.Helper()
	for k := 0; k < maxTicks; k++ {
		s.Step(1)
		if probe != nil {
			probe.sample(t, s)
		}
		oc, err := s.Outcome(id)
		if err != nil {
			t.Fatal(err)
		}
		if isTerminal(oc.State) {
			if oc.State != arbiter.StateCompleted {
				t.Fatalf("command %d ended %s: %s", id, oc.State, oc.Reason)
			}
			return
		}
	}
	t.Fatalf("command %d did not complete within %d ticks", id, maxTicks)
}

// finalPositions runs the full point sequence uninterrupted and returns the
// final measured joint positions (the reference for interrupted runs).
func finalPositions(t *testing.T, points [][]float64) []float64 {
	t.Helper()
	s := newManual(t)
	defer s.Close()
	id, _, err := s.PathJoints(context.Background(), "hmi", 1, arbiter.PriorityNormal, points)
	if err != nil {
		t.Fatalf("reference PathJoints: %v", err)
	}
	driveToCompletion(t, s, id, nil, 80000)
	return axisPositions(s)
}

// requireSameFinal asserts the interrupted run stopped at the same place as
// the uninterrupted reference (within a small multiple of the tolerance).
func requireSameFinal(t *testing.T, s *Service, ref []float64, label string) {
	t.Helper()
	got := axisPositions(s)
	lim := 4 * testCfg().Tolerance.Position
	for ax := range ref {
		if d := math.Abs(got[ax] - ref[ax]); d > lim {
			t.Fatalf("%s: axis %d final %.6f differs from uninterrupted reference %.6f by %.6f (> %.6f)",
				label, ax, got[ax], ref[ax], d, lim)
		}
	}
	t.Logf("%s: final position matches uninterrupted reference within %.6f rad", label, lim)
}

// TestPathAppendMidRunKeepsOrder appends two waypoints while a three-point
// path is mid-leg; the grown path must still be walked strictly in order,
// each point exactly once, ending parked at the last appended point, and the
// final position must match an uninterrupted run of the same full sequence.
func TestPathAppendMidRunKeepsOrder(t *testing.T) {
	head := [][]float64{
		{0.4, 0.1, 0.0},
		{0.8, 0.3, 0.1},
	}
	tail := [][]float64{
		{0.5, 0.6, 0.2},
		{0.9, 0.2, -0.1},
	}
	full := append(append([][]float64{}, head...), tail...)
	ref := finalPositions(t, full)

	s := newManual(t)
	defer s.Close()
	id, info, err := s.PathJoints(context.Background(), "hmi", 1, arbiter.PriorityNormal, head)
	if err != nil {
		t.Fatalf("PathJoints: %v", err)
	}
	probe := newPathProbe(full)
	// Run partway into the first leg, then append the tail.
	for k := 0; k < 120; k++ {
		s.Step(1)
		probe.sample(t, s)
	}
	info, err = s.AppendJoints(context.Background(), "hmi", 2, id, tail)
	if err != nil {
		t.Fatalf("append mid-run: %v", err)
	}
	if info.TotalPoints != 4 {
		t.Fatalf("TotalPoints after append=%d want 4", info.TotalPoints)
	}
	t.Logf("appended %d waypoints mid-run at tick %d, total=%d", len(tail), probe.tick, info.TotalPoints)
	driveToCompletion(t, s, id, probe, 80000)
	probe.requireAllReached(t)
	// Post-completion: no drift back toward earlier waypoints.
	for k := 0; k < 1000; k++ {
		s.Step(1)
		probe.sample(t, s)
	}
	requireSameFinal(t, s, ref, "append mid-run")
}

// TestPathQueuedActivationKeepsOrder queues a path behind a blocking move,
// appends more waypoints while it is still queued, and verifies that once
// activated the path walks every point strictly in order, never turns back,
// and stops at the same final position as an uninterrupted run.
func TestPathQueuedActivationKeepsOrder(t *testing.T) {
	head := [][]float64{
		{0.3, 0.2, 0.1},
		{0.7, 0.4, 0.0},
	}
	tail := [][]float64{
		{0.2, 0.5, 0.3},
		{0.6, 0.1, -0.2},
	}
	full := append(append([][]float64{}, head...), tail...)
	ref := finalPositions(t, full)

	s := newManual(t)
	defer s.Close()
	ctx := context.Background()
	blocker, err := s.MoveJoints(ctx, "a", 1, arbiter.PriorityNormal, []float64{0.9, 0, 0})
	if err != nil {
		t.Fatal(err)
	}
	pid, _, err := s.PathJoints(ctx, "b", 1, arbiter.PriorityLow, head)
	if err != nil {
		t.Fatal(err)
	}
	st, err := s.Queue(ctx)
	if err != nil {
		t.Fatal(err)
	}
	if st.ActiveID != blocker || len(st.QueuedIDs) != 1 || st.QueuedIDs[0] != pid {
		t.Fatalf("queue snapshot before activation: %+v", st)
	}
	if _, err := s.AppendJoints(ctx, "b", 2, pid, tail); err != nil {
		t.Fatalf("append while queued: %v", err)
	}
	t.Log("path queued behind blocker and grown to 4 waypoints while queued")

	// The blocker runs first; the queued path must not move the axes.
	driveToCompletion(t, s, blocker, nil, 40000)
	s.Step(1) // queued commands activate on the cycle after the slot frees
	st, _ = s.Queue(ctx)
	if st.ActiveID != pid {
		t.Fatalf("after blocker completion active=%d want queued path %d", st.ActiveID, pid)
	}
	t.Logf("blocker done; queued path %d activated", pid)

	probe := newPathProbe(full)
	driveToCompletion(t, s, pid, probe, 80000)
	probe.requireAllReached(t)
	for k := 0; k < 1000; k++ {
		s.Step(1)
		probe.sample(t, s)
	}
	requireSameFinal(t, s, ref, "queued activation")
}

// TestPathPauseResumeKeepsOrder pauses a four-point path in the middle of a
// leg, holds it, resumes, and verifies the walk still visits every point
// strictly in order, never turns back, and stops where an uninterrupted run
// stops. While running it also checks the reported progress (ReachedPoints)
// never decreases and never claims a point the probe has not yet observed.
func TestPathPauseResumeKeepsOrder(t *testing.T) {
	points := [][]float64{
		{0.4, 0.1, 0.0},
		{0.8, 0.3, 0.1},
		{0.5, 0.6, 0.2},
		{0.9, 0.2, -0.1},
	}
	ref := finalPositions(t, points)

	s := newManual(t)
	defer s.Close()
	ctx := context.Background()
	id, _, err := s.PathJoints(ctx, "hmi", 1, arbiter.PriorityNormal, points)
	if err != nil {
		t.Fatalf("PathJoints: %v", err)
	}
	probe := newPathProbe(points)
	lastReported := 0
	checkProgress := func() {
		t.Helper()
		st, err := s.Queue(ctx)
		if err != nil {
			t.Fatal(err)
		}
		if st.ActivePath == nil {
			return
		}
		rep := st.ActivePath.ReachedPoints
		if rep < lastReported {
			t.Fatalf("tick %d: ReachedPoints decreased %d -> %d", probe.tick, lastReported, rep)
		}
		observed := 0
		for _, tk := range probe.arrivals {
			if tk >= 0 {
				observed++
			}
		}
		if rep > observed {
			t.Fatalf("tick %d: ReachedPoints=%d claims more waypoints than the %d observed at rest",
				probe.tick, rep, observed)
		}
		lastReported = rep
	}

	// Run until the first waypoint is reached and the second leg is underway.
	for probe.lastHit < 0 {
		s.Step(1)
		probe.sample(t, s)
		checkProgress()
	}
	for k := 0; k < 80; k++ {
		s.Step(1)
		probe.sample(t, s)
		checkProgress()
	}
	ok, err := s.Pause(ctx)
	if err != nil || !ok {
		t.Fatalf("Pause: ok=%v err=%v", ok, err)
	}
	// Let the ramp settle into the hold, then confirm the axes do not drift.
	for k := 0; k < 400; k++ {
		s.Step(1)
	}
	st, _ := s.Queue(ctx)
	if !st.Paused {
		t.Fatalf("expected paused latch set, queue=%+v", st)
	}
	held := axisPositions(s)
	for k := 0; k < 300; k++ {
		s.Step(1)
	}
	for ax := range held {
		if d := math.Abs(axisPositions(s)[ax] - held[ax]); d > testCfg().Tolerance.Position {
			t.Fatalf("axis %d drifted %.6f rad while paused", ax, d)
		}
	}
	t.Logf("paused mid-leg at tick %d and held without drift", probe.tick)

	if err := s.Resume(ctx); err != nil {
		t.Fatalf("Resume: %v", err)
	}
	for {
		s.Step(1)
		probe.sample(t, s)
		checkProgress()
		oc, err := s.Outcome(id)
		if err != nil {
			t.Fatal(err)
		}
		if isTerminal(oc.State) {
			if oc.State != arbiter.StateCompleted {
				t.Fatalf("path ended %s: %s", oc.State, oc.Reason)
			}
			break
		}
		if probe.tick > 90000 {
			t.Fatalf("path did not complete after resume (tick %d)", probe.tick)
		}
	}
	probe.requireAllReached(t)
	st, _ = s.Queue(ctx)
	if st.ActivePath != nil && st.ActivePath.ReachedPoints != len(points) {
		t.Fatalf("final ReachedPoints=%d want %d", st.ActivePath.ReachedPoints, len(points))
	}
	for k := 0; k < 1000; k++ {
		s.Step(1)
		probe.sample(t, s)
	}
	requireSameFinal(t, s, ref, "pause/resume")
}

// TestShortPathsStillRun guards the fix against regressions on one- and
// two-point paths: they must move, complete, and stay parked at the last
// point without extra excursions.
func TestShortPathsStillRun(t *testing.T) {
	for _, points := range [][][]float64{
		{{0.5, 0.2, 0.1}},
		{{0.5, 0.2, 0.1}, {0.1, -0.3, 0.2}},
	} {
		s := newManual(t)
		id, _, err := s.PathJoints(context.Background(), "hmi", 1, arbiter.PriorityNormal, points)
		if err != nil {
			t.Fatalf("PathJoints(%d points): %v", len(points), err)
		}
		probe := newPathProbe(points)
		driveToCompletion(t, s, id, probe, 40000)
		probe.requireAllReached(t)
		for k := 0; k < 500; k++ {
			s.Step(1)
			probe.sample(t, s)
		}
		axes := s.Axes()
		last := points[len(points)-1]
		for ax, g := range last {
			if d := math.Abs(axes[ax].Position - g); d > testCfg().Tolerance.Position {
				t.Fatalf("%d-point path: axis %d final %.5f want %.5f", len(points), ax, axes[ax].Position, g)
			}
		}
		t.Logf("%d-point path completed and parked at the final point", len(points))
		s.Close()
	}
}
