package motion

import (
	"context"
	"fmt"
	"math"
	"runtime"
	"sync"
	"testing"

	"github.com/h3yow/robot-motion-controller-go/internal/arbiter"
)

// tracePoint is one sampled cycle of the driven axes.
type tracePoint struct {
	tick int
	q    []float64
	v    []float64
}

// waypointVisitSeq turns a sampled trace into the compressed sequence of
// waypoint visits: for every cycle the set of waypoints at which every axis
// sits within position tolerance AND at rest is recorded; consecutive cycles
// at the same single waypoint collapse into one event. A correct ordered path
// yields exactly [0 1 ... m-1] with no duplicate and no later event; a path
// that rushes to the tail and doubles back produces repeated indices.
func waypointVisitSeq(t *testing.T, tr []tracePoint, points [][]float64) []int {
	t.Helper()
	cfg := testCfg()
	tolP, tolV := cfg.Tolerance.Position, cfg.Tolerance.Velocity
	var seq []int
	last := -2
	for _, s := range tr {
		at := -1
		for wi, wp := range points {
			ok := true
			for ax := range wp {
				if math.Abs(s.q[ax]-wp[ax]) > tolP || math.Abs(s.v[ax]) > tolV {
					ok = false
					break
				}
			}
			if ok {
				if at != -1 {
					t.Fatalf("tick %d: within tolerance of two waypoints %d and %d; "+
						"test points must be well separated", s.tick, at, wi)
				}
				at = wi
			}
		}
		if at != -1 && at != last {
			seq = append(seq, at)
			last = at
		}
		if at == -1 {
			// Between waypoints: only collapse consecutive same-point runs;
			// re-entering the same point later must be a new (duplicate) event.
			last = -2
		}
	}
	return seq
}

// tracePath samples every cycle until the path is terminal, then keeps
// sampling extraHold cycles to prove the axes never move again after the last
// waypoint.
func tracePath(t *testing.T, s *Service, id uint64, points [][]float64, maxTicks, extraHold int) []tracePoint {
	t.Helper()
	tr := make([]tracePoint, 0, maxTicks)
	terminal := false
	for k := 0; k < maxTicks; k++ {
		s.Step(1)
		axes := s.Axes()
		q := make([]float64, len(axes))
		v := make([]float64, len(axes))
		for i, a := range axes {
			q[i], v[i] = a.Position, a.Velocity
		}
		tr = append(tr, tracePoint{tick: k + 1, q: q, v: v})
		oc, err := s.Outcome(id)
		if err != nil {
			t.Fatal(err)
		}
		if isTerminal(oc.State) {
			if oc.State != arbiter.StateCompleted {
				t.Fatalf("command %d ended %s: %s", id, oc.State, oc.Reason)
			}
			terminal = true
			break
		}
	}
	if !terminal {
		t.Fatalf("command %d did not complete within %d ticks", id, maxTicks)
	}
	for k := 0; k < extraHold; k++ {
		s.Step(1)
		axes := s.Axes()
		q := make([]float64, len(axes))
		v := make([]float64, len(axes))
		for i, a := range axes {
			q[i], v[i] = a.Position, a.Velocity
		}
		tr = append(tr, tracePoint{tick: len(tr) + 1, q: q, v: v})
	}
	return tr
}

func requireVisitSeq(t *testing.T, tr []tracePoint, points [][]float64) {
	t.Helper()
	seq := waypointVisitSeq(t, tr, points)
	want := make([]int, len(points))
	for i := range want {
		want[i] = i
	}
	if len(seq) != len(want) {
		t.Fatalf("waypoint visit sequence = %v, want %v (repeated/skipped waypoints)", seq, want)
	}
	for i := range want {
		if seq[i] != want[i] {
			t.Fatalf("waypoint visit sequence = %v, want %v", seq, want)
		}
	}
	t.Logf("waypoint visit sequence (each point once, in order, nothing after last): %v", seq)
}

// TestWaypointOrderLongPath submits five waypoints forming a zig-zag and
// asserts the axes visit each exactly once in order and never move again
// after reaching the final one.
func TestWaypointOrderLongPath(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	points := [][]float64{
		{0.50, 0.30, 0.10},
		{0.90, 0.30, 0.10},
		{0.90, -0.30, 0.40},
		{0.20, -0.30, 0.40},
		{0.20, 0.40, -0.30},
	}
	id, info, err := s.PathJoints(context.Background(), "hmi", 1, arbiter.PriorityNormal, points)
	if err != nil {
		t.Fatalf("PathJoints: %v", err)
	}
	if info.TotalPoints != 5 || info.ReachedPoints != 0 {
		t.Fatalf("initial info=%+v", info)
	}
	t.Logf("5-point zig-zag path admitted id=%d", id)
	tr := tracePath(t, s, id, points, 200000, 500)
	requireVisitSeq(t, tr, points)
}

// firstArrivals returns the first sampled tick at which each waypoint is
// reached at rest (-1 when never reached).
func firstArrivals(t *testing.T, tr []tracePoint, points [][]float64) []int {
	t.Helper()
	cfg := testCfg()
	tolP, tolV := cfg.Tolerance.Position, cfg.Tolerance.Velocity
	out := make([]int, len(points))
	for i := range out {
		out[i] = -1
	}
	for _, s := range tr {
		for wi, wp := range points {
			if out[wi] >= 0 {
				continue
			}
			ok := true
			for ax := range wp {
				if math.Abs(s.q[ax]-wp[ax]) > tolP || math.Abs(s.v[ax]) > tolV {
					ok = false
					break
				}
			}
			if ok {
				out[wi] = s.tick
			}
		}
	}
	return out
}

func finalPositions(tr []tracePoint) []float64 {
	last := tr[len(tr)-1]
	return append([]float64(nil), last.q...)
}

// TestWaypointOrderAppendMidRun submits three points, appends three more
// while the first leg is underway, and asserts all six are visited once in
// order with no backtracking to a point already passed.
func TestWaypointOrderAppendMidRun(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	ctx := context.Background()
	first := [][]float64{
		{0.80, 0.20, 0.00},
		{0.80, 0.50, 0.00},
		{0.30, 0.50, 0.30},
	}
	id, info, err := s.PathJoints(ctx, "hmi", 1, arbiter.PriorityNormal, first)
	if err != nil {
		t.Fatal(err)
	}
	t.Logf("scenario=append-mid-run: path id=%d admitted with %d points", id, info.TotalPoints)
	// Drive onto the first leg but stop well before its end.
	s.Step(40)
	extra := [][]float64{
		{0.30, 0.00, 0.30},
		{-0.30, 0.00, 0.00},
		{-0.30, 0.40, -0.20},
	}
	info, err = s.AppendJoints(ctx, "hmi", 2, id, extra)
	if err != nil {
		t.Fatalf("append mid-run: %v", err)
	}
	if info.TotalPoints != 6 {
		t.Fatalf("TotalPoints after append=%d want 6", info.TotalPoints)
	}
	t.Logf("scenario=append-mid-run: appended 3 points mid-run, total=%d", info.TotalPoints)
	all := append(append([][]float64{}, first...), extra...)
	tr := tracePath(t, s, id, all, 200000, 500)
	requireVisitSeq(t, tr, all)
	arrivals := firstArrivals(t, tr, all)
	t.Logf("scenario=append-mid-run: result=pass first-arrival ticks=%v", arrivals)
}

// TestWaypointOrderQueuedActivation leaves a path waiting in the queue behind
// a blocker, appends to it while queued, then asserts strict in-order visits
// after activation - the queued path must plan from its activation position
// and never replay anything.
func TestWaypointOrderQueuedActivation(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	ctx := context.Background()
	blocker, err := s.MoveJoints(ctx, "a", 1, arbiter.PriorityNormal, []float64{0.9, 0.0, 0.0})
	if err != nil {
		t.Fatal(err)
	}
	queued := [][]float64{
		{0.50, 0.40, 0.10},
		{0.10, 0.40, 0.10},
		{0.10, -0.20, 0.40},
	}
	pid, info, err := s.PathJoints(ctx, "b", 1, arbiter.PriorityLow, queued)
	if err != nil {
		t.Fatal(err)
	}
	t.Logf("scenario=queued-activation: path id=%d queued behind blocker id=%d", pid, blocker)
	extra := [][]float64{
		{-0.40, -0.20, 0.40},
		{-0.40, 0.30, -0.20},
	}
	if info, err = s.AppendJoints(ctx, "b", 2, pid, extra); err != nil {
		t.Fatalf("append while queued: %v", err)
	}
	st, _ := s.Queue(ctx)
	if st.ActiveID != blocker || len(st.QueuedIDs) != 1 || st.QueuedIDs[0] != pid {
		t.Fatalf("queue snapshot before activation: %+v", st)
	}
	t.Logf("scenario=queued-activation: appended 2 points while queued, total=%d; waiting for activation", info.TotalPoints)
	oc := runUntil(t, s, blocker, 40000)
	if oc.State != arbiter.StateCompleted {
		t.Fatalf("blocker ended %s", oc.State)
	}
	all := append(append([][]float64{}, queued...), extra...)
	tr := tracePath(t, s, pid, all, 200000, 500)
	requireVisitSeq(t, tr, all)
	t.Log("scenario=queued-activation: result=pass strict order after queue activation")
}

// TestWaypointOrderPauseResume interrupts a four-point path after the first
// point has been reached, holds it, and verifies: (a) no already-reached
// point is replayed on resume, (b) every point is visited once in order,
// (c) the final stop equals the uninterrupted run, (d) a second service
// replaying the exact same interruption reproduces the identical trace.
func TestWaypointOrderPauseResume(t *testing.T) {
	points := [][]float64{
		{0.70, 0.00, 0.00},
		{0.70, 0.50, 0.00},
		{0.20, 0.50, 0.30},
		{0.20, -0.20, 0.30},
	}

	// Reference: uninterrupted execution.
	ref := newManual(t)
	defer ref.Close()
	rid, _, err := ref.PathJoints(context.Background(), "hmi", 1, arbiter.PriorityNormal, points)
	if err != nil {
		t.Fatal(err)
	}
	refTrace := tracePath(t, ref, rid, points, 200000, 0)
	refFinal := finalPositions(refTrace)
	t.Logf("scenario=pause-resume: uninterrupted run final=%v", refFinal)

	// Interrupted run: the ENTIRE run is sampled from cycle 1 (including the
	// pause ramp and hold), so the trace proves point 0 is never revisited
	// after resume. The pause is scheduled at a fixed offset after point 0 is
	// first observed, making the interruption replayable across services.
	runInterrupted := func() []tracePoint {
		s := newManual(t)
		defer s.Close()
		ctx := context.Background()
		id, _, err := s.PathJoints(ctx, "hmi", 1, arbiter.PriorityNormal, points)
		if err != nil {
			t.Fatal(err)
		}
		sample := func() tracePoint {
			axes := s.Axes()
			q := make([]float64, len(axes))
			v := make([]float64, len(axes))
			for i, a := range axes {
				q[i], v[i] = a.Position, a.Velocity
			}
			return tracePoint{tick: 0, q: q, v: v}
		}
		tr := make([]tracePoint, 0, 4096)
		add := func(tp tracePoint) { tp.tick = len(tr) + 1; tr = append(tr, tp) }
		const pauseOffset = 30 // cycles after first observing point 0
		const holdTicks = 400  // pause ramp settle + hold
		pauseTick := -1
		arrived0Tick := -1
		resumed := false
		paused := false
		for k := 0; k < 200000; k++ {
			s.Step(1)
			add(sample())
			axes := s.Axes()
			at0 := true
			for ax, g := range points[0] {
				if math.Abs(axes[ax].Position-g) > testCfg().Tolerance.Position ||
					math.Abs(axes[ax].Velocity) > testCfg().Tolerance.Velocity {
					at0 = false
				}
			}
			if at0 && arrived0Tick == -1 {
				arrived0Tick = k
				pauseTick = k + pauseOffset
			}
			if !paused && pauseTick >= 0 && k >= pauseTick {
				if ok, err := s.Pause(ctx); err != nil || !ok {
					t.Fatalf("Pause: ok=%v err=%v", ok, err)
				}
				paused = true
				for h := 0; h < holdTicks; h++ {
					s.Step(1)
					add(sample())
				}
				st, _ := s.Queue(ctx)
				if !st.Paused || st.ActivePath == nil {
					t.Fatalf("paused snapshot wrong: %+v", st)
				}
				rp := st.ActivePath.ReachedPoints
				if rp != 1 {
					t.Fatalf("ReachedPoints during pause=%d want 1 (point 0 already done)", rp)
				}
				t.Logf("scenario=pause-resume: paused %d cycles after point 0 (tick %d), ReachedPoints=%d",
					pauseOffset, arrived0Tick+1, rp)
				if err := s.Resume(ctx); err != nil {
					t.Fatalf("Resume: %v", err)
				}
				resumed = true
			}
			oc, err := s.Outcome(id)
			if err != nil {
				t.Fatal(err)
			}
			if isTerminal(oc.State) {
				if oc.State != arbiter.StateCompleted {
					t.Fatalf("command ended %s: %s", oc.State, oc.Reason)
				}
				break
			}
		}
		if !resumed {
			t.Fatal("never paused/resumed")
		}
		// Extra samples after completion prove the axes stay put at the last
		// point and do not swing back through any earlier one.
		for h := 0; h < 500; h++ {
			s.Step(1)
			add(sample())
		}
		return tr
	}

	tr1 := runInterrupted()
	requireVisitSeq(t, tr1, points)
	fin1 := finalPositions(tr1)
	for ax := range points[3] {
		if d := math.Abs(fin1[ax] - refFinal[ax]); d > testCfg().Tolerance.Position {
			t.Fatalf("interrupted final axis %d=%.6f differs from uninterrupted %.6f by %.6f",
				ax, fin1[ax], refFinal[ax], d)
		}
	}
	t.Logf("scenario=pause-resume: interrupted final=%v matches uninterrupted run", fin1)

	// Same inputs (same fixed pause offset) must reproduce the same result.
	tr2 := runInterrupted()
	requireVisitSeq(t, tr2, points)
	fin2 := finalPositions(tr2)
	for ax := range fin1 {
		if fin2[ax] != fin1[ax] {
			t.Fatalf("replay not reproducible axis %d: %.12f vs %.12f", ax, fin2[ax], fin1[ax])
		}
	}
	if a1, a2 := firstArrivals(t, tr1, points), firstArrivals(t, tr2, points); len(a1) != len(a2) {
		t.Fatal("replay arrival count differs")
	} else {
		for i := range a1 {
			if a1[i] != a2[i] {
				t.Fatalf("replay arrival ticks differ at point %d: %d vs %d", i, a1[i], a2[i])
			}
		}
		t.Logf("scenario=pause-resume: result=pass replay arrivals identical %v", a1)
	}
}

// TestWaypointOrderShortPaths ensures one- and two-point paths still execute
// (no no-op, no extra leg) after the per-leg execution change.
func TestWaypointOrderShortPaths(t *testing.T) {
	cases := []struct {
		name   string
		points [][]float64
	}{
		{"one point", [][]float64{{0.6, -0.2, 0.3}}},
		{"two points", [][]float64{{0.0, 0.5, 0.0}, {-0.4, 0.5, -0.2}}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			s := newManual(t)
			defer s.Close()
			id, info, err := s.PathJoints(context.Background(), "hmi", 1, arbiter.PriorityNormal, tc.points)
			if err != nil {
				t.Fatal(err)
			}
			if info.TotalPoints != len(tc.points) || info.ReachedPoints != 0 {
				t.Fatalf("initial info=%+v", info)
			}
			tr := tracePath(t, s, id, tc.points, 200000, 200)
			requireVisitSeq(t, tr, tc.points)
			fin := finalPositions(tr)
			for ax, g := range tc.points[len(tc.points)-1] {
				if d := math.Abs(fin[ax] - g); d > testCfg().Tolerance.Position {
					t.Fatalf("%s final axis %d=%.5f want %.5f", tc.name, ax, fin[ax], g)
				}
			}
			t.Logf("scenario=short-path(%s): result=pass", tc.name)
		})
	}
}

// TestWaypointOrderProgressMatchesMotion samples Queue().ActivePath every
// cycle while a path runs through a pause and a mid-run append, and asserts
// the reported ReachedPoints is always exactly the number of points already
// physically reached at that instant: it never leads the motion (no point
// reported before arrival), never lags, never goes backwards, and ends at
// TotalPoints. This rules out "reported point i done then driven back through
// it" - the same trace is checked for the visit order too.
func TestWaypointOrderProgressMatchesMotion(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	ctx := context.Background()
	initial := [][]float64{
		{0.60, 0.00, 0.00},
		{0.60, 0.45, 0.00},
		{0.10, 0.45, 0.30},
		{0.10, -0.10, 0.30},
	}
	id, _, err := s.PathJoints(ctx, "hmi", 1, arbiter.PriorityNormal, initial)
	if err != nil {
		t.Fatal(err)
	}
	extra := [][]float64{{-0.40, -0.10, 0.00}, {-0.40, 0.35, -0.20}}
	all := append(append([][]float64{}, initial...), extra...)
	appended := false
	pausedOnce := false
	arrivals := make([]int, len(all)) // first tick physically reached
	for i := range arrivals {
		arrivals[i] = -1
	}
	tick := 0
	complete := false
	for k := 0; k < 200000; k++ {
		s.Step(1)
		tick++
		axes := s.Axes()
		for wi, wp := range all {
			if wi >= 3 && !appended {
				break // point not admitted yet
			}
			if arrivals[wi] >= 0 {
				continue
			}
			ok := true
			for ax := range wp {
				if math.Abs(axes[ax].Position-wp[ax]) > testCfg().Tolerance.Position ||
					math.Abs(axes[ax].Velocity) > testCfg().Tolerance.Velocity {
					ok = false
				}
			}
			if ok {
				arrivals[wi] = tick
			}
		}
		st, err := s.Queue(ctx)
		if err != nil {
			t.Fatal(err)
		}
		// Prefix of points physically reached at this instant.
		physReached := 0
		for _, a := range arrivals {
			if a >= 0 {
				physReached++
			}
		}
		if st.ActivePath != nil {
			rp := st.ActivePath.ReachedPoints
			total := st.ActivePath.TotalPoints
			if total != len(all) && appended {
				t.Fatalf("TotalPoints=%d want %d after append", total, len(all))
			}
			if total > len(all) {
				t.Fatalf("TotalPoints=%d exceeds admitted %d", total, len(all))
			}
			if rp < 0 || rp > total {
				t.Fatalf("ReachedPoints=%d out of range total=%d", rp, total)
			}
			// Never report a point before it is physically reached.
			if rp > physReached {
				t.Fatalf("tick %d: ReachedPoints=%d but only %d point(s) physically arrived",
					tick, rp, physReached)
			}
			// Every point physically reached must already be counted by the
			// time the axes have left it; check the invariant against the
			// prefix using first-arrival ticks (a mid-leg cycle can briefly
			// sit inside a new point only once the prior leg completed, i.e.
			// reached was already incremented at the leg boundary).
			for wi := 0; wi < rp; wi++ {
				if arrivals[wi] < 0 {
					t.Fatalf("tick %d: ReachedPoints=%d counts point %d not yet reached physically",
						tick, rp, wi)
				}
			}
		}
		oc, _ := s.Outcome(id)
		// Interrupt once: pause on the second leg.
		if !pausedOnce && arrivals[0] >= 0 && arrivals[1] < 0 && tick > arrivals[0]+20 {
			if ok, _ := s.Pause(ctx); ok {
				s.Step(300)
				if err := s.Resume(ctx); err != nil {
					t.Fatal(err)
				}
				pausedOnce = true
			}
		}
		// Append once while point 1 leg is underway / queued batch after pause.
		if !appended && arrivals[0] >= 0 {
			if info, err := s.AppendJoints(ctx, "hmi", 2, id, extra); err == nil {
				appended = true
				t.Logf("scenario=progress: appended 2 points at tick %d, total=%d", tick, info.TotalPoints)
			}
		}
		if isTerminal(oc.State) {
			if oc.State != arbiter.StateCompleted {
				t.Fatalf("ended %s: %s", oc.State, oc.Reason)
			}
			complete = true
			break
		}
	}
	if !complete {
		t.Fatal("path never completed")
	}
	st, _ := s.Queue(ctx)
	if st.ActiveID != 0 {
		t.Fatalf("ActiveID=%d after completion", st.ActiveID)
	}
	for wi, a := range arrivals {
		if a < 0 {
			t.Fatalf("point %d never physically reached", wi)
		}
	}
	// Final accounting: terminal info must show every point reached.
	oc, _ := s.Outcome(id)
	if oc.State != arbiter.StateCompleted {
		t.Fatalf("final state %s", oc.State)
	}
	t.Logf("scenario=progress: result=pass arrivals=%v, report never led/lagged/reversed", arrivals)
}

// TestWaypointOrderConcurrentAppendAndReaders exercises concurrent appends
// and Queue/Axes readers against one driven path. The writer goroutine owns
// the path and appends while the main goroutine pauses, keeps appending
// blocked content serially and pumps cycles; outcomes must be deterministic
// and the visit sequence strictly ordered. Run with -race.
func TestWaypointOrderConcurrentAppendAndReaders(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	ctx := context.Background()
	initial := [][]float64{{0.5, 0.0, 0.0}, {0.5, 0.4, 0.0}, {0.0, 0.4, 0.0}}
	id, _, err := s.PathJoints(ctx, "hmi", 1, arbiter.PriorityNormal, initial)
	if err != nil {
		t.Fatal(err)
	}
	// The appended batch as REQUESTED; concurrent admission from distinct
	// entrances is scheduling-ordered, so the service (correctly) executes
	// points in their actual ADMISSION order. The test does not assume that
	// order: after every append is admitted it reads the admitted order and
	// verifies execution against it.
	extra := [][]float64{
		{0.0, 0.0, 0.3},
		{-0.4, 0.0, 0.3},
		{-0.4, 0.3, -0.2},
		{0.2, 0.3, -0.2},
	}
	extraByEntrance := make(map[int][]float64, len(extra)) // entrance seq -> point
	for i, wp := range extra {
		extraByEntrance[i+1] = wp
	}

	// Pause so appends race a stable, held cycle loop and readers.
	if ok, _ := s.Pause(ctx); !ok {
		t.Fatal("pause")
	}
	s.Step(300) // settle

	var appWG, readWG sync.WaitGroup
	appendErr := make(chan error, len(extra)+16)
	// Each append gets its own entrance/seq so all are admitted regardless of
	// scheduling; admission order among distinct entrances is the job-channel
	// FIFO order, which the serial replay below reproduces point by point.
	for seq, point := range extraByEntrance {
		appWG.Add(1)
		go func(seq int, point []float64) {
			defer appWG.Done()
			_, err := s.AppendJoints(ctx, fmt.Sprintf("ext%d", seq), 1, id, [][]float64{point})
			appendErr <- err
		}(seq, point)
	}
	stopReaders := make(chan struct{})
	for r := 0; r < 4; r++ {
		readWG.Add(1)
		go func() {
			defer readWG.Done()
			for {
				select {
				case <-stopReaders:
					return
				default:
					if st := s.Axes(); len(st) != 3 {
						appendErr <- fmt.Errorf("snapshot cardinality %d", len(st))
						return
					}
					if _, err := s.Queue(ctx); err != nil {
						appendErr <- err
						return
					}
					runtime.Gosched()
				}
			}
		}()
	}
	// Pump cycles while the goroutines append (hold keeps axes parked).
	for k := 0; k < 200; k++ {
		s.Step(1)
	}
	appWG.Wait() // appenders finish; readers keep running until stopReaders
	close(stopReaders)
	readWG.Wait()
	close(appendErr)
	for e := range appendErr {
		if e != nil {
			t.Fatal(e)
		}
	}
	// A Queue job (same FIFO channel, processed strictly after every append
	// job) establishes happens-before for all point writes. No cycle mutates
	// the points slice afterwards, so reading the admitted order here is
	// race-free; copy it before resuming.
	st, _ := s.Queue(ctx)
	if st.ActivePath == nil {
		t.Fatal("ActivePath missing while paused")
	}
	if st.ActivePath.TotalPoints != len(initial)+len(extra) {
		t.Fatalf("after concurrent appends TotalPoints=%d want %d",
			st.ActivePath.TotalPoints, len(initial)+len(extra))
	}
	admitted := admittedPoints(t, s, id)
	if len(admitted) != len(initial)+len(extra) {
		t.Fatalf("admitted %d points want %d", len(admitted), len(initial)+len(extra))
	}
	// Every requested appended point must be present exactly once.
	seen := map[int]bool{}
	for _, p := range admitted[len(initial):] {
		matched := -1
		for k, wp := range extra {
			if p[0] == wp[0] && p[1] == wp[1] && p[2] == wp[2] {
				matched = k
			}
		}
		if matched < 0 || seen[matched] {
			t.Fatalf("admitted tail %v missing/duplicated vs requested %v", p, extra)
		}
		seen[matched] = true
	}
	t.Logf("scenario=concurrent: %d appends + readers clean; admitted tail order=%v",
		len(extra), admitted[len(initial):])

	if err := s.Resume(ctx); err != nil {
		t.Fatal(err)
	}
	tr := tracePath(t, s, id, admitted, 200000, 300)
	requireVisitSeq(t, tr, admitted)
	// Determinism: replay the SAME admitted admission order serially and
	// compare the final position and arrival ticks.
	ref := newManual(t)
	defer ref.Close()
	rid, _, rerr := ref.PathJoints(ctx, "hmi", 1, arbiter.PriorityNormal, initial)
	if rerr != nil {
		t.Fatal(rerr)
	}
	if ok, _ := ref.Pause(ctx); !ok {
		t.Fatal("ref pause")
	}
	ref.Step(300 + 200)
	for i, wp := range admitted[len(initial):] {
		if _, err := ref.AppendJoints(ctx, "ref", uint64(i+1), rid, [][]float64{wp}); err != nil {
			t.Fatal(err)
		}
	}
	if err := ref.Resume(ctx); err != nil {
		t.Fatal(err)
	}
	refTrace := tracePath(t, ref, rid, admitted, 200000, 300)
	requireVisitSeq(t, refTrace, admitted)
	got, want := finalPositions(tr), finalPositions(refTrace)
	for ax := range got {
		if got[ax] != want[ax] {
			t.Fatalf("concurrent result axis %d=%.12f differs from serial replay %.12f", ax, got[ax], want[ax])
		}
	}
	if a1, a2 := firstArrivals(t, tr, admitted), firstArrivals(t, refTrace, admitted); len(a1) != len(a2) {
		t.Fatal("replay arrival count differs")
	} else {
		for i := range a1 {
			if a1[i] != a2[i] {
				t.Fatalf("replay arrival ticks differ at admitted point %d: %d vs %d", i, a1[i], a2[i])
			}
		}
		t.Logf("scenario=concurrent: result=pass final=%v, arrivals %v identical to serial replay", got, a1)
	}
}

// admittedPoints returns a deep copy of the path's currently admitted joint
// points as owned by the cycle loop. It is safe to call only when the caller
// has synchronized with the loop (a completed Queue/Append call) and no
// append is in flight; the points slice itself is never touched by cycles.
func admittedPoints(t *testing.T, s *Service, id uint64) [][]float64 {
	t.Helper()
	m, ok := s.moves[id]
	if !ok {
		t.Fatalf("move %d missing", id)
	}
	out := make([][]float64, len(m.points))
	for i, p := range m.points {
		out[i] = append([]float64(nil), p...)
	}
	return out
}
