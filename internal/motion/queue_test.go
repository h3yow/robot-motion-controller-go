package motion

import (
	"context"
	"fmt"
	"math"
	"runtime"
	"sync"
	"testing"

	"github.com/h3yow/robot-motion-controller-go/internal/arbiter"
	"github.com/h3yow/robot-motion-controller-go/internal/config"
	"github.com/h3yow/robot-motion-controller-go/internal/mcerr"
)

func cfgWithQueue(capacity int) *config.Config {
	cfg := testCfg()
	cfg.Queue = &config.QueuePolicy{Capacity: capacity}
	return cfg
}

// TestQueueCapacityRejectsOverflow fills the configured capacity and checks
// the NEXT admission is refused with queue_full, distinct from every other
// rejection; completing one command frees a slot again.
func TestQueueCapacityRejectsOverflow(t *testing.T) {
	const cap = 3
	cfg := cfgWithQueue(cap)
	s, err := New(cfg, WithManualClock())
	if err != nil {
		t.Fatal(err)
	}
	defer s.Close()
	ctx := context.Background()
	ids := make([]uint64, 0, cap)
	// First command activates immediately; the following two fill the queue.
	for i := 0; i < cap; i++ {
		g := 0.1 + 0.1*float64(i)
		id, err := s.MoveJoints(ctx, fmt.Sprintf("ui%d", i), 1, arbiter.PriorityNormal,
			[]float64{g, 0, 0})
		if err != nil {
			t.Fatalf("admission %d: %v", i, err)
		}
		ids = append(ids, id)
	}
	st, _ := s.Queue(ctx)
	if st.Capacity != cap || st.Pending != cap || len(st.QueuedIDs) != cap-1 {
		t.Fatalf("full queue snapshot: %+v", st)
	}
	t.Logf("capacity reached: %d/%d pending, queued=%v", st.Pending, st.Capacity, st.QueuedIDs)
	_, err = s.MoveJoints(ctx, "extra", 1, arbiter.PriorityNormal, []float64{0.5, 0, 0})
	if !mcerr.HasCode(err, mcerr.CodeQueueFull) {
		t.Fatalf("overflow rejection: want queue_full, got %v", err)
	}
	t.Logf("overflow rejected distinctly: %v", err)
	// A stale-sequence rejection must still report its own code (not masked as
	// full) even while the queue is full.
	_, err = s.MoveJoints(ctx, "ui0", 1, arbiter.PriorityNormal, []float64{0.5, 0, 0})
	if !mcerr.HasCode(err, mcerr.CodeStaleSequence) {
		t.Fatalf("stale seq while full: want stale_sequence, got %v", err)
	}
	// A preempting (higher priority) command replaces the active one and does
	// not grow the pending set, so it is admitted despite the full queue.
	preemptID, err := s.MoveJoints(ctx, "estop", 1, arbiter.PriorityEmergency, []float64{0, 0, 0})
	if err != nil {
		t.Fatalf("preempt while full must be admitted: %v", err)
	}
	st, _ = s.Queue(ctx)
	if st.ActiveID != preemptID {
		t.Fatalf("preempting command not active: %+v", st)
	}
	t.Logf("preempt admitted while full; active=%d queued=%v", st.ActiveID, st.QueuedIDs)
}

// TestQueueCapacityOneOnlyActive with capacity 1 nothing can queue; once the
// active command completes the slot is reusable and the same entrance may
// retry a previously-refused sequence number.
func TestQueueCapacityOneOnlyActive(t *testing.T) {
	cfg := cfgWithQueue(1)
	s, err := New(cfg, WithManualClock())
	if err != nil {
		t.Fatal(err)
	}
	defer s.Close()
	ctx := context.Background()
	id, err := s.MoveJoints(ctx, "a", 1, arbiter.PriorityNormal, []float64{0.4, 0, 0})
	if err != nil {
		t.Fatal(err)
	}
	// Equal/lower priority cannot be admitted: no queue slot.
	if _, err := s.MoveJoints(ctx, "b", 1, arbiter.PriorityNormal, []float64{0.2, 0, 0}); !mcerr.HasCode(err, mcerr.CodeQueueFull) {
		t.Fatalf("capacity-1 queue: %v", err)
	}
	runUntil(t, s, id, 40000)
	// Retry with the SAME seq from b: the rejection committed no sequence.
	id2, err := s.MoveJoints(ctx, "b", 1, arbiter.PriorityNormal, []float64{0.2, 0, 0})
	if err != nil {
		t.Fatalf("retry after slot freed: %v", err)
	}
	t.Logf("retry after completion admitted as %d", id2)
	runUntil(t, s, id2, 40000)
}

// TestQueueStatusConsistency walks a path through the queue and checks the
// snapshot always agrees with command outcomes and controller progress.
func TestQueueStatusConsistency(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	ctx := context.Background()
	id1, err := s.MoveJoints(ctx, "a", 1, arbiter.PriorityNormal, []float64{0.6, 0, 0})
	if err != nil {
		t.Fatal(err)
	}
	id2, _, err := s.PathJoints(ctx, "b", 1, arbiter.PriorityNormal,
		[][]float64{{0.6, 0.4, 0}, {0.1, 0.4, 0.2}})
	if err != nil {
		t.Fatal(err)
	}
	st, _ := s.Queue(ctx)
	if st.ActiveID != id1 || st.QueuedIDs[0] != id2 || st.Pending != 2 ||
		st.ActiveProgress.Total == 0 || st.ActivePath != nil {
		t.Fatalf("initial status: %+v", st)
	}
	t.Logf("status while running+queued: active=%d queued=%v progress=%d/%d",
		st.ActiveID, st.QueuedIDs, st.ActiveProgress.Index, st.ActiveProgress.Total)
	// While the path is queued its info is not "active"; once it activates the
	// path info appears and progresses with reached point counts.
	var sawReachedOne bool
	for k := 0; k < 80000; k++ {
		s.Step(1)
		st, _ := s.Queue(ctx)
		// Invariant checks on every cycle.
		if st.Pending != len(st.QueuedIDs)+boolToInt(st.ActiveID != 0) {
			t.Fatalf("inconsistent pending count: %+v", st)
		}
		if st.Pending > st.Capacity {
			t.Fatalf("pending %d exceeds capacity %d", st.Pending, st.Capacity)
		}
		if st.ActiveID == id2 {
			if st.ActivePath == nil {
				t.Fatal("active path info missing")
			}
			if st.ActivePath.ReachedPoints > st.ActivePath.TotalPoints {
				t.Fatalf("path progress impossible: %+v", st.ActivePath)
			}
			if st.ActivePath.ReachedPoints >= 1 {
				sawReachedOne = true
			}
			idx, total := s.ctrl.Progress()
			if st.ActiveProgress.Index != idx || st.ActiveProgress.Total != total {
				t.Fatalf("progress mismatch snapshot=%+v ctrl=%d/%d", st.ActiveProgress, idx, total)
			}
		}
		oc1, _ := s.Outcome(id1)
		if isTerminal(oc1.State) && st.ActiveID == id1 {
			t.Fatalf("terminal command still reported active: %+v", st)
		}
		oc2, _ := s.Outcome(id2)
		if isTerminal(oc2.State) {
			break
		}
	}
	if !sawReachedOne {
		t.Fatal("never observed the active path reach its first point")
	}
	st, _ = s.Queue(ctx)
	if st.Pending != 0 || st.ActiveID != 0 || len(st.QueuedIDs) != 0 {
		t.Fatalf("status after drain: %+v", st)
	}
	t.Logf("drained status: %+v (observed reached-point transition)", st)
}

func boolToInt(b bool) int {
	if b {
		return 1
	}
	return 0
}

// TestConcurrentSubmitsAndQueueReads runs many entrances submitting at once
// while readers poll the queue snapshot and axis statuses under the race
// detector. Concurrency only changes ADMISSION interleaving, never targets,
// so two replays must reach the same final configuration within tolerance
// (convergence-level equality; bit-identical equality cannot hold when the
// activation order legitimately differs between runs).
func TestConcurrentSubmitsAndQueueReads(t *testing.T) {
	run := func() []float64 {
		cfg := testCfg()
		cfg.Queue = &config.QueuePolicy{Capacity: 512}
		s, err := New(cfg, WithManualClock())
		if err != nil {
			t.Fatal(err)
		}
		defer s.Close()
		const n = 24
		var subWg, readWg sync.WaitGroup
		errs := make(chan error, 256)
		ids := make(chan uint64, n)
		goal := []float64{0.3, 0.2, -0.1}
		for g := 0; g < n; g++ {
			subWg.Add(1)
			go func(g int) {
				defer subWg.Done()
				entrance := fmt.Sprintf("ui%02d", g)
				// Mix the two entrances: even goroutines use joints, odd ones
				// resolve the identical target through a pose.
				var id uint64
				var err error
				if g%2 == 0 {
					id, err = s.MoveJoints(context.Background(), entrance, 1, arbiter.PriorityNormal, goal)
				} else {
					pose := fkOf(t, cfg, goal)
					id, err = s.MovePose(context.Background(), entrance, 1, arbiter.PriorityNormal, pose)
				}
				if err != nil {
					errs <- fmt.Errorf("submit %s: %w", entrance, err)
					return
				}
				ids <- id
			}(g)
		}
		stopReaders := make(chan struct{})
		for g := 0; g < 4; g++ {
			readWg.Add(1)
			go func(g int) {
				defer readWg.Done()
				for {
					select {
					case <-stopReaders:
						return
					default:
						st, qerr := s.Queue(context.Background())
						if qerr != nil {
							errs <- qerr
							return
						}
						if st.Pending > st.Capacity || st.Pending != len(st.QueuedIDs)+boolToInt(st.ActiveID != 0) {
							errs <- fmt.Errorf("inconsistent snapshot: %+v", st)
							return
						}
						if len(s.Axes()) != len(cfg.Axes) {
							errs <- fmt.Errorf("half-updated axis snapshot")
							return
						}
						runtime.Gosched()
					}
				}
			}(g)
		}
		go func() { subWg.Wait(); close(ids) }()
		seen := map[uint64]bool{}
		draining := true
		for draining {
			select {
			case id, ok := <-ids:
				if !ok {
					draining = false
					continue
				}
				seen[id] = true
			default:
				s.Step(1)
			}
		}
		if len(seen) != n {
			t.Fatalf("admitted %d, want %d", len(seen), n)
		}
		pending := true
		for k := 0; k < 300000 && pending; k++ {
			s.Step(1)
			pending = false
			for id := range seen {
				oc, qerr := s.Outcome(id)
				if qerr != nil {
					t.Fatal(qerr)
				}
				if !isTerminal(oc.State) {
					pending = true
				}
			}
		}
		for id := range seen {
			oc, _ := s.Outcome(id)
			if oc.State != arbiter.StateCompleted {
				t.Errorf("id %d ended %s: %s", id, oc.State, oc.Reason)
			}
		}
		close(stopReaders)
		readWg.Wait()
		close(errs)
		for e := range errs {
			t.Fatal(e)
		}
		return axisPositions(s)
	}
	first := run()
	second := run()
	tol := testCfg().Tolerance.Position
	for ax := range first {
		if d := math.Abs(first[ax] - second[ax]); d > tol {
			t.Fatalf("concurrent runs disagree beyond tolerance on axis %d: %.12f vs %.12f (d=%.6f)",
				ax, first[ax], second[ax], d)
		}
		if d := math.Abs(first[ax] - []float64{0.3, 0.2, -0.1}[ax]); d > tol {
			t.Fatalf("axis %d final %.6f off the common goal", ax, first[ax])
		}
	}
	t.Logf("two concurrent runs reproduced the same goal within %.1e: %.6f", tol, first)
}

// TestConfigQueueValidationAndDefault covers the config compatibility
// surface: omission keeps the documented default, explicit zero/negative is
// rejected, explicit positive is honored.
func TestConfigQueueValidationAndDefault(t *testing.T) {
	if config.DefaultQueueCapacity <= 0 {
		t.Fatal("default queue capacity must be positive")
	}
	cfg := testCfg()
	if got := cfg.QueueCapacity(); got != config.DefaultQueueCapacity {
		t.Fatalf("missing queue section: got capacity %d, want documented default %d", got, config.DefaultQueueCapacity)
	}
	t.Logf("legacy config without queue section -> default capacity %d (no silent downgrade)", config.DefaultQueueCapacity)
	for _, bad := range []int{0, -5} {
		c := testCfg()
		c.Queue = &config.QueuePolicy{Capacity: bad}
		if err := c.Validate(); !mcerr.HasCode(err, mcerr.CodeConfigInvalid) {
			t.Fatalf("capacity %d validation: %v", bad, err)
		}
	}
	c := testCfg()
	c.Queue = &config.QueuePolicy{Capacity: 7}
	if err := c.Validate(); err != nil {
		t.Fatal(err)
	}
	if c.QueueCapacity() != 7 {
		t.Fatalf("explicit capacity not honored: %d", c.QueueCapacity())
	}
}
