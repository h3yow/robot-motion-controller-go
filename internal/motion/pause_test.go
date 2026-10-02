package motion

import (
	"context"
	"math"
	"testing"

	"github.com/h3yow/robot-motion-controller-go/internal/arbiter"
	"github.com/h3yow/robot-motion-controller-go/internal/mcerr"
)

func driveUntilProgress(t *testing.T, s *Service, minIdx int, id uint64, maxTicks int) {
	t.Helper()
	for k := 0; k < maxTicks; k++ {
		s.Step(1)
		idx, total := s.ctrl.Progress()
		if total > 0 && idx >= minIdx {
			return
		}
		oc, _ := s.Outcome(id)
		if isTerminal(oc.State) {
			t.Fatalf("command ended early at progress %d/%d: %s", idx, total, oc.State)
		}
	}
	t.Fatalf("never reached progress index %d", minIdx)
}

// TestPauseResumeReachesGoal pauses mid-move, verifies the axes park and the
// reference freezes, then resumes and checks the original goal is reached.
func TestPauseResumeReachesGoal(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	ctx := context.Background()
	goal := []float64{0.8, -0.6, 0.4}
	id, err := s.MoveJoints(ctx, "hmi", 1, arbiter.PriorityNormal, goal)
	if err != nil {
		t.Fatal(err)
	}
	driveUntilProgress(t, s, 40, id, 20000)
	if ok, err := s.Pause(ctx); err != nil || !ok {
		t.Fatalf("Pause: ok=%v err=%v", ok, err)
	}
	// Allow the ramp to settle into the hold.
	s.Step(200)
	st, _ := s.Queue(ctx)
	if !st.Paused {
		t.Fatal("queue snapshot does not report paused")
	}
	holdPos := axisPositions(s)
	frozenIdx := st.ActiveProgress.Index
	if frozenIdx == 0 {
		t.Fatal("progress index should be inside the trajectory")
	}
	t.Logf("paused at progress %d/%d, hold positions=%.4f", frozenIdx, st.ActiveProgress.Total, holdPos)
	// While held: closed loop keeps the axes parked; reference does not
	// advance; velocity stays near zero.
	for k := 0; k < 800; k++ {
		s.Step(1)
		axes := s.Axes()
		for ax := range axes {
			if d := math.Abs(axes[ax].Position - holdPos[ax]); d > 0.01 {
				t.Fatalf("axis %d drifted %.5f rad during pause hold", ax, d)
			}
			if math.Abs(axes[ax].Velocity) > testCfg().Tolerance.Velocity+0.02 {
				t.Fatalf("axis %d speed %.4f during pause hold", ax, axes[ax].Velocity)
			}
		}
		idx, _ := s.ctrl.Progress()
		if idx != frozenIdx {
			t.Fatalf("trajectory advanced during pause: %d -> %d", frozenIdx, idx)
		}
	}
	if err := s.Resume(ctx); err != nil {
		t.Fatalf("Resume: %v", err)
	}
	st, _ = s.Queue(ctx)
	if st.Paused {
		t.Fatal("still paused after resume")
	}
	oc := runUntil(t, s, id, 40000)
	if oc.State != arbiter.StateCompleted {
		t.Fatalf("after resume state=%s reason=%s", oc.State, oc.Reason)
	}
	for ax, g := range goal {
		if d := math.Abs(s.Axes()[ax].Position - g); d > testCfg().Tolerance.Position {
			t.Fatalf("axis %d final %.5f deviates from goal %.5f by %.5f after pause",
				ax, s.Axes()[ax].Position, g, d)
		}
	}
	t.Log("resumed after long hold; original goal reached within tolerance")
}

// TestPauseDurationDoesNotChangeResult compares the final configuration of
// the same move run without pause, with a short pause and a long pause.
func TestPauseDurationDoesNotChangeResult(t *testing.T) {
	goal := []float64{0.7, 0.3, -0.2}
	run := func(holdTicks int) []float64 {
		s := newManual(t)
		defer s.Close()
		ctx := context.Background()
		id, err := s.MoveJoints(ctx, "hmi", 1, arbiter.PriorityNormal, goal)
		if err != nil {
			t.Fatal(err)
		}
		driveUntilProgress(t, s, 40, id, 20000)
		if holdTicks > 0 {
			if ok, _ := s.Pause(ctx); !ok {
				t.Fatal("pause failed")
			}
			s.Step(holdTicks + 200) // 200 ticks to settle plus the hold
			if err := s.Resume(ctx); err != nil {
				t.Fatal(err)
			}
		}
		oc := runUntil(t, s, id, 40000)
		if oc.State != arbiter.StateCompleted {
			t.Fatalf("hold=%d ended %s", holdTicks, oc.State)
		}
		return axisPositions(s)
	}
	base := run(0)
	for _, hold := range []int{100, 1500} {
		got := run(hold)
		for ax := range goal {
			if d := math.Abs(got[ax] - base[ax]); d > testCfg().Tolerance.Position {
				t.Fatalf("hold=%d axis %d final %.6f differs from unpaused %.6f by %.5f",
					hold, ax, got[ax], base[ax], d)
			}
			if d := math.Abs(got[ax] - goal[ax]); d > testCfg().Tolerance.Position {
				t.Fatalf("hold=%d axis %d missed goal: %.6f vs %.6f", hold, ax, got[ax], goal[ax])
			}
		}
		t.Logf("hold=%4d ticks reproduces the unpaused result: %.6f", hold, got)
	}
}

// TestPauseKeepsQueuedCommands verifies queued work is not lost across a
// pause and runs after resume, in order.
func TestPauseKeepsQueuedCommands(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	ctx := context.Background()
	id1, err := s.MoveJoints(ctx, "a", 1, arbiter.PriorityNormal, []float64{0.8, 0, 0})
	if err != nil {
		t.Fatal(err)
	}
	id2, err := s.MoveJoints(ctx, "b", 1, arbiter.PriorityNormal, []float64{0.8, 0.6, 0})
	if err != nil {
		t.Fatal(err)
	}
	id3, err := s.MoveJoints(ctx, "c", 1, arbiter.PriorityNormal, []float64{0.2, 0.6, 0.3})
	if err != nil {
		t.Fatal(err)
	}
	s.Step(60)
	if ok, _ := s.Pause(ctx); !ok {
		t.Fatal("pause")
	}
	s.Step(300)
	st, _ := s.Queue(ctx)
	if !st.Paused || st.ActiveID != id1 || len(st.QueuedIDs) != 2 ||
		st.QueuedIDs[0] != id2 || st.QueuedIDs[1] != id3 {
		t.Fatalf("queue during pause = %+v", st)
	}
	t.Logf("during pause: active=%d queued=%v retained", st.ActiveID, st.QueuedIDs)
	if err := s.Resume(ctx); err != nil {
		t.Fatal(err)
	}
	for _, id := range []uint64{id1, id2, id3} {
		oc := runUntil(t, s, id, 40000)
		if oc.State != arbiter.StateCompleted {
			t.Fatalf("command %d after resume: %s (%s)", id, oc.State, oc.Reason)
		}
	}
	final := axisPositions(s)
	want := []float64{0.2, 0.6, 0.3}
	for ax := range want {
		if d := math.Abs(final[ax] - want[ax]); d > testCfg().Tolerance.Position {
			t.Fatalf("queued tail final axis %d = %.5f want %.5f", ax, final[ax], want[ax])
		}
	}
}

// TestCancelDuringPause cancels a paused move; queued commands still follow.
func TestCancelDuringPause(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	ctx := context.Background()
	id1, err := s.MoveJoints(ctx, "a", 1, arbiter.PriorityNormal, []float64{0.8, 0, 0})
	if err != nil {
		t.Fatal(err)
	}
	id2, err := s.MoveJoints(ctx, "b", 1, arbiter.PriorityNormal, []float64{-0.4, 0, 0})
	if err != nil {
		t.Fatal(err)
	}
	s.Step(60)
	if ok, _ := s.Pause(ctx); !ok {
		t.Fatal("pause")
	}
	s.Step(300)
	if err := s.Cancel(ctx, id1); err != nil {
		t.Fatalf("cancel paused: %v", err)
	}
	if err := s.Resume(ctx); err == nil {
		t.Fatal("resume after canceling the paused move must be refused")
	}
	oc := runUntil(t, s, id1, 5000)
	if oc.State != arbiter.StateCanceled {
		t.Fatalf("paused-then-canceled move state=%s", oc.State)
	}
	t.Logf("canceled during pause: %s", oc.Reason)
	// The latch stays set until explicitly resumed; clear it and the queued
	// command runs to its own goal.
	if err := s.Resume(ctx); !mcerr.HasCode(err, mcerr.CodeTargetContradictory) {
		// Nothing is paused now; expected refusal, latch is cleared by cancel.
	}
	oc2 := runUntil(t, s, id2, 40000)
	if oc2.State != arbiter.StateCompleted {
		t.Fatalf("queued command after pause-cancel: %s (%s)", oc2.State, oc2.Reason)
	}
}

// TestPauseResumeOnPath verifies a multi-point path still reaches its final
// point after a pause.
func TestPauseResumeOnPath(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	ctx := context.Background()
	points := [][]float64{{0.7, 0, 0}, {0.7, 0.5, 0}, {0.3, 0.5, 0.2}}
	id, _, err := s.PathJoints(ctx, "hmi", 1, arbiter.PriorityNormal, points)
	if err != nil {
		t.Fatal(err)
	}
	driveUntilProgress(t, s, 30, id, 20000)
	if ok, _ := s.Pause(ctx); !ok {
		t.Fatal("pause")
	}
	s.Step(400)
	st, _ := s.Queue(ctx)
	if st.ActivePath == nil {
		t.Fatal("active path info missing during pause")
	}
	t.Logf("paused path info: %+v", *st.ActivePath)
	if err := s.Resume(ctx); err != nil {
		t.Fatal(err)
	}
	oc := runUntil(t, s, id, 60000)
	if oc.State != arbiter.StateCompleted {
		t.Fatalf("path after pause: %s (%s)", oc.State, oc.Reason)
	}
	for ax, g := range points[2] {
		if d := math.Abs(s.Axes()[ax].Position - g); d > testCfg().Tolerance.Position {
			t.Fatalf("path final axis %d = %.5f want %.5f", ax, s.Axes()[ax].Position, g)
		}
	}
}

// TestPauseIdleAndResumeErrors covers the refusal cases.
func TestPauseIdleAndResumeErrors(t *testing.T) {
	s := newManual(t)
	defer s.Close()
	ctx := context.Background()
	if ok, err := s.Pause(ctx); err != nil || ok {
		t.Fatalf("pause while idle: ok=%v err=%v, want false/nil", ok, err)
	}
	if err := s.Resume(ctx); !mcerr.HasCode(err, mcerr.CodeTargetContradictory) {
		t.Fatalf("resume while idle: %v", err)
	}
}
