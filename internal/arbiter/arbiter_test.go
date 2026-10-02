package arbiter

import (
	"fmt"
	"sync"
	"testing"

	"github.com/h3yow/robot-motion-controller-go/internal/mcerr"
)

func mkCmd(id uint64, entrance string, seq uint64, prio Priority, goal ...float64) *Command {
	return &Command{ID: id, Entrance: entrance, Seq: seq, Priority: prio, Goal: goal}
}

func TestFirstCommandRuns(t *testing.T) {
	a := New()
	res, err := a.Submit(mkCmd(1, "hmi", 1, PriorityNormal, 1))
	if err != nil {
		t.Fatal(err)
	}
	if res.State != StateRunning {
		t.Fatalf("state=%s want running", res.State)
	}
	if got := a.Active(); got == nil || got.ID != 1 {
		t.Fatalf("active=%v want cmd 1", got)
	}
}

func TestLowerPriorityQueues(t *testing.T) {
	a := New()
	a.Submit(mkCmd(1, "a", 1, PriorityNormal))
	res, err := a.Submit(mkCmd(2, "b", 1, PriorityLow))
	if err != nil {
		t.Fatal(err)
	}
	if res.State != StateQueued {
		t.Fatalf("low priority should queue, got %s", res.State)
	}
	if a.Active().ID != 1 {
		t.Fatal("active should remain cmd 1")
	}
	a.Finish(1, StateCompleted, "ok")
	nxt := a.PopQueued()
	if nxt == nil || nxt.ID != 2 {
		t.Fatalf("queued next = %v want cmd 2", nxt)
	}
}

func TestHigherPriorityPreempts(t *testing.T) {
	a := New()
	a.Submit(mkCmd(1, "a", 1, PriorityNormal))
	res, err := a.Submit(mkCmd(2, "b", 1, PriorityHigh))
	if err != nil {
		t.Fatal(err)
	}
	if res.State != StateRunning || res.PreemptedID != 1 {
		t.Fatalf("want running preempting 1, got state=%s preempted=%d", res.State, res.PreemptedID)
	}
	oc1, _ := a.Outcome(1)
	if oc1.State != StatePreempted {
		t.Fatalf("preempted cmd state=%s", oc1.State)
	}
	if a.Active().ID != 2 {
		t.Fatal("active should be cmd 2")
	}
	// Emergency preempts high.
	res3, err := a.Submit(mkCmd(3, "c", 1, PriorityEmergency))
	if err != nil {
		t.Fatal(err)
	}
	if res3.PreemptedID != 2 {
		t.Fatalf("emergency should preempt cmd 2, got %d", res3.PreemptedID)
	}
}

func TestEqualPriorityFIFO(t *testing.T) {
	a := New()
	a.Submit(mkCmd(1, "a", 1, PriorityNormal))
	a.Submit(mkCmd(2, "b", 1, PriorityNormal))
	a.Submit(mkCmd(3, "c", 1, PriorityNormal))
	ids := a.QueuedIDs()
	if len(ids) != 2 || ids[0] != 2 || ids[1] != 3 {
		t.Fatalf("FIFO queue = %v want [2 3]", ids)
	}
}

func TestPriorityWithinQueue(t *testing.T) {
	a := New()
	a.Submit(mkCmd(1, "a", 1, PriorityHigh))
	a.Submit(mkCmd(2, "b", 1, PriorityLow))
	a.Submit(mkCmd(3, "c", 1, PriorityNormal))
	nxt := a.PopQueued()
	if nxt == nil || nxt.ID != 3 {
		t.Fatalf("after finish, highest queued should be cmd 3, got %v", nxt)
	}
}

func TestStaleSequenceRejected(t *testing.T) {
	a := New()
	if _, err := a.Submit(mkCmd(1, "a", 5, PriorityNormal)); err != nil {
		t.Fatal(err)
	}
	if _, err := a.Submit(mkCmd(2, "a", 4, PriorityHigh)); !mcerr.HasCode(err, mcerr.CodeStaleSequence) {
		t.Fatalf("out-of-order seq should be stale, got %v", err)
	}
	if _, err := a.Submit(mkCmd(3, "a", 5, PriorityHigh)); !mcerr.HasCode(err, mcerr.CodeStaleSequence) {
		t.Fatalf("equal seq should be stale, got %v", err)
	}
	// Higher seq from same entrance is admitted.
	if _, err := a.Submit(mkCmd(4, "a", 6, PriorityHigh)); err != nil {
		t.Fatalf("next seq should be admitted: %v", err)
	}
	// Other entrances have independent sequences.
	if _, err := a.Submit(mkCmd(5, "b", 1, PriorityLow)); err != nil {
		t.Fatalf("other entrance seq independent: %v", err)
	}
}

func TestDuplicateIDIdempotent(t *testing.T) {
	a := New()
	a.Submit(mkCmd(7, "a", 1, PriorityNormal, 0.5))
	res, err := a.Submit(mkCmd(7, "a", 1, PriorityNormal, 0.99))
	if err != nil {
		t.Fatalf("identical retry must not error: %v", err)
	}
	if !res.Duplicate {
		t.Fatal("identical retry should be flagged duplicate")
	}
	// Same ID different content is rejected.
	if _, err := a.Submit(mkCmd(7, "a", 2, PriorityNormal)); !mcerr.HasCode(err, mcerr.CodeDuplicateCmd) {
		t.Fatalf("reused ID with new seq should be duplicate, got %v", err)
	}
	if _, err := a.Submit(mkCmd(7, "b", 1, PriorityNormal)); !mcerr.HasCode(err, mcerr.CodeDuplicateCmd) {
		t.Fatalf("reused ID with new entrance should be duplicate, got %v", err)
	}
}

func TestCancelSemantics(t *testing.T) {
	a := New()
	a.Submit(mkCmd(1, "a", 1, PriorityNormal))
	a.Submit(mkCmd(2, "b", 1, PriorityLow))

	if _, err := a.Cancel(2); err != nil {
		t.Fatal(err)
	}
	oc2, _ := a.Outcome(2)
	if oc2.State != StateCanceled {
		t.Fatalf("queued cancel should finish immediately, got %s", oc2.State)
	}
	if _, err := a.Cancel(99); !mcerr.HasCode(err, mcerr.CodeCommandUnknown) {
		t.Fatalf("unknown cancel = %v", err)
	}
	if _, err := a.Cancel(1); err != nil {
		t.Fatal(err)
	}
	oc1, _ := a.Outcome(1)
	if oc1.State != StateStopping {
		t.Fatalf("active cancel moves to stopping, got %s", oc1.State)
	}
	// Executor finishes after safe stop.
	a.Finish(1, StateCanceled, "done")
	a.Finish(1, StateCompleted, "rollback attempt") // must be ignored
	oc1b, _ := a.Outcome(1)
	if oc1b.State != StateCanceled {
		t.Fatalf("late Finish must not roll back terminal state, got %s", oc1b.State)
	}
}

func TestRecoverableFaultRoundTrip(t *testing.T) {
	a := New()
	a.Submit(mkCmd(1, "a", 1, PriorityNormal))
	a.Finish(1, StateFaultedRecov, "feedback dropout")
	if isTerminalLocal(a, 1) {
		t.Fatal("recoverable fault must not be terminal")
	}
	if !a.MarkRecovered(1) {
		t.Fatal("MarkRecovered should succeed")
	}
	oc, _ := a.Outcome(1)
	if oc.State != StateRunning {
		t.Fatalf("recovered state=%s", oc.State)
	}
	// Escalation path.
	a.Submit(mkCmd(2, "b", 1, PriorityHigh))
	oc1, _ := a.Outcome(1)
	if oc1.State != StatePreempted {
		t.Fatalf("cmd1 should be preempted, got %s", oc1.State)
	}
}

func isTerminalLocal(a *Arbiter, id uint64) bool {
	oc, _ := a.Outcome(id)
	switch oc.State {
	case StateCompleted, StateCanceled, StatePreempted, StateFaulted, StateRejected:
		return true
	}
	return false
}

func TestConcurrentSubmitConsistency(t *testing.T) {
	a := New()
	const goroutines = 8
	const per = 50
	var wg sync.WaitGroup
	errs := make(chan error, goroutines*per)
	for g := 0; g < goroutines; g++ {
		wg.Add(1)
		go func(g int) {
			defer wg.Done()
			entrance := string(rune('A' + g))
			for k := 1; k <= per; k++ {
				id := uint64(g*per + k)
				if _, err := a.Submit(mkCmd(id, entrance, uint64(k), PriorityNormal)); err != nil {
					errs <- err
				}
			}
		}(g)
	}
	wg.Wait()
	close(errs)
	for err := range errs {
		t.Fatalf("concurrent submit error: %v", err)
	}
	snap := a.Snapshot()
	if len(snap) != goroutines*per {
		t.Fatalf("snapshot has %d entries want %d", len(snap), goroutines*per)
	}
}

func TestCapacityBoundAndPreemption(t *testing.T) {
	a := NewWithCapacity(2)
	cmd := func(id uint64, ent string, p Priority) *Command {
		return &Command{ID: id, Entrance: ent, Seq: id, Priority: p, Goal: []float64{1}}
	}
	if _, err := a.Submit(cmd(1, "a", PriorityNormal)); err != nil {
		t.Fatal(err)
	}
	if _, err := a.Submit(cmd(2, "b", PriorityNormal)); err != nil {
		t.Fatal(err) // queued: 1 active + 1 queued == capacity
	}
	if _, err := a.Submit(cmd(3, "c", PriorityLow)); !mcerr.HasCode(err, mcerr.CodeQueueFull) {
		t.Fatalf("third admission: want queue_full, got %v", err)
	}
	// Higher priority preempts: pending count unchanged, admitted.
	res, err := a.Submit(cmd(4, "e", PriorityHigh))
	if err != nil || res.State != StateRunning || res.PreemptedID != 1 {
		t.Fatalf("preempt while full: res=%+v err=%v", res, err)
	}
	if active, queued := a.PendingCount(); active != 1 || queued != 1 {
		t.Fatalf("pending after preempt = %d/%d", active, queued)
	}
}

func TestCapacityUnboundedZero(t *testing.T) {
	a := NewWithCapacity(0)
	for i := uint64(1); i <= 100; i++ {
		ent := fmt.Sprintf("e%d", i)
		if _, err := a.Submit(&Command{ID: i, Entrance: ent, Seq: 1, Priority: PriorityLow, Goal: []float64{1}}); err != nil {
			t.Fatalf("unbounded admission %d: %v", i, err)
		}
	}
	if _, queued := a.PendingCount(); queued != 99 {
		t.Fatalf("queued=%d want 99", queued)
	}
}
