package arbiter_test

import (
	"testing"

	"github.com/h3yow/robot-motion-controller-go/internal/motion/arbiter"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/classify"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/mctest"
)

func mv(src string, seq, prio int64) arbiter.Command {
	return arbiter.Command{Source: src, Sequence: seq, Priority: int(prio), Op: arbiter.OpMoveTo, Goal: []float64{1}}
}

func TestAcceptDedupStale(t *testing.T) {
	mt := mctest.Wrap(t)
	a := arbiter.New()
	id := arbiter.ID{Source: "ui", Sequence: 1}
	d1 := a.Submit(mv("ui", 1, 0), false, id, 0, false)
	mt.CheckTrue("accept first", "ui#1", d1.Accepted, d1.Message)
	// Exact replay after completion -> duplicate, must not re-run.
	a.Complete(id)
	d2 := a.Submit(mv("ui", 1, 0), false, id, 0, false)
	mt.CheckEq("duplicate replay", "ui#1 again", d2.Reason, classify.ReasonDuplicateCommand)
	// Out-of-order older seq -> stale.
	d3 := a.Submit(mv("ui", 0, 0), false, id, 0, false)
	mt.CheckEq("stale", "ui#0 after ui#1", d3.Reason, classify.ReasonStaleCommand)
	// Newer seq accepted.
	d4 := a.Submit(mv("ui", 2, 0), false, id, 0, false)
	mt.CheckTrue("accept newer", "ui#2", d4.Accepted, d4.Message)
	// Each source has its own watermark.
	d5 := a.Submit(mv("api", 1, 0), false, id, 0, false)
	mt.CheckTrue("independent source", "api#1", d5.Accepted, d5.Message)
}

func TestPriorityPreemption(t *testing.T) {
	mt := mctest.Wrap(t)
	a := arbiter.New()
	active := arbiter.ID{Source: "low", Sequence: 5}
	// Same priority -> no preempt.
	d := a.Submit(mv("other", 1, 5), true, active, 5, false)
	mt.CheckEq("tie loses", "prio 5 vs 5", d.Reason, classify.ReasonPriorityDenied)
	// Lower priority -> no preempt.
	d = a.Submit(mv("other", 1, 4), true, active, 5, false)
	mt.CheckEq("lower loses", "prio 4 vs 5", d.Reason, classify.ReasonPriorityDenied)
	// Higher -> preempt exactly once; replay of the loser cannot sneak in.
	d = a.Submit(mv("other", 2, 9), true, active, 5, false)
	mt.CheckTrue("preempt", "prio 9 vs 5", d.Accepted && d.Preempted && d.PreemptedID == active, d.Message)
	d = a.Submit(mv("other", 1, 4), true, active, 5, false)
	mt.CheckEq("old loser stays stale", "other#1 replay", d.Reason, classify.ReasonStaleCommand)
}

func TestCancel(t *testing.T) {
	mt := mctest.Wrap(t)
	a := arbiter.New()
	active := arbiter.ID{Source: "ui", Sequence: 7}
	cancel := arbiter.Command{Source: "ui", Sequence: 8, Op: arbiter.OpCancel, CancelSeq: 7}
	// Wrong target seq -> rejected, no state change.
	bad := cancel
	bad.CancelSeq = 6
	dbad := a.Submit(bad, true, active, 5, false)
	mt.CheckEq("wrong cancel target", "cancel #6 active #7", dbad.Reason, classify.ReasonInvalidCommand)
	d := a.Submit(cancel, true, active, 5, false)
	mt.CheckTrue("cancel accepted", "cancel #8", d.Accepted && d.Preempted && d.Reason == classify.ReasonCancelled, d.Message)
	// Replayed cancel is duplicate -> cannot cancel anything twice.
	d2 := a.Submit(cancel, false, arbiter.ID{}, 0, false)
	mt.CheckEq("cancel replay", "same cancel twice", d2.Reason, classify.ReasonDuplicateCommand)
	// Cancel from a different source cannot touch the active command.
	other := arbiter.Command{Source: "api", Sequence: 1, Op: arbiter.OpCancel, CancelSeq: 7}
	d3 := a.Submit(other, true, active, 5, false)
	mt.CheckEq("foreign cancel", "api cancels ui cmd", d3.Reason, classify.ReasonInvalidCommand)
}

func TestFaultActiveBarsMotion(t *testing.T) {
	mt := mctest.Wrap(t)
	a := arbiter.New()
	d := a.Submit(mv("ui", 1, 0), false, arbiter.ID{}, 0, true)
	mt.CheckEq("fault bars motion", "faultActive=true", d.Reason, classify.ReasonFaultActive)
	rst := arbiter.Command{Source: "ui", Sequence: 2, Op: arbiter.OpReset}
	dr := a.Submit(rst, false, arbiter.ID{}, 0, true)
	mt.CheckTrue("reset accepted", "reset with fault", dr.Accepted, dr.Message)
	// Reset without fault is rejected.
	dr2 := a.Submit(arbiter.Command{Source: "ui", Sequence: 3, Op: arbiter.OpReset}, false, arbiter.ID{}, 0, false)
	mt.CheckEq("reset w/o fault", "faultActive=false", dr2.Reason, classify.ReasonInvalidCommand)
}
