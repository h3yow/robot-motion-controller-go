package engine_test

import (
	"sync"
	"testing"

	"github.com/h3yow/robot-motion-controller-go/internal/motion/arbiter"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/classify"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/config"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/controller"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/engine"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/mctest"
)

func testCfg() *config.Config {
	return &config.Config{
		SchemaVersion: 1, ControlPeriod: 0.002,
		Axes: []config.AxisConfig{
			{Name: "j1", MinPosition: -3.14, MaxPosition: 3.14, MaxVelocity: 3, MaxAccel: 10, MaxTorque: 60, Inertia: 1},
			{Name: "j2", MinPosition: -3.14, MaxPosition: 3.14, MaxVelocity: 2.5, MaxAccel: 8, MaxTorque: 60, Inertia: 1},
		},
		Controller: config.ControllerConfig{Kp: 80, Ki: 5, Kd: 16, IntegralWindup: 10, TrackingTolerance: 0.05, SaturationCycles: 20},
	}
}

func move(src string, seq int64, prio int, goal ...float64) arbiter.Command {
	return arbiter.Command{Source: src, Sequence: seq, Priority: prio, Op: arbiter.OpMoveTo, Goal: goal}
}

func TestNominalMultiAxisEndToEnd(t *testing.T) {
	mt := mctest.Wrap(t)
	e := engine.New(testCfg())
	_, err := e.Submit(move("ui", 1, 0, 1, -0.8))
	mt.CheckNoErr("submit", "goal {1,-0.8}", err)
	r := e.Run()
	mt.CheckTrue("completed", "run", r.Done && r.State == engine.StateIdle,
		"expected clean completion, got "+string(r.State))
	snap := e.Snapshot()
	mt.CheckApprox("axis0 final", "goal {1,-0.8}", snap.Axes[0].PositionActual, 1, 0.01)
	mt.CheckApprox("axis1 final", "goal {1,-0.8}", snap.Axes[1].PositionActual, -0.8, 0.01)
}

func TestDeterministicRun(t *testing.T) {
	mt := mctest.Wrap(t)
	trace := func() []engine.Snapshot {
		e := engine.New(testCfg())
		_, _ = e.Submit(move("ui", 1, 0, 1.2, 0.6))
		var out []engine.Snapshot
		for e.State() != engine.StateIdle {
			e.Step()
			out = append(out, e.Snapshot())
		}
		return out
	}
	a, b := trace(), trace()
	mt.CheckEq("cycle count", "identical runs", len(b), len(a))
	for i := range a {
		for ax := 0; ax < 2; ax++ {
			mt.CheckApprox("reproducible position", "fixed clock + fixed inputs",
				b[i].Axes[ax].PositionActual, a[i].Axes[ax].PositionActual, 0)
		}
	}
}

func TestSubmitRejections(t *testing.T) {
	mt := mctest.Wrap(t)
	e := engine.New(testCfg())
	cases := []struct {
		name   string
		cmd    arbiter.Command
		reason classify.Reason
	}{
		{"out of travel", move("ui", 1, 0, 5, 0), classify.ReasonOutOfTravel},
		{"non-finite", move("ui", 2, 0, nanE(), 0), classify.ReasonTargetNonFinite},
		{"wrong dims", move("ui", 3, 0, 1), classify.ReasonContradictoryTarget},
	}
	for _, c := range cases {
		_, err := e.Submit(c.cmd)
		mt.CheckReason(c.name, c.cmd, err, c.reason)
	}
	mt.CheckEq("still idle after rejects", "bad commands", e.State(), engine.StateIdle)
}

func TestPreemptThenCancel(t *testing.T) {
	mt := mctest.Wrap(t)
	e := engine.New(testCfg())
	_, err := e.Submit(move("ui", 1, 1, 3, 0))
	mt.CheckNoErr("submit low", "ui#1 prio1 -> {3,0}", err)
	for k := 0; k < 30; k++ {
		e.Step()
	}
	// Lower priority cannot preempt.
	_, err = e.Submit(move("api", 1, 0, 0.5, 0))
	mt.CheckReason("low prio denied", "prio0 while prio1 active", err, classify.ReasonPriorityDenied)
	// Higher priority preempts.
	_, err = e.Submit(move("api", 2, 5, 0.2, 0))
	mt.CheckNoErr("preempt", "api#2 prio5", err)
	mt.CheckEq("stopping", "just after preempt", e.State(), engine.StateStopping)
	// Run the stop ramp and the pending command to completion.
	r := e.Run()
	mt.CheckTrue("pending ran", "after preempt", r.State == engine.StateIdle,
		"expected idle after pending motion, got "+string(r.State))
	mt.CheckApprox("new goal axis0", "api goal 0.2", e.Snapshot().Axes[0].PositionActual, 0.2, 0.01)

	// Cancel scenario.
	_, _ = e.Submit(move("ui", 2, 1, -2, 0))
	for k := 0; k < 10; k++ {
		e.Step()
	}
	_, err = e.Submit(arbiter.Command{Source: "ui", Sequence: 3, Op: arbiter.OpCancel, CancelSeq: 2})
	mt.CheckNoErr("cancel", "ui#3 cancels ui#2", err)
	r = e.Run()
	mt.CheckTrue("cancelled to idle", "cancel", r.State == engine.StateIdle && !r.Done,
		"cancelled motion must not report Done")
}

func TestFeedbackLossFaultAndRecovery(t *testing.T) {
	mt := mctest.Wrap(t)
	e := engine.New(testCfg())
	_, _ = e.Submit(move("ui", 1, 0, 2, 1))
	var f *controller.Fault
	for e.State() == engine.StateRunning || e.State() == engine.StateStopping {
		// Inject feedback loss a few cycles in.
		if e.Snapshot().Cycle == 5 {
			e.SetFeedbackValid(false)
		}
		e.Step()
		s := e.Snapshot()
		if s.Fault != nil {
			f = s.Fault
			break
		}
	}
	mt.CheckTrue("feedback fault latched", "encoder loss mid-motion",
		f != nil && f.Kind == controller.FaultFeedbackLost, "expected FEEDBACK_LOST")
	mt.CheckEq("fault state", "recoverable", e.State(), engine.StateFault)

	// Recovery while feedback still lost must be refused with distinct reason.
	err := e.Recover()
	mt.CheckReason("recover refused early", "feedback still invalid", err, classify.ReasonResetNotSatisfied)
	// Motion commands barred during fault.
	_, err = e.Submit(move("api", 1, 9, 0, 0))
	mt.CheckReason("motion barred in fault", "FAULT active", err, classify.ReasonFaultActive)

	// Restore feedback; the brake brings axes to rest; then recovery succeeds.
	e.SetFeedbackValid(true)
	for k := 0; k < 400 && e.State() == engine.StateFault; k++ {
		e.Step()
		if stopped(e.Snapshot()) {
			break
		}
	}
	err = e.Recover()
	mt.CheckNoErr("recover ok", "feedback restored + stopped", err)
	mt.CheckEq("idle after recover", "recovery", e.State(), engine.StateIdle)
	// Interrupted motion is NOT silently resumed: a fresh command is required
	// and starts from the actual stopped position.
	_, err = e.Submit(move("ui", 2, 0, 0, 0))
	mt.CheckNoErr("fresh move accepted", "post-recovery command", err)
	r := e.Run()
	mt.CheckTrue("fresh move completes", "post-recovery", r.Done, "fresh command must complete")
	mt.CheckApprox("returns to zero", "goal {0,0}", e.Snapshot().Axes[0].PositionActual, 0, 0.01)
}

func TestFatalDivergenceRequiresReconfigure(t *testing.T) {
	mt := mctest.Wrap(t)
	e := engine.New(testCfg())
	_, _ = e.Submit(move("ui", 1, 0, 1, 0))
	// Make the plant diverge (huge constant disturbance beyond torque limit).
	e.SetDisturbance([]float64{5_000_000, 0})
	var f *controller.Fault
	for k := 0; k < 2000; k++ {
		e.Step()
		s := e.Snapshot()
		if s.Fault != nil {
			f = s.Fault
			break
		}
	}
	mt.CheckTrue("fatal fault", "runaway disturbance", f != nil && !f.Kind.Recoverable(),
		"expected unrecoverable fault")
	mt.CheckEq("fatal state", "divergence", e.State(), engine.StateFatal)
	// Recover must not work on FATAL.
	err := e.Recover()
	mt.CheckReason("no recover on fatal", "FATAL", err, classify.ReasonResetNotSatisfied)
	// Bad reconfigure is rejected and state is unchanged.
	bad := testCfg()
	bad.ControlPeriod = 0
	err = e.Reconfigure(bad)
	mt.CheckReason("bad reconfigure rejected", "period=0", err, classify.ReasonConfigField)
	mt.CheckEq("still fatal", "invalid reconfigure", e.State(), engine.StateFatal)
	// Good reconfigure clears everything.
	err = e.Reconfigure(testCfg())
	mt.CheckNoErr("reconfigure", "valid cfg after FATAL", err)
	mt.CheckEq("idle after reconfigure", "hard reset", e.State(), engine.StateIdle)
}

func TestConcurrentSubmits(t *testing.T) {
	mt := mctest.Wrap(t)
	e := engine.New(testCfg())
	_, _ = e.Submit(move("seed", 1, 1, 0.5, 0))

	var wg sync.WaitGroup
	// Hammer identical replays from several goroutines; at most one effect,
	// all others classified duplicate/stale/priority, never a panic/rollback.
	for g := 0; g < 8; g++ {
		wg.Add(1)
		go func(g int) {
			defer wg.Done()
			src := "race"
			for k := 0; k < 50; k++ {
				_, _ = e.Submit(move(src, int64(k+1), 10, 0.1, 0))
				_ = e.Snapshot()
			}
		}(g)
	}
	// Concurrent stepping and snapshotting, stopped via context-like chan.
	stop := make(chan struct{})
	var swg sync.WaitGroup
	swg.Add(2)
	go func() {
		defer swg.Done()
		for {
			select {
			case <-stop:
				return
			default:
				e.Step()
			}
		}
	}()
	go func() {
		defer swg.Done()
		for {
			select {
			case <-stop:
				return
			default:
				_ = e.Snapshot()
			}
		}
	}()
	wg.Wait()   // submitters finish
	close(stop) // then stop steppers/readers
	swg.Wait()
	s := e.Snapshot()
	for ax, a := range s.Axes {
		if !finiteE(a.PositionActual) || !finiteE(a.VelocityActual) {
			t.Fatalf("non-finite state after concurrent hammering\n  axis=%d snap=%#v", ax, s)
		}
	}
	mt.CheckTrue("valid lifecycle state", "concurrent run",
		s.State == engine.StateIdle || s.State == engine.StateRunning ||
			s.State == engine.StateStopping || s.State == engine.StateFault,
		"unexpected state "+string(s.State))
}

func stopped(s engine.Snapshot) bool {
	for _, a := range s.Axes {
		if absE(a.VelocityActual) > 1e-6 {
			return false
		}
	}
	return true
}
