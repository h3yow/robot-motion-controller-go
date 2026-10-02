// Command demo runs a fully local, offline end-to-end scenario with the
// fixed (manual) simulation clock. It exercises:
//
//  1. a nominal synchronized multi-axis move;
//  2. rejection of an unreachable Cartesian target (distinct reason);
//  3. a recoverable feedback-loss fault and explicit recovery;
//  4. higher-priority preemption and a queued cancel;
//  5. deterministic replay of the same inputs.
//
// No network, hardware or credentials are used.
package main

import (
	"context"
	"fmt"
	"log"

	"github.com/h3yow/robot-motion-controller-go/internal/arbiter"
	"github.com/h3yow/robot-motion-controller-go/internal/config"
	"github.com/h3yow/robot-motion-controller-go/internal/control"
	"github.com/h3yow/robot-motion-controller-go/internal/kinematics"
	"github.com/h3yow/robot-motion-controller-go/internal/mcerr"
	"github.com/h3yow/robot-motion-controller-go/internal/motion"
)

func main() {
	cfg := demoConfig()

	fmt.Println("== 1. nominal multi-axis move ==")
	final1 := run(cfg, nil)
	fmt.Printf("   final joint positions: %+.4f\n", final1)

	fmt.Println("== 2. unreachable Cartesian target is rejected ==")
	s := mustService(cfg, nil)
	_, err := s.MovePose(context.Background(), "hmi", 1, arbiter.PriorityNormal,
		kinematics.Pose{X: 50, Y: 50, Phi: 0})
	fmt.Printf("   rejected: code=%s\n", code(err))
	s.Close()

	fmt.Println("== 3. recoverable feedback loss, then recovery ==")
	drop := func(tick int64, axis int) bool { return axis == 0 && tick >= 311 && tick < 312 }
	s = mustService(cfg, drop)
	id, _ := s.MoveJoints(context.Background(), "hmi", 1, arbiter.PriorityNormal,
		[]float64{0.8, 0.2, -0.3})
	driveToFault(s, id)
	// Premature recovery is refused.
	if err := s.Recover(context.Background()); err != nil {
		fmt.Printf("   premature recover refused: code=%s\n", code(err))
	}
	// Step on healthy feedback until the recovery hold window is satisfied.
	for k := 0; k < 500; k++ {
		s.Step(1)
		if s.Recover(context.Background()) == nil {
			fmt.Println("   fault recovered; move resumed")
			break
		}
	}
	oc := wait(s, id)
	fmt.Printf("   outcome: %s (%s)\n", oc.State, oc.Reason)
	s.Close()

	fmt.Println("== 4. high-priority preemption and queued cancel ==")
	s = mustService(cfg, nil)
	id1, _ := s.MoveJoints(context.Background(), "hmi", 1, arbiter.PriorityNormal, []float64{1.2, 0, 0})
	s.Step(40)
	id2, _ := s.MoveJoints(context.Background(), "estop", 1, arbiter.PriorityEmergency, []float64{-0.4, 0, 0})
	oc1 := wait(s, id1)
	fmt.Printf("   preempted cmd: %s (%s)\n", oc1.State, oc1.Reason)
	oc2 := wait(s, id2)
	fmt.Printf("   emergency cmd: %s\n", oc2.State)
	// A late cancel of the already-terminal id1 is idempotent, not an error.
	_ = s.Cancel(context.Background(), id1)
	s.Close()

	fmt.Println("== 5. deterministic replay ==")
	f2 := run(cfg, nil)
	if fmt.Sprintf("%v", final1) != fmt.Sprintf("%v", f2) {
		log.Fatalf("replay differs: %v vs %v", final1, f2)
	}
	fmt.Printf("   replay matches: %+.4f\n", f2)

	fmt.Println("== 6. waypoint path, append, pause/resume, queue status ==")
	demoPathQueue(cfg)
}

// demoPathQueue exercises the new ordered-waypoint path, mid-run append,
// pause/resume, the pending-command ceiling and the consistent queue query.
func demoPathQueue(cfg *config.Config) {
	ctx := context.Background()
	s := mustService(cfg, nil)
	defer s.Close()

	id, err := s.MovePath(ctx, "hmi", 1, arbiter.PriorityNormal, []motion.Waypoint{
		{Joints: []float64{0.5, 0, 0}},
		{Pose: &kinematics.Pose{X: 0.55, Y: 0.2, Phi: 0.3}},
	})
	if err != nil {
		log.Fatalf("MovePath: %v", err)
	}
	// Append a joint waypoint while the first segment is in flight.
	s.Step(40)
	if err := s.AppendWaypoints(ctx, id, []motion.Waypoint{
		{Joints: []float64{-0.2, 0.2, -0.2}},
	}); err != nil {
		log.Fatalf("append: %v", err)
	}

	// Pause: axes brake to rest then hold in place with the loop active.
	if err := s.Pause(ctx, id); err != nil {
		log.Fatalf("pause: %v", err)
	}
	stepUntilSettled(s, control.PhasePaused, 30, 20000)
	st, _ := s.Queue(ctx)
	fmt.Printf("   paused: phase=%s active=#%d waypoint=%d/%d pending=%d/%d\n",
		st.Phase, st.ActiveID, st.Active.WaypointIndex, st.Active.WaypointTotal,
		st.Pending, st.MaxPending)
	held := s.Axes()[0].Position
	for k := 0; k < 200; k++ {
		s.Step(1)
	}
	if d := s.Axes()[0].Position - held; d > 0.005 || d < -0.005 {
		log.Fatalf("axis drifted %.5f rad while paused", d)
	}
	if err := s.Resume(ctx, id); err != nil {
		log.Fatalf("resume: %v", err)
	}
	oc := wait(s, id)
	fmt.Printf("   path outcome: %s, final joints %+.4f\n", oc.State, roundAxes(s.Axes()))

	// Pending ceiling: submit a burst of multi-point paths and show the
	// distinct queue_full rejection once the budget is exceeded.
	full := false
	for k := 0; k < 1000; k++ {
		_, e := s.MovePath(ctx, fmt.Sprintf("bulk%d", k), 1, arbiter.PriorityLow,
			[]motion.Waypoint{{Joints: []float64{0.1, 0, 0}}})
		if e != nil {
			if code(e) != mcerr.CodeQueueFull {
				log.Fatalf("unexpected submit error: %v", e)
			}
			full = true
			break
		}
	}
	if !full {
		log.Fatal("expected queue_full under a bounded backlog")
	}
	st, _ = s.Queue(ctx)
	fmt.Printf("   backlog capped: pending=%d/%d, over-capacity submit -> code=%s\n",
		st.Pending, st.MaxPending, mcerr.CodeQueueFull)
}

// stepUntilSettled steps until the controller reports want and every axis
// has stayed within the convergence velocity band for run consecutive
// cycles (covering the small post-brake rebound into the held position).
func stepUntilSettled(s *motion.Service, want control.Phase, run, max int) {
	parked := false
	rest := 0
	for k := 0; k < max; k++ {
		s.Step(1)
		st, err := s.Queue(context.Background())
		if err != nil {
			log.Fatal(err)
		}
		if st.Phase == want {
			parked = true
		}
		atRest := true
		for _, a := range s.Axes() {
			if a.Velocity > 0.02 || a.Velocity < -0.02 {
				atRest = false
			}
		}
		if parked && atRest {
			rest++
		} else {
			rest = 0
		}
		if rest >= run {
			return
		}
	}
	log.Fatalf("never settled in phase %s", want)
}

func roundAxes(axes []control.AxisStatus) []float64 {
	out := make([]float64, len(axes))
	for i, a := range axes {
		out[i] = a.Position
	}
	return out
}

func run(cfg *config.Config, drop motion.DropFn) []float64 {
	s := mustService(cfg, drop)
	defer s.Close()
	id, err := s.MoveJoints(context.Background(), "hmi", 1, arbiter.PriorityNormal,
		[]float64{0.7, -0.5, 0.3})
	if err != nil {
		log.Fatalf("submit: %v", err)
	}
	oc := wait(s, id)
	if oc.State != arbiter.StateCompleted {
		log.Fatalf("expected completed, got %s: %s", oc.State, oc.Reason)
	}
	out := make([]float64, len(s.Axes()))
	for i, a := range s.Axes() {
		out[i] = a.Position
	}
	return out
}

func mustService(cfg *config.Config, drop motion.DropFn) *motion.Service {
	opts := []motion.Option{motion.WithManualClock()}
	if drop != nil {
		opts = append(opts, motion.WithFeedbackDrop(drop))
	}
	s, err := motion.New(cfg, opts...)
	if err != nil {
		log.Fatalf("motion.New: %v", err)
	}
	return s
}

func wait(s *motion.Service, id uint64) *arbiter.Outcome {
	for k := 0; k < 1_000_000; k++ {
		s.Step(1)
		oc, err := s.Outcome(id)
		if err != nil {
			log.Fatal(err)
		}
		switch oc.State {
		case arbiter.StateCompleted, arbiter.StateCanceled, arbiter.StatePreempted,
			arbiter.StateFaulted, arbiter.StateRejected:
			return oc
		}
	}
	log.Fatal("timeout waiting for command")
	return nil
}

func driveToFault(s *motion.Service, id uint64) {
	for k := 0; k < 5000; k++ {
		s.Step(1)
		oc, _ := s.Outcome(id)
		if oc.State == arbiter.StateFaultedRecov || oc.State == arbiter.StateFaulted {
			return
		}
	}
	log.Fatal("fault never occurred")
}

func code(err error) mcerr.Code { return mcerr.CodeOf(err) }

func demoConfig() *config.Config {
	maxPending := 64
	return &config.Config{
		Version: "1.1.0", Period: 2_000_000, MaxPendingCommands: &maxPending,
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
			DropoutMaxTicks: 2, SaturationMaxTicks: 15, EndGraceTicks: 200,
			RecoveryHoldTicks: 2, SafeDecel: 12},
	}
}
