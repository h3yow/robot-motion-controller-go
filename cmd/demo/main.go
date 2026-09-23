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
	return &config.Config{
		Version: "1.0.0", Period: 2_000_000,
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
