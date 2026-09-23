// Command demo runs an entirely offline, deterministic scenario suite:
// nominal multi-axis motion, distinguishable constraint/target rejections,
// feedback-loss fault with guarded recovery, command preemption/cancel and
// concurrent command arbitration. It uses the fixed simulation clock only
// (engine.Run / Step), so output is reproducible bit-for-bit.
package main

import (
	"fmt"
	"math"
	"sync"

	"github.com/h3yow/robot-motion-controller-go/internal/motion"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/arbiter"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/classify"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/config"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/engine"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/kinematics"
)

func main() {
	cfg := demoConfig()
	r, err := motion.NewRobot(cfg, 1.0, 0.8)
	must(err)
	e := r.Engine()

	section("1. Nominal multi-axis move to a reachable Cartesian pose")
	pose := kinematics.Vec{X: 1.3, Y: 0.4}
	_, err = r.MoveToPose("ui", 1, 0, pose, true)
	must(err)
	res := e.Run()
	snap := e.Snapshot()
	q := kinematics.Joints{Q1: snap.Axes[0].PositionActual, Q2: snap.Axes[1].PositionActual}
	ee := r.Arm().Forward(q)
	fmt.Printf("   cycles=%d done=%v  end-effector=(%.4f,%.4f) target=(%.4f,%.4f)\n",
		snap.Cycle, res.Done, ee.X, ee.Y, pose.X, pose.Y)

	section("2. Rejected targets with distinguishable reasons")
	rejectPose(r, 2, kinematics.Vec{X: 5, Y: 0})   // outside reach
	rejectPose(r, 3, kinematics.Vec{X: 0.1, Y: 0}) // inner radius (rmin=0.2)
	rejectPose(r, 4, kinematics.Vec{X: math.NaN(), Y: 0})
	_, err = e.Submit(move("ui", 5, 0, 9, 0)) // out of joint travel
	printReason(err)

	section("3. Feedback loss -> FAULT -> guarded recovery -> fresh command")
	_, err = r.MoveToPose("ui", 6, 0, kinematics.Vec{X: 0.6, Y: 0.9}, true)
	must(err)
	var faultCycles int64
	startCycle := e.Snapshot().Cycle
	for {
		if e.Snapshot().Cycle-startCycle == 8 {
			e.SetFeedbackValid(false)
		}
		e.Step()
		s := e.Snapshot()
		if s.Fault != nil {
			faultCycles = s.Cycle
			fmt.Printf("   fault=%s at cycle=%d state=%s\n", s.Fault.Kind, faultCycles, s.State)
			break
		}
		if e.State() == engine.StateIdle {
			panic("expected a fault but motion completed")
		}
	}
	fmt.Print("   recover while feedback lost -> ")
	printReason(e.Recover())
	e.SetFeedbackValid(true)
	for k := 0; k < 1000 && e.State() == engine.StateFault; k++ {
		e.Step()
		if stopped(e.Snapshot()) {
			break
		}
	}
	must(e.Recover())
	fmt.Printf("   recovered, state=%s; issuing fresh move\n", e.State())
	_, err = e.Submit(move("ui", 7, 0, 0, 0))
	must(err)
	e.Run()
	fmt.Printf("   back at joint position (%.4f,%.4f)\n",
		e.Snapshot().Axes[0].PositionActual, e.Snapshot().Axes[1].PositionActual)

	section("4. Priority preemption and cancel")
	_, err = e.Submit(move("ui", 8, 1, -1.2, 0.5)) // long move, prio 1
	must(err)
	for k := 0; k < 40; k++ {
		e.Step()
	}
	_, err = e.Submit(move("api", 1, 0, 0.2, 0)) // lower priority denied
	printReason(err)
	_, err = e.Submit(move("api", 2, 9, 0.2, 0)) // higher priority preempts
	must(err)
	fmt.Printf("   state after preempt=%s; finishing stop ramp then pending move\n", e.State())
	e.Run()
	fmt.Printf("   preempting goal reached at (%.4f,%.4f)\n",
		e.Snapshot().Axes[0].PositionActual, e.Snapshot().Axes[1].PositionActual)
	_, _ = e.Submit(move("ui", 9, 1, -1, -1))
	for k := 0; k < 20; k++ {
		e.Step()
	}
	_, err = e.Submit(arbiter.Command{Source: "ui", Sequence: 10, Op: arbiter.OpCancel, CancelSeq: 9})
	must(err)
	cr := e.Run()
	fmt.Printf("   cancel result: state=%s done=%v (a cancel never reports Done)\n", cr.State, cr.Done)

	section("5. Concurrent entry points: duplicate/late commands arbitrated")
	_, _ = e.Submit(move("seed", 1, 1, 0.5, 0.5))
	var wg sync.WaitGroup
	results := make(chan classify.Reason, 16)
	for g := 0; g < 4; g++ {
		wg.Add(1)
		go func(g int) {
			defer wg.Done()
			for k := 0; k < 4; k++ { // intentionally identical seq across goroutines
				_, err := e.Submit(move("race", int64(k), 5, 0, 0))
				if rj, ok := classify.AsReject(err); ok {
					results <- rj.Reason
				} else {
					results <- classify.ReasonOK
				}
			}
		}(g)
	}
	done := make(chan struct{})
	var swg sync.WaitGroup
	swg.Add(1)
	go func() {
		defer swg.Done()
		for {
			select {
			case <-done:
				return
			default:
				e.Step()
			}
		}
	}()
	wg.Wait()
	close(done)
	swg.Wait()
	close(results)
	tally := map[classify.Reason]int{}
	for rsn := range results {
		tally[rsn]++
	}
	fmt.Printf("   identical concurrent submissions classified: accepted=%d duplicate=%d priority=%d stale=%d\n",
		tally[classify.ReasonOK], tally[classify.ReasonDuplicateCommand],
		tally[classify.ReasonPriorityDenied], tally[classify.ReasonStaleCommand])

	section("done")
}

func section(t string) { fmt.Printf("\n=== %s ===\n", t) }

func must(err error) {
	if err != nil {
		panic(err)
	}
}

func move(src string, seq int64, prio int, goal ...float64) arbiter.Command {
	return arbiter.Command{Source: src, Sequence: seq, Priority: prio, Op: arbiter.OpMoveTo, Goal: goal}
}

func rejectPose(r *motion.Robot, seq int64, p kinematics.Vec) {
	_, err := r.MoveToPose("ui", seq, 0, p, false)
	printReason(err)
}

func printReason(err error) {
	if rj, ok := classify.AsReject(err); ok {
		fmt.Printf("   rejected reason=%s | %s\n", rj.Reason, rj.Msg)
	} else if err != nil {
		fmt.Printf("   error=%v\n", err)
	}
}

func stopped(s engine.Snapshot) bool {
	for _, a := range s.Axes {
		if math.Abs(a.VelocityActual) > 1e-6 {
			return false
		}
	}
	return true
}

func demoConfig() *config.Config {
	return &config.Config{
		SchemaVersion: 1, ControlPeriod: 0.002,
		Axes: []config.AxisConfig{
			{Name: "j1", MinPosition: -math.Pi, MaxPosition: math.Pi, MaxVelocity: 3, MaxAccel: 12, MaxTorque: 60, Inertia: 1},
			{Name: "j2", MinPosition: -math.Pi, MaxPosition: math.Pi, MaxVelocity: 3, MaxAccel: 12, MaxTorque: 60, Inertia: 1},
		},
		Controller: config.ControllerConfig{Kp: 80, Ki: 5, Kd: 16, IntegralWindup: 10, TrackingTolerance: 0.05, SaturationCycles: 20},
	}
}
