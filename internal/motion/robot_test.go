package motion_test

import (
	"math"
	"testing"

	"github.com/h3yow/robot-motion-controller-go/internal/motion"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/classify"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/config"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/engine"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/kinematics"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/mctest"
)

func cfg() *config.Config {
	return &config.Config{
		SchemaVersion: 1, ControlPeriod: 0.002,
		Axes: []config.AxisConfig{
			{Name: "j1", MinPosition: -math.Pi, MaxPosition: math.Pi, MaxVelocity: 3, MaxAccel: 10, MaxTorque: 60, Inertia: 1},
			{Name: "j2", MinPosition: -math.Pi, MaxPosition: math.Pi, MaxVelocity: 3, MaxAccel: 10, MaxTorque: 60, Inertia: 1},
		},
		Controller: config.ControllerConfig{Kp: 80, Ki: 5, Kd: 16, IntegralWindup: 10, TrackingTolerance: 0.05, SaturationCycles: 20},
	}
}

func TestPoseRejections(t *testing.T) {
	mt := mctest.Wrap(t)
	r, err := motion.NewRobot(cfg(), 1.0, 0.6)
	mt.CheckNoErr("new robot", "l1=1,l2=0.6", err)

	cases := []struct {
		name   string
		pose   kinematics.Vec
		reason classify.Reason
	}{
		{"too far", kinematics.Vec{X: 3, Y: 0}, classify.ReasonTargetUnreachable},
		{"inside inner radius", kinematics.Vec{X: 0.2, Y: 0}, classify.ReasonTargetUnreachable},
		{"nonfinite x", kinematics.Vec{X: math.NaN(), Y: 0}, classify.ReasonTargetNonFinite},
		{"nonfinite y", kinematics.Vec{X: 0, Y: math.Inf(1)}, classify.ReasonTargetNonFinite},
	}
	for _, c := range cases {
		_, err := r.MoveToPose("ui", 1, 0, c.pose, false)
		mt.CheckReason(c.name, c.pose, err, c.reason)
	}
	mt.CheckEq("idle after rejects", "bad poses", r.Engine().State(), engine.StateIdle)
}

func TestReachablePoseExecutes(t *testing.T) {
	mt := mctest.Wrap(t)
	c := cfg()
	r, err := motion.NewRobot(c, 1.0, 1.0)
	mt.CheckNoErr("new", "l1=l2=1", err)
	pose := kinematics.Vec{X: 1.2, Y: 0.3}
	_, err = r.MoveToPose("ui", 1, 0, pose, true)
	mt.CheckNoErr("reachable", pose, err)
	res := r.Engine().Run()
	mt.CheckTrue("done", "pose (1.2,0.3)", res.Done, "expected convergence")
	// End-effector must actually reach the requested Cartesian pose.
	snap := r.Engine().Snapshot()
	got := r.Arm().Forward(kinematics.Joints{Q1: snap.Axes[0].PositionActual, Q2: snap.Axes[1].PositionActual})
	mt.CheckApprox("fk x", pose, got.X, pose.X, 0.01)
	mt.CheckApprox("fk y", pose, got.Y, pose.Y, 0.01)
}

func TestNewRobotValidation(t *testing.T) {
	mt := mctest.Wrap(t)
	bad := cfg()
	bad.SchemaVersion = 99
	_, err := motion.NewRobot(bad, 1, 1)
	mt.CheckReason("bad version", "v99", err, classify.ReasonConfigVersion)
	one := cfg()
	one.Axes = one.Axes[:1]
	_, err = motion.NewRobot(one, 1, 1)
	mt.CheckReason("axis count", "1 axis", err, classify.ReasonConfigField)
	_, err = motion.NewRobot(cfg(), 0, 1)
	mt.CheckReason("link length", "l1=0", err, classify.ReasonConfigField)
}
