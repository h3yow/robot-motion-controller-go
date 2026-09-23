package trajectory_test

import (
	"testing"

	"github.com/h3yow/robot-motion-controller-go/internal/motion/classify"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/config"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/mctest"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/trajectory"
)

func testCfg() *config.Config {
	return &config.Config{
		SchemaVersion: 1,
		ControlPeriod: 0.01,
		Axes: []config.AxisConfig{
			{Name: "j1", MinPosition: -3.14, MaxPosition: 3.14, MaxVelocity: 2, MaxAccel: 4, MaxTorque: 10, Inertia: 1},
			{Name: "j2", MinPosition: -3.14, MaxPosition: 3.14, MaxVelocity: 1.5, MaxAccel: 3, MaxTorque: 10, Inertia: 1, GravityTorque: 1},
		},
		Controller: config.ControllerConfig{Kp: 40, Kd: 10, IntegralWindup: 5, TrackingTolerance: 0.2, SaturationCycles: 5},
	}
}

func TestPointToPointNominal(t *testing.T) {
	mt := mctest.Wrap(t)
	cfg := testCfg()
	mt.CheckNoErr("cfg", cfg, cfg.Validate())
	pl := trajectory.NewPlanner(cfg)
	in := map[string]any{"start": []float64{0, 0}, "goal": []float64{1, -0.5}, "vel": []float64{2, 1.5}, "accel": []float64{4, 3}}
	tr, err := pl.PointToPoint([]float64{0, 0}, []float64{1, -0.5}, []float64{2, 1.5}, []float64{4, 3})
	mt.CheckNoErr("plan", in, err)

	last := tr.Points[len(tr.Points)-1]
	mt.CheckApprox("final q0", in, last.Position[0], 1, 1e-12)
	mt.CheckApprox("final q1", in, last.Position[1], -0.5, 1e-12)
	mt.CheckApprox("final v", in, last.Velocity[0]+last.Velocity[1], 0, 1e-12)
	for _, pt := range tr.Points {
		if abs(pt.Velocity[0]) > 2+1e-9 || abs(pt.Velocity[1]) > 1.5+1e-9 {
			t.Fatalf("velocity limit violated\n  input: %#v\n  point: %#v", in, pt)
		}
		if abs(pt.Accel[0]) > 4+1e-9 || abs(pt.Accel[1]) > 3+1e-9 {
			t.Fatalf("accel limit violated\n  input: %#v\n  point: %#v", in, pt)
		}
	}
}

func TestDeterminismAndContinuity(t *testing.T) {
	mt := mctest.Wrap(t)
	cfg := testCfg()
	pl := trajectory.NewPlanner(cfg)
	tr1, _ := pl.PointToPoint([]float64{0, 0}, []float64{1, -0.5}, []float64{2, 1.5}, []float64{4, 3})
	tr2, _ := pl.PointToPoint([]float64{0, 0}, []float64{1, -0.5}, []float64{2, 1.5}, []float64{4, 3})
	mt.CheckEq("same cycle count", "fixed inputs", len(tr2.Points), len(tr1.Points))
	for k := range tr1.Points {
		for a := 0; a < 2; a++ {
			mt.CheckApprox("reproducible position", "fixed inputs", tr2.Points[k].Position[a], tr1.Points[k].Position[a], 0)
		}
	}
	tr3, _ := pl.PointToPoint([]float64{0, 0}, []float64{1.001, -0.5}, []float64{2, 1.5}, []float64{4, 3})
	var maxDiff float64
	for k := range tr1.Points {
		if k < len(tr3.Points) {
			maxDiff = maxf(maxDiff, abs(tr3.Points[k].Position[0]-tr1.Points[k].Position[0]))
		}
	}
	mt.CheckTrue("continuous in goal", "goal perturbation 1e-3", maxDiff < 0.01, "max sample diff "+ftoa(maxDiff))
}

func TestPlanningRejections(t *testing.T) {
	mt := mctest.Wrap(t)
	cfg := testCfg()
	cases := []struct {
		name   string
		start  []float64
		goal   []float64
		vel    []float64
		accel  []float64
		reason classify.Reason
	}{
		{"nonfinite goal", []float64{0, 0}, []float64{1, nan()}, []float64{2, 1.5}, []float64{4, 3}, classify.ReasonTargetNonFinite},
		{"out of travel", []float64{0, 0}, []float64{4, 0}, []float64{2, 1.5}, []float64{4, 3}, classify.ReasonOutOfTravel},
		{"velocity envelope", []float64{0, 0}, []float64{1, 0}, []float64{3, 1.5}, []float64{4, 3}, classify.ReasonVelocityLimit},
		{"accel envelope", []float64{0, 0}, []float64{1, 0}, []float64{2, 1.5}, []float64{5, 3}, classify.ReasonAccelerationLimit},
		{"torque infeasible", []float64{0, 0}, []float64{1, 0}, []float64{2, 1.5}, []float64{4, 3}, classify.ReasonTorqueLimit},
		{"length mismatch", []float64{0}, []float64{1, 0}, []float64{2, 1.5}, []float64{4, 3}, classify.ReasonContradictoryTarget},
		{"zero velocity", []float64{0, 0}, []float64{1, 0}, []float64{0, 1.5}, []float64{4, 3}, classify.ReasonContradictoryTarget},
	}
	for _, c := range cases {
		// The torque case needs a stricter torque cap on axis 0.
		c2 := *cfg
		if c.name == "torque infeasible" {
			c2.Axes[0].MaxTorque = 2 // inertia 1 * accel 4 + 0 = 4 > 2
		}
		pl2 := trajectory.NewPlanner(&c2)
		in := map[string]any{"case": c.name, "start": c.start, "goal": c.goal, "vel": c.vel, "accel": c.accel}
		_, err := pl2.PointToPoint(c.start, c.goal, c.vel, c.accel)
		mt.CheckReason(c.name, in, err, c.reason)
	}
}

func abs(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}
func maxf(a, b float64) float64 {
	if a > b {
		return a
	}
	return b
}
func nan() float64          { return float64NaN() }
func ftoa(x float64) string { return fmtSprint(x) }
