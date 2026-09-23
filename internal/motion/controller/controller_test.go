package controller_test

import (
	"testing"

	"github.com/h3yow/robot-motion-controller-go/internal/motion/config"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/controller"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/mctest"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/plant"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/trajectory"
)

func cfg() *config.Config {
	return &config.Config{
		SchemaVersion: 1, ControlPeriod: 0.002,
		Axes: []config.AxisConfig{
			{Name: "j1", MinPosition: -6, MaxPosition: 6, MaxVelocity: 5, MaxAccel: 20, MaxTorque: 40, Inertia: 1},
		},
		Controller: config.ControllerConfig{Kp: 60, Ki: 10, Kd: 14, IntegralWindup: 10, TrackingTolerance: 0.05, SaturationCycles: 10},
	}
}

func cmd(p, v, a float64) trajectory.Point {
	return trajectory.Point{Position: []float64{p}, Velocity: []float64{v}, Accel: []float64{a}}
}

func TestNominalTracking(t *testing.T) {
	mt := mctest.Wrap(t)
	c := cfg()
	pl := trajectory.NewPlanner(c)
	tr, err := pl.PointToPoint([]float64{0}, []float64{1}, []float64{3}, []float64{10})
	mt.CheckNoErr("plan", "0->1", err)
	sim := plant.New(c.ControlPeriod, []float64{1}, nil)
	ctrl := controller.New(c)
	var maxErr float64
	lastTorque = []float64{0}
	for _, pt := range tr.Points {
		fb := sim.Step(lastTorque, nil)
		tau, _, f := ctrl.Step(pt, fb)
		if f != nil {
			t.Fatalf("unexpected fault during nominal tracking\n  t=%v point=%#v fault=%v", pt.T, pt, f)
		}
		lastTorque = tau
		if e := abs(pt.Position[0] - fb.Position[0]); e > maxErr {
			maxErr = e
		}
	}
	mt.CheckApprox("settled", "end of trajectory", sim.Position()[0], 1, 0.02)
	mt.CheckTrue("within tolerance", "max tracking error", maxErr <= 0.05, "maxErr too big")
}

var lastTorque []float64

func TestFeedbackLostFault(t *testing.T) {
	mt := mctest.Wrap(t)
	c := cfg()
	ctrl := controller.New(c)
	sim := plant.New(c.ControlPeriod, []float64{1}, nil)
	sim.SetFeedbackValid(false)
	_, _, f := ctrl.Step(cmd(0, 0, 0), sim.Step([]float64{0}, nil))
	mt.CheckTrue("feedback-lost kind", "invalid encoder", f != nil && f.Kind == controller.FaultFeedbackLost,
		"expected FEEDBACK_LOST")
	mt.CheckTrue("recoverable", "FEEDBACK_LOST", f.Kind.Recoverable(), "feedback loss must be recoverable")
}

func TestTrackingFault(t *testing.T) {
	mt := mctest.Wrap(t)
	c := cfg()
	ctrl := controller.New(c)
	// Actual position far from command with a plant that cannot move (zero torque).
	fb := plant.EncoderReading{Valid: true, Position: []float64{1}, Velocity: []float64{0}}
	_, _, f := ctrl.Step(cmd(0, 0, 0), fb)
	mt.CheckTrue("tracking kind", "cmd 0 actual 1", f != nil && f.Kind == controller.FaultTrackingExceeded,
		"expected FEEDBACK_TRACKING_ERROR")
	mt.CheckTrue("recoverable", "tracking", f.Kind.Recoverable(), "tracking fault must be recoverable")
}

func TestSaturationFault(t *testing.T) {
	mt := mctest.Wrap(t)
	c := cfg()
	c.Axes[0].MaxTorque = 0.5 // tiny torque cap => any nonzero response saturates
	ctrl := controller.New(c)
	var f *controller.Fault
	// Command stays INSIDE the 0.05 tracking tolerance but away from zero,
	// so the only possible fault is persistent saturation.
	for k := 0; k < 20; k++ {
		fb := plant.EncoderReading{Valid: true, Position: []float64{0}, Velocity: []float64{0}}
		_, _, f = ctrl.Step(cmd(0.04, 0, 0), fb)
		if f != nil {
			break
		}
	}
	mt.CheckTrue("saturation kind", "tiny torque cap, step command",
		f != nil && f.Kind == controller.FaultDriveSaturated, "expected DRIVE_SATURATION_PERSISTENT")
	mt.CheckTrue("recoverable", "saturation", f.Kind.Recoverable(), "saturation must be recoverable")
}

func TestDivergenceFault(t *testing.T) {
	mt := mctest.Wrap(t)
	c := cfg()
	c.Controller.TrackingTolerance = 0.01
	ctrl := controller.New(c)
	// Hard divergence envelope is 100x tolerance = 1.0.
	fb := plant.EncoderReading{Valid: true, Position: []float64{-2.5}, Velocity: []float64{0}}
	_, _, f := ctrl.Step(cmd(0, 0, 0), fb)
	mt.CheckTrue("divergence kind", "monotonic growing error",
		f != nil && f.Kind == controller.FaultControlDiverged, "expected CONTROL_DIVERGED")
	mt.CheckTrue("unrecoverable", "divergence", !f.Kind.Recoverable(), "divergence must be unrecoverable")
}

func TestNonFiniteFault(t *testing.T) {
	mt := mctest.Wrap(t)
	c := cfg()
	ctrl := controller.New(c)
	fb := plant.EncoderReading{Valid: true, Position: []float64{nanF()}, Velocity: []float64{0}}
	_, _, f := ctrl.Step(cmd(0, 0, 0), fb)
	mt.CheckTrue("nonfinite kind", "NaN feedback",
		f != nil && f.Kind == controller.FaultNonFinite, "expected NON_FINITE_STATE")
	mt.CheckTrue("unrecoverable", "nonfinite", !f.Kind.Recoverable(), "non-finite must be unrecoverable")
}

func abs(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}
