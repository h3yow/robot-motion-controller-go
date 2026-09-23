package control

import (
	"math"
	"testing"
	"time"

	"github.com/h3yow/robot-motion-controller-go/internal/config"
	"github.com/h3yow/robot-motion-controller-go/internal/mcerr"
	"github.com/h3yow/robot-motion-controller-go/internal/sim"
	"github.com/h3yow/robot-motion-controller-go/internal/trajectory"
)

func cfgForTest() *config.Config {
	return &config.Config{
		Version: "1.0.0", Period: 2 * time.Millisecond,
		Links: []float64{0.4, 0.3, 0.1}, Elbow: "up",
		Axes: []config.AxisConfig{
			{Name: "j1", HomePosition: 0, MinPosition: -2, MaxPosition: 2,
				MaxVelocity: 5, MaxAccel: 20, MaxTorque: 25, Inertia: 1,
				GravityTorque: 0, FrictionTorque: 0.2,
				Gains: config.PIDGains{Kp: 40, Ki: 8, Kd: 10, IntegralLimit: 8}},
			{Name: "j2", HomePosition: 0, MinPosition: -2, MaxPosition: 2,
				MaxVelocity: 5, MaxAccel: 20, MaxTorque: 25, Inertia: 1,
				GravityTorque: 0, FrictionTorque: 0.2,
				Gains: config.PIDGains{Kp: 40, Ki: 8, Kd: 10, IntegralLimit: 8}},
			{Name: "j3", HomePosition: 0, MinPosition: -2, MaxPosition: 2,
				MaxVelocity: 5, MaxAccel: 20, MaxTorque: 25, Inertia: 1,
				GravityTorque: 0, FrictionTorque: 0.2,
				Gains: config.PIDGains{Kp: 40, Ki: 8, Kd: 10, IntegralLimit: 8}},
		},
		Tolerance: config.Tolerance{Position: 0.005, Velocity: 0.02, HoldTicks: 3},
		Fault: config.FaultPolicy{MaxPositionError: 0.2, DivergeError: 0.6,
			DropoutMaxTicks: 2, SaturationMaxTicks: 15, EndGraceTicks: 40,
			RecoveryHoldTicks: 2, SafeDecel: 10},
	}
}

func limits(cfg *config.Config) []trajectory.AxisLimits {
	out := make([]trajectory.AxisLimits, len(cfg.Axes))
	for i, a := range cfg.Axes {
		out[i] = trajectory.AxisLimits{Name: a.Name, MinPosition: a.MinPosition, MaxPosition: a.MaxPosition,
			MaxVelocity: a.MaxVelocity, MaxAccel: a.MaxAccel, MaxTorque: a.MaxTorque,
			Inertia: a.Inertia, GravityTorque: a.GravityTorque, FrictionTorque: a.FrictionTorque}
	}
	return out
}

func planTraj(t *testing.T, cfg *config.Config, goal []float64) *trajectory.Trajectory {
	t.Helper()
	n := len(cfg.Axes)
	tr, err := trajectory.Plan(&trajectory.Request{
		StartPosition: make([]float64, n), StartVelocity: make([]float64, n),
		GoalPosition: goal, Limits: limits(cfg), Period: cfg.Period,
	})
	if err != nil {
		t.Fatalf("plan: %v", err)
	}
	return tr
}

// cycle runs one plant+controller period.
func cycle(t *testing.T, cfg *config.Config, ctrl *Controller, plant *sim.Plant,
	q0, v0 []float64, fb []bool, push []float64) (Phase, *Fault, []float64, []float64) {
	tau := ctrl.LastTorque()
	if push != nil {
		plant.SetLoadTorque(push)
	}
	q, v, err := plant.Step(tau)
	if err != nil {
		t.Fatalf("plant step: %v", err)
	}
	phase, f := ctrl.Tick(time.Time{}, q, v, fb)
	return phase, f, q, v
}

func TestNominalMoveConverges(t *testing.T) {
	cfg := cfgForTest()
	ctrl := New(cfg)
	ctrl.Reset([]float64{0, 0, 0})
	goal := []float64{0.5, -0.5, 0.25}
	plant := sim.NewPlant(cfg)
	if err := ctrl.Load(planTraj(t, cfg, goal)); err != nil {
		t.Fatal(err)
	}
	q, v := []float64{0, 0, 0}, []float64{0, 0, 0}
	// Prime the controller to emit the first torque.
	if _, f := ctrl.Tick(time.Time{}, q, v, []bool{true, true, true}); f != nil {
		t.Fatalf("prime: %+v", f)
	}
	phase := PhaseRunning
	var fault *Fault
	for k := 0; k < 5000 && phase != PhaseDone; k++ {
		phase, fault, q, v = cycle(t, cfg, ctrl, plant, q, v, []bool{true, true, true}, nil)
		if phase == PhaseFaulted {
			t.Fatalf("unexpected fault at k=%d: %+v status=%+v", k, fault, ctrl.Status())
		}
	}
	if phase != PhaseDone {
		t.Fatalf("move did not converge: phase=%s fault=%v status=%+v", phase, fault, ctrl.Status())
	}
	st := ctrl.Status()
	for i := 0; i < 3; i++ {
		if math.Abs(st[i].Position-goal[i]) > cfg.Tolerance.Position {
			t.Fatalf("axis %d final %.5f off goal %.3f by %.5f", i, st[i].Position, goal[i], st[i].Position-goal[i])
		}
	}
}

func TestFeedbackDropoutRecoverableThenFatal(t *testing.T) {
	cfg := cfgForTest()
	ctrl := New(cfg)
	ctrl.Reset([]float64{0, 0, 0})
	plant := sim.NewPlant(cfg)
	ctrl.Load(planTraj(t, cfg, []float64{0.5, 0, 0}))
	q, v := []float64{0, 0, 0}, []float64{0, 0, 0}
	ctrl.Tick(time.Time{}, q, v, []bool{true, true, true})

	phase, fault := PhaseRunning, (*Fault)(nil)
	for k := 0; k < 5000 && phase != PhaseFaulted; k++ {
		phase, fault, q, v = cycle(t, cfg, ctrl, plant, q, v, []bool{false, true, true}, nil)
	}
	if phase != PhaseFaulted || fault == nil || !fault.Recoverable || fault.Code != FaultFeedbackLost {
		t.Fatalf("want recoverable feedback_lost, got phase=%s fault=%+v", phase, fault)
	}
	if err := ctrl.Recover(); !mcerr.HasCode(err, mcerr.CodeRecoveryPrecondition) {
		t.Fatalf("Recover while feedback lost should be rejected, got %v", err)
	}
	// Persistent dropout escalates.
	for fault != nil && fault.Recoverable {
		phase, fault, q, v = cycle(t, cfg, ctrl, plant, q, v, []bool{false, true, true}, nil)
	}
	if fault == nil || fault.Recoverable || fault.Code != FaultFeedbackDead {
		t.Fatalf("want fatal feedback_dead, got %+v", fault)
	}
	if err := ctrl.Recover(); !mcerr.HasCode(err, mcerr.CodeFaultActive) {
		t.Fatalf("fatal Recover should be rejected, got %v", err)
	}
}

func TestRecoverySucceedsWhenPreconditionsHold(t *testing.T) {
	cfg := cfgForTest()
	ctrl := New(cfg)
	ctrl.Reset([]float64{0, 0, 0})
	plant := sim.NewPlant(cfg)
	ctrl.Load(planTraj(t, cfg, []float64{0.5, 0, 0}))
	q, v := []float64{0, 0, 0}, []float64{0, 0, 0}
	healthy := []bool{true, true, true}
	ctrl.Tick(time.Time{}, q, v, healthy)
	phase, fault := PhaseRunning, (*Fault)(nil)
	// Run almost the whole trajectory; stop one sample before the final
	// goal so the reference is essentially at rest, then drop feedback.
	idx, total := ctrl.Progress()
	for idx < total-2 && phase == PhaseRunning {
		phase, fault, q, v = cycle(t, cfg, ctrl, plant, q, v, healthy, nil)
		idx, total = ctrl.Progress()
		if fault != nil {
			t.Fatalf("setup fault: %+v (status=%+v)", fault, ctrl.Status())
		}
	}
	phase, fault, q, v = cycle(t, cfg, ctrl, plant, q, v, []bool{false, true, true}, nil)
	if fault == nil || !fault.Recoverable {
		t.Fatalf("want recoverable fault after dropout, got %+v", fault)
	}
	// With feedback restored and the reference at rest, the plant settles
	// under zero torque; hold until recovery preconditions are satisfied.
	recovered := false
	for k := 0; k < 200; k++ {
		phase, fault, q, v = cycle(t, cfg, ctrl, plant, q, v, healthy, nil)
		if ctrl.Recover() == nil {
			recovered = true
			break
		}
	}
	if !recovered {
		t.Fatalf("recovery preconditions never satisfied: status=%+v", ctrl.Status())
	}
	if ctrl.Phase() != PhaseRunning {
		t.Fatalf("phase after recover = %s, want running", ctrl.Phase())
	}
}

func TestTrackingDivergenceFatal(t *testing.T) {
	cfg := cfgForTest()
	cfg.Fault.DivergeError = 0.1
	ctrl := New(cfg)
	ctrl.Reset([]float64{0, 0, 0})
	ctrl.Load(planTraj(t, cfgForTest(), []float64{0.2, 0, 0}))
	q, v := []float64{0, 0, 0}, []float64{0, 0, 0}
	// First healthy tick, then a grossly off measurement -> fatal divergence.
	ctrl.Tick(time.Time{}, q, v, []bool{true, true, true})
	_, f := ctrl.Tick(time.Time{}, []float64{5, 0, 0}, []float64{0, 0, 0}, []bool{true, true, true})
	if f == nil || f.Recoverable || f.Code != FaultTrackingDiverged {
		t.Fatalf("want fatal tracking_diverged, got %+v", f)
	}
	if err := ctrl.Recover(); !mcerr.HasCode(err, mcerr.CodeFaultActive) {
		t.Fatalf("fatal recovery rejected expected, got %v", err)
	}
	// Remove feed-forward torque on axis 0 (actuator partially disabled)
	// and run weak feedback gains: feedback alone cannot keep up with the
	// trajectory, so the lag crosses 0.2 rad (recoverable), not 0.6 (fatal),
	// and torque stays below the ceiling (so it is tracking, not saturation).
	cfg2 := cfgForTest()
	for i := range cfg2.Axes {
		cfg2.Axes[i].Gains = config.PIDGains{Kp: 2.0, Ki: 0.0, Kd: 0.2, IntegralLimit: 0.1}
	}
	ctrl2 := New(cfg2)
	ctrl2.Reset([]float64{0, 0, 0})
	plant2 := sim.NewPlant(cfg2)
	plant2.SetTorqueScale([]float64{0.25, 1, 1})
	ctrl2.Load(planTraj(t, cfg2, []float64{0.8, 0, 0}))
	q2, v2 := []float64{0, 0, 0}, []float64{0, 0, 0}
	ctrl2.Tick(time.Time{}, q2, v2, []bool{true, true, true})
	phase2, fault2 := PhaseRunning, (*Fault)(nil)
	for k := 0; k < 5000 && phase2 != PhaseFaulted; k++ {
		phase2, fault2, q2, v2 = cycle(t, cfg2, ctrl2, plant2, q2, v2,
			[]bool{true, true, true}, nil)
	}
	if fault2 == nil || !fault2.Recoverable || fault2.Code != FaultTracking {
		t.Fatalf("want recoverable tracking_error with weak feedback, got %+v", fault2)
	}
}

func TestDriveSaturationRecoverable(t *testing.T) {
	cfg := cfgForTest()
	cfg.Axes[0].MaxTorque = 0.2
	ctrl := New(cfg)
	ctrl.Reset([]float64{0, 0, 0})
	plant := sim.NewPlant(cfg)
	// Trajectory planned with nominal torque capacity; controller clips to
	// the tiny per-axis ceiling and must flag sustained saturation.
	tr := planTraj(t, cfgForTest(), []float64{0.5, 0, 0})
	if err := ctrl.Load(tr); err != nil {
		t.Fatal(err)
	}
	q, v := []float64{0, 0, 0}, []float64{0, 0, 0}
	ctrl.Tick(time.Time{}, q, v, []bool{true, true, true})
	phase, fault := PhaseRunning, (*Fault)(nil)
	for k := 0; k < 5000 && phase != PhaseFaulted; k++ {
		phase, fault, q, v = cycle(t, cfg, ctrl, plant, q, v, []bool{true, true, true}, nil)
	}
	if fault == nil || !fault.Recoverable || fault.Code != FaultDriveSaturated {
		t.Fatalf("want recoverable drive_saturated, got %+v", fault)
	}
}

func TestCancelSafeStops(t *testing.T) {
	cfg := cfgForTest()
	ctrl := New(cfg)
	ctrl.Reset([]float64{0, 0, 0})
	plant := sim.NewPlant(cfg)
	ctrl.Load(planTraj(t, cfg, []float64{1, 0, 0}))
	q, v := []float64{0, 0, 0}, []float64{0, 0, 0}
	healthy := []bool{true, true, true}
	ctrl.Tick(time.Time{}, q, v, healthy)
	for k := 0; k < 100; k++ {
		_, _, q, v = cycle(t, cfg, ctrl, plant, q, v, healthy, nil)
	}
	if math.Abs(v[0]) < 0.05 {
		t.Fatal("setup expected appreciable velocity before cancel")
	}
	ctrl.Cancel()
	if ctrl.Phase() != PhaseStopping {
		t.Fatalf("phase after Cancel = %s want stopping", ctrl.Phase())
	}
	phase, fault := PhaseStopping, (*Fault)(nil)
	for k := 0; k < 5000 && phase != PhaseDone; k++ {
		phase, fault, q, v = cycle(t, cfg, ctrl, plant, q, v, healthy, nil)
		if phase == PhaseFaulted {
			t.Fatalf("fault during safe stop: %+v", fault)
		}
	}
	if phase != PhaseDone {
		t.Fatalf("safe stop did not settle, phase=%s", phase)
	}
	st := ctrl.Status()
	if math.Abs(st[0].Velocity) > cfg.Tolerance.Velocity {
		t.Fatalf("residual velocity after stop %.4f", st[0].Velocity)
	}
}

func TestLoadRejectedWhileFaulted(t *testing.T) {
	cfg := cfgForTest()
	ctrl := New(cfg)
	ctrl.Reset([]float64{0, 0, 0})
	ctrl.Load(planTraj(t, cfg, []float64{0.2, 0, 0}))
	ctrl.Tick(time.Time{}, []float64{5, 0, 0}, []float64{0, 0, 0}, []bool{true, true, true})
	if ctrl.Phase() != PhaseFaulted {
		t.Fatal("setup fault")
	}
	if err := ctrl.Load(planTraj(t, cfg, []float64{0.3, 0, 0})); !mcerr.HasCode(err, mcerr.CodeFaultActive) {
		t.Fatalf("Load while faulted should be rejected, got %v", err)
	}
}
