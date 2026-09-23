package sim

import (
	"math"
	"testing"
	"time"

	"github.com/h3yow/robot-motion-controller-go/internal/config"
	"github.com/h3yow/robot-motion-controller-go/internal/mcerr"
)

func testCfg() *config.Config {
	cfg := &config.Config{
		Version: "1.0.0", Period: 2 * time.Millisecond,
		Links: []float64{0.4, 0.3, 0.1}, Elbow: "up",
		Axes: []config.AxisConfig{
			{Name: "j1", HomePosition: 0, MinPosition: -2, MaxPosition: 2,
				MaxVelocity: 5, MaxAccel: 20, MaxTorque: 20, Inertia: 1,
				GravityTorque: 0, FrictionTorque: 0.2, Gains: config.PIDGains{Kp: 10, Ki: 1, Kd: 2, IntegralLimit: 5}},
			{Name: "j2", HomePosition: 0, MinPosition: -2, MaxPosition: 2,
				MaxVelocity: 5, MaxAccel: 20, MaxTorque: 20, Inertia: 1,
				GravityTorque: 0, FrictionTorque: 0.2, Gains: config.PIDGains{Kp: 10, Ki: 1, Kd: 2, IntegralLimit: 5}},
			{Name: "j3", HomePosition: 0, MinPosition: -2, MaxPosition: 2,
				MaxVelocity: 5, MaxAccel: 20, MaxTorque: 20, Inertia: 1,
				GravityTorque: 0, FrictionTorque: 0.2, Gains: config.PIDGains{Kp: 10, Ki: 1, Kd: 2, IntegralLimit: 5}},
		},
		Tolerance: config.Tolerance{Position: 0.002, Velocity: 0.02, HoldTicks: 3},
		Fault: config.FaultPolicy{MaxPositionError: 0.2, DivergeError: 0.6,
			DropoutMaxTicks: 2, SaturationMaxTicks: 10, EndGraceTicks: 30,
			RecoveryHoldTicks: 2, SafeDecel: 10},
	}
	return cfg
}

func TestDeterministic(t *testing.T) {
	run := func() []float64 {
		p := NewPlant(testCfg())
		for k := 0; k < 100; k++ {
			p.Step([]float64{1, -1, 0.5})
		}
		q, _ := p.State()
		return q
	}
	a, b := run(), run()
	for i := range a {
		if a[i] != b[i] {
			t.Fatalf("plant non-deterministic axis %d: %.12f vs %.12f", i, a[i], b[i])
		}
	}
}

func TestStaticFrictionHoldsSmallTorque(t *testing.T) {
	p := NewPlant(testCfg())
	// 0.1 N·m is below the 0.2 N·m Coulomb friction: the axis must stay put.
	for k := 0; k < 50; k++ {
		q, v, err := p.Step([]float64{0.1, 0, 0})
		if err != nil {
			t.Fatal(err)
		}
		if math.Abs(q[0]) > 0 || math.Abs(v[0]) > 0 {
			t.Fatalf("static friction failed to hold: q=%.5f v=%.5f at k=%d", q[0], v[0], k)
		}
	}
}

func TestConstantTorqueAccelerates(t *testing.T) {
	p := NewPlant(testCfg())
	for k := 0; k < 50; k++ {
		p.Step([]float64{2, 0, 0})
	}
	q, v := p.State()
	if v[0] <= 0 {
		t.Fatalf("expected positive velocity under positive torque, got v=%.4f q=%.4f", v[0], q[0])
	}
}

func TestHardStop(t *testing.T) {
	p := NewPlant(testCfg())
	for k := 0; k < 1000; k++ {
		p.Step([]float64{20, 0, 0})
	}
	q, v := p.State()
	if q[0] > 2+1e-12 {
		t.Fatalf("joint exceeded hard stop: q=%.5f", q[0])
	}
	if math.Abs(v[0]) > 1e-9 {
		t.Fatalf("velocity should be zero at hard stop, got %.5f", v[0])
	}
}

func TestTorqueLossStopsAcceleration(t *testing.T) {
	p := NewPlant(testCfg())
	p.SetTorqueScale([]float64{0, 1, 1})
	for k := 0; k < 50; k++ {
		p.Step([]float64{5, 0, 0})
	}
	q, v := p.State()
	if math.Abs(q[0]) > 0 || math.Abs(v[0]) > 0 {
		t.Fatalf("zero torque scale must produce no motion, got q=%.5f v=%.5f", q[0], v[0])
	}
}

func TestNonFiniteCommandRejected(t *testing.T) {
	p := NewPlant(testCfg())
	if _, _, err := p.Step([]float64{math.NaN(), 0, 0}); !mcerr.HasCode(err, mcerr.CodeCommandNonFinite) {
		t.Fatalf("want command_non_finite, got %v", err)
	}
	qBefore, _ := p.State()
	p.Step([]float64{1, 0, 0})
	// After the rejection the plant must still step normally.
	if _, _, err := p.Step([]float64{math.Inf(1), 0, 0}); !mcerr.HasCode(err, mcerr.CodeCommandNonFinite) {
		t.Fatalf("want command_non_finite for Inf, got %v", err)
	}
	qAfter, _ := p.State()
	if qBefore[0] == qAfter[0] {
		// sanity: healthy step in between moved the plant
		t.Fatalf("plant did not advance on healthy step between bad commands")
	}
}
