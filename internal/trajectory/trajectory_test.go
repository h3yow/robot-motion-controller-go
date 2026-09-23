package trajectory

import (
	"math"
	"testing"
	"time"

	"github.com/h3yow/robot-motion-controller-go/internal/mcerr"
)

var dt = 2 * time.Millisecond

func testLimits() []AxisLimits {
	return []AxisLimits{
		{Name: "j1", MinPosition: -3, MaxPosition: 3, MaxVelocity: 2, MaxAccel: 4, MaxTorque: 20,
			Inertia: 1, GravityTorque: 1, FrictionTorque: 0.5},
		{Name: "j2", MinPosition: -2.5, MaxPosition: 2.5, MaxVelocity: 2.5, MaxAccel: 5, MaxTorque: 18,
			Inertia: 0.8, GravityTorque: 1, FrictionTorque: 0.5},
		{Name: "j3", MinPosition: -3, MaxPosition: 3, MaxVelocity: 3, MaxAccel: 6, MaxTorque: 12,
			Inertia: 0.4, GravityTorque: 0.5, FrictionTorque: 0.3},
	}
}

func TestPlanBasicAndEndpoints(t *testing.T) {
	lim := testLimits()
	req := &Request{
		StartPosition: []float64{0, 0, 0},
		StartVelocity: []float64{0, 0, 0},
		GoalPosition:  []float64{1, -1, 0.5},
		Limits:        lim, Period: dt,
	}
	tr, err := Plan(req)
	if err != nil {
		t.Fatalf("Plan: %v", err)
	}
	n := tr.Len()
	if n < 2 {
		t.Fatalf("trajectory too short: %d", n)
	}
	first, last := tr.At(0), tr.At(n-1)
	for i := 0; i < 3; i++ {
		if math.Abs(first.Position[i]-0) > 1e-9 || math.Abs(first.Velocity[i]) > 1e-9 {
			t.Errorf("start axis %d = (%.5f,%.5f), want (0,0)", i, first.Position[i], first.Velocity[i])
		}
		goal := req.GoalPosition[i]
		if math.Abs(last.Position[i]-goal) > 1e-9 {
			t.Errorf("goal axis %d = %.6f, want %.6f", i, last.Position[i], goal)
		}
		if math.Abs(last.Velocity[i]) > 1e-9 {
			t.Errorf("final velocity axis %d = %.6f, want 0", i, last.Velocity[i])
		}
	}
}

func TestPlanEnforcesPeakLimits(t *testing.T) {
	lim := testLimits()
	tr, err := Plan(&Request{
		StartPosition: []float64{0, 0, 0}, StartVelocity: []float64{0, 0, 0},
		GoalPosition: []float64{1.5, -2, 0.8}, Limits: lim, Period: dt,
	})
	if err != nil {
		t.Fatalf("Plan: %v", err)
	}
	for k := 0; k < tr.Len(); k++ {
		p := tr.At(k)
		for i := 0; i < 3; i++ {
			if math.Abs(p.Velocity[i]) > lim[i].MaxVelocity+1e-9 {
				t.Fatalf("axis %d velocity %.6f > %.6f at k=%d", i, p.Velocity[i], lim[i].MaxVelocity, k)
			}
			if math.Abs(p.Accel[i]) > lim[i].MaxAccel+1e-9 {
				t.Fatalf("axis %d accel %.6f > %.6f at k=%d", i, p.Accel[i], lim[i].MaxAccel, k)
			}
			if p.Position[i] < lim[i].MinPosition-1e-9 || p.Position[i] > lim[i].MaxPosition+1e-9 {
				t.Fatalf("axis %d position %.6f out of travel at k=%d", i, p.Position[i], k)
			}
			if math.Abs(p.Torque[i]) > lim[i].MaxTorque+1e-9 {
				t.Fatalf("axis %d torque %.6f > %.6f at k=%d", i, p.Torque[i], lim[i].MaxTorque, k)
			}
		}
	}
}

func TestPlanRejectionReasons(t *testing.T) {
	lim := testLimits()
	base := func(goal []float64, startV []float64) *Request {
		return &Request{StartPosition: []float64{0, 0, 0}, StartVelocity: startV,
			GoalPosition: goal, Limits: lim, Period: dt}
	}
	cases := []struct {
		name string
		req  *Request
		code mcerr.Code
	}{
		{"goal beyond travel", base([]float64{4, 0, 0}, []float64{0, 0, 0}), mcerr.CodeJointLimitExceeded},
		{"goal non-finite", base([]float64{math.NaN(), 0, 0}, []float64{0, 0, 0}), mcerr.CodeTargetNonFinite},
		{"start too fast", base([]float64{1, 0, 0}, []float64{5, 0, 0}), mcerr.CodeInfeasibleStart},
		{"cardinality mismatch", &Request{StartPosition: []float64{0}, StartVelocity: []float64{0},
			GoalPosition: []float64{1}, Limits: lim, Period: dt}, mcerr.CodeTargetContradictory},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			_, err := Plan(tc.req)
			if err == nil {
				t.Fatalf("expected rejection %s, accepted", tc.code)
			}
			if !mcerr.HasCode(err, tc.code) {
				t.Fatalf("want %s got %v; request=%+v", tc.code, err, tc.req)
			}
		})
	}
}

func TestTorqueLimitRejected(t *testing.T) {
	lim := testLimits()
	// Force the inertia demand above torque capacity: accel 6 needs 6 N·m;
	// shrink max torque below J*A + passive load.
	lim[0].MaxTorque = 6.0 // J*A = 4, passive = 1.5 -> need 5.5 < 6 ok
	lim[0].MaxTorque = 5.0 // need 5.5 -> rejected
	_, err := Plan(&Request{StartPosition: []float64{0, 0, 0}, StartVelocity: []float64{0, 0, 0},
		GoalPosition: []float64{1, 0, 0}, Limits: lim, Period: dt})
	if !mcerr.HasCode(err, mcerr.CodeTorqueLimit) {
		t.Fatalf("want torque_limit_exceeded, got %v", err)
	}
}

func TestNonzeroStartVelocityBrakesFirst(t *testing.T) {
	lim := testLimits()
	// Moving at +1.5 rad/s while goal is behind start: must brake to rest
	// without exceeding limits and reach the goal exactly.
	req := &Request{StartPosition: []float64{0, 0, 0}, StartVelocity: []float64{1.5, 0, 0},
		GoalPosition: []float64{-0.5, 0, 0}, Limits: lim, Period: dt}
	tr, err := Plan(req)
	if err != nil {
		t.Fatalf("Plan: %v", err)
	}
	for k := 0; k < tr.Len(); k++ {
		p := tr.At(k)
		if math.Abs(p.Velocity[0]) > lim[0].MaxVelocity+1e-9 {
			t.Fatalf("velocity %.6f exceeds limit", p.Velocity[0])
		}
		if math.Abs(p.Accel[0]) > lim[0].MaxAccel+1e-9 {
			t.Fatalf("accel %.6f exceeds limit at k=%d", p.Accel[0], k)
		}
	}
	last := tr.Final()
	if math.Abs(last.Position[0]+0.5) > 1e-9 || math.Abs(last.Velocity[0]) > 1e-9 {
		t.Fatalf("endpoint = %.6f, %.6f", last.Position[0], last.Velocity[0])
	}
	// Early samples must show braking (negative accel while velocity positive).
	p1 := tr.At(1)
	if p1.Accel[0] >= 0 {
		t.Fatalf("expected initial braking accel < 0, got %.4f", p1.Accel[0])
	}
}

func TestMultiAxisSynchronizedEnds(t *testing.T) {
	lim := testLimits()
	tr, err := Plan(&Request{StartPosition: []float64{0, 0, 0}, StartVelocity: []float64{0, 0, 0},
		GoalPosition: []float64{0.1, 2, 0.05}, Limits: lim, Period: dt})
	if err != nil {
		t.Fatalf("Plan: %v", err)
	}
	last := tr.Final()
	for i, g := range []float64{0.1, 2, 0.05} {
		if math.Abs(last.Position[i]-g) > 1e-9 {
			t.Fatalf("axis %d final %.6f != goal %.6f (axes must finish together)", i, last.Position[i], g)
		}
	}
}

func TestDeterministic(t *testing.T) {
	lim := testLimits()
	req := &Request{StartPosition: []float64{0.1, -0.2, 0}, StartVelocity: []float64{0, 0, 0},
		GoalPosition: []float64{0.8, 0.6, -0.4}, Limits: lim, Period: dt}
	tr1, err := Plan(req)
	if err != nil {
		t.Fatalf("Plan: %v", err)
	}
	tr2, err := Plan(req)
	if err != nil {
		t.Fatalf("Plan2: %v", err)
	}
	if tr1.Len() != tr2.Len() {
		t.Fatalf("lengths differ %d vs %d", tr1.Len(), tr2.Len())
	}
	for k := 0; k < tr1.Len(); k++ {
		a, b := tr1.At(k), tr2.At(k)
		for i := 0; i < 3; i++ {
			if a.Position[i] != b.Position[i] || a.Velocity[i] != b.Velocity[i] || a.Accel[i] != b.Accel[i] {
				t.Fatalf("non-deterministic sample k=%d axis=%d", k, i)
			}
		}
	}
}

func TestContinuousInGoal(t *testing.T) {
	lim := testLimits()
	mk := func(g float64) float64 {
		tr, err := Plan(&Request{StartPosition: []float64{0, 0, 0}, StartVelocity: []float64{0, 0, 0},
			GoalPosition: []float64{g, 0, 0}, Limits: lim, Period: dt})
		if err != nil {
			t.Fatalf("Plan g=%.6f: %v", g, err)
		}
		return tr.Final().Position[0]
	}
	g1, g2 := 0.5, 0.5+1e-9
	if math.Abs(mk(g1)-g1) > 1e-9 || math.Abs(mk(g2)-g2) > 1e-9 {
		t.Fatal("endpoint must equal goal exactly (continuous)")
	}
}

func TestPlanPathStopsAtWaypoints(t *testing.T) {
	lim := testLimits()
	tr, err := PlanPath(&PathRequest{
		StartPosition: []float64{0, 0, 0}, StartVelocity: []float64{0, 0, 0},
		Waypoints: [][]float64{{1, 0, 0}, {1, 1, 0}, {0, 0, 0}},
		Limits:    lim, Period: dt,
	})
	if err != nil {
		t.Fatalf("PlanPath: %v", err)
	}
	// Find a sample at the first waypoint and assert zero velocity there.
	found := false
	for k := 0; k < tr.Len(); k++ {
		p := tr.At(k)
		if math.Abs(p.Position[0]-1) < 1e-9 && math.Abs(p.Position[1]) < 1e-9 && math.Abs(p.Position[2]) < 1e-9 {
			speed := math.Abs(p.Velocity[0]) + math.Abs(p.Velocity[1]) + math.Abs(p.Velocity[2])
			if speed < 1e-9 {
				found = true
			}
		}
	}
	if !found {
		t.Fatal("path never rests at first waypoint")
	}
	last := tr.Final()
	for i := 0; i < 3; i++ {
		if math.Abs(last.Position[i]) > 1e-9 {
			t.Fatalf("path final axis %d = %.6f, want 0", i, last.Position[i])
		}
	}
}

// TestContinuousInParameters asserts that slightly perturbing a goal changes
// the resulting samples only slightly (no discontinuous replanning).
func TestContinuousInParameters(t *testing.T) {
	lim := testLimits()
	mk := func(g float64) *Trajectory {
		tr, err := Plan(&Request{StartPosition: []float64{0, 0, 0}, StartVelocity: []float64{0, 0, 0},
			GoalPosition: []float64{g, 0, 0}, Limits: lim, Period: dt})
		if err != nil {
			t.Fatalf("plan g=%.6f: %v", g, err)
		}
		return tr
	}
	a, b := mk(1.0), mk(1.0+1e-6)
	if d := absF(float64(b.Len() - a.Len())); d > 2 {
		t.Fatalf("sample count jumps by %v for a 1e-6 goal perturbation", d)
	}
	// Compare at matched normalized progress: final positions differ by
	// exactly the goal delta and never overshoot it disproportionately.
	d := b.Final().Position[0] - a.Final().Position[0]
	if absF(d-1e-6) > 1e-9 {
		t.Fatalf("final-position delta %.3e must equal goal delta 1e-6", d)
	}
}

func absF(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}
