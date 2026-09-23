package kinematics

import (
	"math"
	"testing"

	"github.com/h3yow/robot-motion-controller-go/internal/mcerr"
)

func newTestArm(t *testing.T) *Arm {
	t.Helper()
	a, err := NewArm([]float64{0.4, 0.3, 0.1}, "up")
	if err != nil {
		t.Fatalf("NewArm: %v", err)
	}
	return a
}

func TestIKFKRoundTrip(t *testing.T) {
	a := newTestArm(t)
	poses := []Pose{
		{X: 0.8, Y: 0.0, Phi: 0},
		{X: 0.5, Y: 0.3, Phi: 0.5},
		{X: 0.2, Y: 0.5, Phi: -0.4},
		{X: 0.0, Y: 0.4, Phi: 1.2},
	}
	for _, p := range poses {
		q, err := a.IK(p, []float64{0, 0, 0})
		if err != nil {
			t.Fatalf("IK(%+v) rejected: %v", p, err)
		}
		got, err := a.FK(q)
		if err != nil {
			t.Fatalf("FK(%v): %v", q, err)
		}
		if d := math.Hypot(got.X-p.X, got.Y-p.Y); d > 1e-9 {
			t.Errorf("position mismatch: target=(%.3f,%.3f) got=(%.6f,%.6f) d=%.2e; q=%v",
				p.X, p.Y, got.X, got.Y, d, q)
		}
		if d := normAngle(got.Phi - p.Phi); math.Abs(d) > 1e-9 {
			t.Errorf("orientation mismatch: target=%.3f got=%.6f; q=%v", p.Phi, got.Phi, q)
		}
	}
}

func normAngle(x float64) float64 {
	for x > math.Pi {
		x -= 2 * math.Pi
	}
	for x < -math.Pi {
		x += 2 * math.Pi
	}
	return x
}

func TestIKUnreachable(t *testing.T) {
	a := newTestArm(t)
	bad := []Pose{
		{X: 10, Y: 10, Phi: 0},   // far outside workspace
		{X: 0.105, Y: 0, Phi: 0}, // wrist radius 0.005, below l1-l2 = 0.1
	}
	for _, p := range bad {
		_, err := a.IK(p, []float64{0, 0, 0})
		if err == nil {
			t.Fatalf("unreachable target %+v was accepted", p)
		}
		if !mcerr.HasCode(err, mcerr.CodeTargetUnreachable) {
			t.Fatalf("target %+v: want target_unreachable, got %v", p, err)
		}
	}
}

func TestIKNonFinite(t *testing.T) {
	a := newTestArm(t)
	for _, p := range []Pose{
		{X: math.NaN(), Y: 0, Phi: 0},
		{X: 0, Y: math.Inf(1), Phi: 0},
		{X: 0.1, Y: 0.1, Phi: math.NaN()},
	} {
		_, err := a.IK(p, []float64{0, 0, 0})
		if !mcerr.HasCode(err, mcerr.CodeTargetNonFinite) {
			t.Fatalf("pose %+v: want target_non_finite, got %v", p, err)
		}
	}
}

func TestIKContinuity(t *testing.T) {
	a := newTestArm(t)
	// Small target changes must produce small joint changes (same branch).
	seed := []float64{0.2, 0.4, 0.1}
	p0 := Pose{X: 0.45, Y: 0.35, Phi: 0.3}
	q0, err := a.IK(p0, seed)
	if err != nil {
		t.Fatalf("IK base: %v", err)
	}
	const eps = 1e-6
	p1 := Pose{X: p0.X + eps, Y: p0.Y, Phi: p0.Phi}
	q1, err := a.IK(p1, q0)
	if err != nil {
		t.Fatalf("IK perturbed: %v", err)
	}
	dq := 0.0
	for i := range q0 {
		dq += (q1[i] - q0[i]) * (q1[i] - q0[i])
	}
	if math.Sqrt(dq) > 1e-4 {
		t.Fatalf("discontinuous IK jump %.2e for target step %.0e; q0=%v q1=%v", math.Sqrt(dq), eps, q0, q1)
	}
}

func TestElbowConfigurations(t *testing.T) {
	up, _ := NewArm([]float64{0.4, 0.3, 0.1}, "up")
	down, _ := NewArm([]float64{0.4, 0.3, 0.1}, "down")
	p := Pose{X: 0.5, Y: 0.2, Phi: 0}
	qu, err := up.IK(p, []float64{0, 0, 0})
	if err != nil {
		t.Fatalf("up IK: %v", err)
	}
	qd, err := down.IK(p, []float64{0, 0, 0})
	if err != nil {
		t.Fatalf("down IK: %v", err)
	}
	// q2 signs must reflect the elbow choice.
	if qu[1]*qd[1] >= 0 {
		t.Fatalf("elbow configs not distinguished: up q2=%v, down q2=%v", qu[1], qd[1])
	}
	if _, err := NewArm([]float64{0.4, 0.3}, "up"); err == nil {
		t.Fatal("2-DOF arm must be rejected")
	}
	if _, err := NewArm([]float64{0.4, -0.3, 0.1}, "up"); err == nil {
		t.Fatal("negative link length must be rejected")
	}
	if _, err := NewArm([]float64{0.4, 0.3, 0.1}, "weird"); err == nil {
		t.Fatal("bad elbow must be rejected")
	}
}
