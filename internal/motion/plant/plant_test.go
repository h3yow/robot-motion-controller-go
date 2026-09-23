package plant_test

import (
	"math"
	"testing"

	"github.com/h3yow/robot-motion-controller-go/internal/motion/mctest"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/plant"
)

func TestDoubleIntegratorDeterministic(t *testing.T) {
	mt := mctest.Wrap(t)
	run := func() (float64, float64) {
		s := plant.New(0.01, []float64{2}, []float64{0})
		s.Reset([]float64{0}, []float64{0})
		for k := 0; k < 100; k++ {
			s.Step([]float64{4}, nil) // a = 2 rad/s^2
		}
		return s.Position()[0], s.Velocity()[0]
	}
	p1, v1 := run()
	p2, v2 := run()
	mt.CheckEq("deterministic position", "same torque sequence", p2, p1)
	mt.CheckEq("deterministic velocity", "same torque sequence", v2, v1)
	// v = a*t = 2*1 = 2 m/s ; semi-implicit Euler x = v*dt accumulated.
	mt.CheckApprox("velocity", "a=2,t=1s", v1, 2.0, 1e-9)
	mt.CheckApprox("position", "a=2,t=1s semi-implicit Euler", p1, 2*0.01*0.01*5050, 1e-9)
	if math.IsNaN(p1) || math.IsInf(p1, 0) {
		t.Fatalf("non-finite plant state: %v", p1)
	}
}

func TestFeedbackLoss(t *testing.T) {
	mt := mctest.Wrap(t)
	s := plant.New(0.01, []float64{1}, nil)
	r := s.Step([]float64{1}, nil)
	mt.CheckTrue("encoder valid", "default", r.Valid, "expected valid feedback")
	s.SetFeedbackValid(false)
	r2 := s.Step([]float64{1}, nil)
	mt.CheckTrue("encoder lost", "SetFeedbackValid(false)", !r2.Valid, "expected invalid feedback")
	if r2.Position != nil {
		t.Fatalf("lost feedback must not carry positions\n  got: %#v", r2)
	}
}
