package kinematics_test

import (
	"math"
	"testing"

	"github.com/h3yow/robot-motion-controller-go/internal/motion/classify"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/kinematics"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/mctest"
)

func TestForwardInverseRoundTrip(t *testing.T) {
	mt := mctest.Wrap(t)
	a := kinematics.Planar2R{L1: 1, L2: 0.8}
	poses := []kinematics.Vec{{X: 1.5, Y: 0.2}, {X: 0.5, Y: 0.5}, {X: -0.3, Y: 1.1}, {X: 0.2, Y: -1.3}}
	for _, p := range poses {
		q, err := a.Inverse(p, false)
		mt.CheckNoErr("inverse", p, err)
		f := a.Forward(q)
		mt.CheckApprox("roundtrip x", p, f.X, p.X, 1e-9)
		mt.CheckApprox("roundtrip y", p, f.Y, p.Y, 1e-9)
	}
}

func TestInverseRejections(t *testing.T) {
	mt := mctest.Wrap(t)
	a := kinematics.Planar2R{L1: 1, L2: 0.6}
	_, err := a.Inverse(kinematics.Vec{X: math.NaN(), Y: 0}, false)
	mt.CheckReason("nan", "NaN x", err, classify.ReasonTargetNonFinite)
	_, err = a.Inverse(kinematics.Vec{X: 5, Y: 0}, false)
	mt.CheckReason("too far", "r=5", err, classify.ReasonTargetUnreachable)
	_, err = a.Inverse(kinematics.Vec{X: 0.1, Y: 0}, false)
	mt.CheckReason("inner radius", "r=0.1<0.4", err, classify.ReasonTargetUnreachable)

	bad := kinematics.Planar2R{L1: 0, L2: 1}
	_, err = bad.Inverse(kinematics.Vec{X: 0.5, Y: 0}, false)
	mt.CheckReason("bad model", "l1=0", err, classify.ReasonConfigField)
}

func TestDeterminism(t *testing.T) {
	mt := mctest.Wrap(t)
	a := kinematics.Planar2R{L1: 1.2, L2: 0.7}
	p := kinematics.Vec{X: 0.9, Y: -0.4}
	q1, _ := a.Inverse(p, true)
	q2, _ := a.Inverse(p, true)
	mt.CheckEq("deterministic q1", p, q2.Q1, q1.Q1)
	mt.CheckEq("deterministic q2", p, q2.Q2, q1.Q2)
	q3, _ := a.Inverse(kinematics.Vec{X: 0.900001, Y: -0.4}, true)
	mt.CheckTrue("continuous", "dx=1e-6", math.Abs(q3.Q1-q1.Q1) < 1e-4 && math.Abs(q3.Q2-q1.Q2) < 1e-4,
		"IK must be continuous in the target")
}
