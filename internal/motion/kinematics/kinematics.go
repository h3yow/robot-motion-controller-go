// Package kinematics provides a deterministic planar 2-link arm model used
// to demonstrate reachable/unreachable target classification.
//
// Conventions: q1 is the shoulder angle measured from the +X axis; q2 is
// the elbow angle measured relative to the first link. All angles in rad.
package kinematics

import (
	"math"

	"github.com/h3yow/robot-motion-controller-go/internal/motion/classify"
)

// Vec is a 2D Cartesian vector (metres).
type Vec struct {
	X, Y float64
}

// Joints is a 2-axis joint vector (angles in radians).
type Joints struct {
	Q1, Q2 float64
}

// Planar2R is a planar two-link arm. Link lengths are in metres and must be
// strictly positive.
type Planar2R struct {
	L1 float64
	L2 float64
}

// Forward returns the Cartesian position of the end effector.
func (a Planar2R) Forward(q Joints) Vec {
	x := a.L1*math.Cos(q.Q1) + a.L2*math.Cos(q.Q1+q.Q2)
	y := a.L1*math.Sin(q.Q1) + a.L2*math.Sin(q.Q1+q.Q2)
	return Vec{X: x, Y: y}
}

// Reachable reports whether the point p lies within the arm workspace,
// accounting for floating point boundary points. Non-finite inputs are not
// reachable.
func (a Planar2R) Reachable(p Vec) bool {
	if !IsFinite(p.X, p.Y, a.L1, a.L2) || a.L1 <= 0 || a.L2 <= 0 {
		return false
	}
	r := math.Hypot(p.X, p.Y)
	rMin := math.Abs(a.L1 - a.L2)
	rMax := a.L1 + a.L2
	eps := 1e-9 * math.Max(1.0, rMax)
	return r >= rMin-eps && r <= rMax+eps
}

// Inverse returns the joint angles for a Cartesian target. elbowUp selects
// the solution branch. Non-finite targets are rejected with
// ReasonTargetNonFinite; out-of-workspace targets with
// ReasonTargetUnreachable. The two reasons are always distinguishable.
func (a Planar2R) Inverse(p Vec, elbowUp bool) (Joints, error) {
	if !IsFinite(p.X, p.Y) {
		return Joints{}, classify.New(classify.ReasonTargetNonFinite,
			"target position contains NaN or Inf")
	}
	if !IsFinite(a.L1, a.L2) || a.L1 <= 0 || a.L2 <= 0 {
		return Joints{}, classify.New(classify.ReasonConfigField,
			"Planar2R link lengths must be finite and positive")
	}
	r2 := p.X*p.X + p.Y*p.Y
	cosQ2 := (r2 - a.L1*a.L1 - a.L2*a.L2) / (2 * a.L1 * a.L2)
	// Clamp only the tiny floating-point overshoot at the workspace
	// boundary; genuinely unreachable radii are rejected below.
	if cosQ2 > 1 || cosQ2 < -1 {
		r := math.Sqrt(r2)
		rMin := math.Abs(a.L1 - a.L2)
		rMax := a.L1 + a.L2
		eps := 1e-9 * math.Max(1.0, rMax)
		if r < rMin-eps || r > rMax+eps {
			return Joints{}, classify.New(classify.ReasonTargetUnreachable,
				"target radius outside workspace [rmin,rmax]")
		}
		if cosQ2 > 1 {
			cosQ2 = 1
		} else {
			cosQ2 = -1
		}
	}
	q2 := math.Acos(cosQ2)
	if elbowUp {
		q2 = -q2
	}
	q1 := math.Atan2(p.Y, p.X) -
		math.Atan2(a.L2*math.Sin(q2), a.L1+a.L2*math.Cos(q2))
	return Joints{Q1: q1, Q2: q2}, nil
}

// IsFinite reports whether all values are finite (not NaN/+-Inf).
func IsFinite(xs ...float64) bool {
	for _, x := range xs {
		if math.IsNaN(x) || math.IsInf(x, 0) {
			return false
		}
	}
	return true
}
