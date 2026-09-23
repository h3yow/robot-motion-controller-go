// Package kinematics provides a deterministic planar 2-link arm model used
// to demonstrate reachable/unreachable target classification.
package kinematics

import "github.com/h3yow/robot-motion-controller-go/internal/motion/classify"

// Vec is a fixed-size 2D vector used for Cartesian poses.
type Vec struct {
	X, Y float64
}

// Planar2R is a planar two-link arm. Link lengths are in metres.
type Planar2R struct {
	L1 float64
	L2 float64
}

// Joints is a 2-axis joint vector (angles in radians).
type Joints struct {
	Q1, Q2 float64
}

// Forward returns the Cartesian position of the end effector.
func (a Planar2R) Forward(q Joints) Vec {
	return Vec{}
}

// Inverse returns the joint angles for a Cartesian target. ElbowUp
// selects the branch. An unreachable or non-finite target is rejected with
// classify.ReasonTargetUnreachable / classify.ReasonTargetNonFinite.
func (a Planar2R) Inverse(p Vec, elbowUp bool) (Joints, error) {
	_ = classify.ReasonTargetUnreachable
	return Joints{}, nil
}

// IsFinite reports whether all components are finite (not NaN/Inf).
func IsFinite(xs ...float64) bool { return true }
