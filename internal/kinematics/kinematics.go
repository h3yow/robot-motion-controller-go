// Package kinematics implements a planar serial revolute arm: analytical
// forward kinematics and a closed-form inverse kinematics solution with a
// fixed elbow configuration, plus explicit reachability checks.
//
// Convention (SI units):
//
//   - joint angles q in radians, measured counter-clockwise;
//   - links l1..ln in meters;
//   - forward kinematics: x = sum lk cos(sum_{j<=k} qj), likewise y;
//   - end-effector orientation phi = sum qj.
//
// The supported arm has 3 DOF: the first two joints place the wrist center,
// the third orients the end effector. This keeps IK fully closed-form and
// therefore deterministic and continuous in the target.
package kinematics

import (
	"fmt"
	"math"

	"github.com/h3yow/robot-motion-controller-go/internal/mcerr"
	"github.com/h3yow/robot-motion-controller-go/internal/numeric"
)

// Pose is a planar end-effector pose. X/Y in meters, Phi in radians.
type Pose struct {
	X   float64
	Y   float64
	Phi float64
}

// supportedDOF is the only supported joint count.
const supportedDOF = 3

// Arm is an n-link planar revolute arm (n == 3 supported).
type Arm struct {
	links []float64
	elbow int // +1 = elbow up, -1 = elbow down
}

// NewArm constructs an arm. elbow must be "up" or "down"; links must be
// positive, finite and match a supported DOF count.
func NewArm(links []float64, elbow string) (*Arm, error) {
	if len(links) != supportedDOF {
		return nil, mcerr.New(mcerr.CodeConfigInvalid,
			fmt.Sprintf("exactly %d links are supported, got %d", supportedDOF, len(links)))
	}
	if !numeric.AllFinite(links) {
		return nil, mcerr.New(mcerr.CodeConfigInvalid, "link lengths must be finite")
	}
	for i, l := range links {
		if !(l > 0) {
			return nil, mcerr.New(mcerr.CodeConfigInvalid,
				fmt.Sprintf("links[%d] must be strictly positive, got %v", i, l))
		}
	}
	sign := 0
	switch elbow {
	case "up":
		sign = 1
	case "down":
		sign = -1
	default:
		return nil, mcerr.New(mcerr.CodeConfigInvalid, "elbow must be \"up\" or \"down\", got "+fmt.Sprintf("%q", elbow))
	}
	cp := make([]float64, len(links))
	copy(cp, links)
	return &Arm{links: cp, elbow: sign}, nil
}

// DOF returns the number of joints.
func (a *Arm) DOF() int { return len(a.links) }

// Links returns a copy of the link lengths.
func (a *Arm) Links() []float64 {
	out := make([]float64, len(a.links))
	copy(out, a.links)
	return out
}

// FK computes the end-effector pose for joint angles q (radians).
func (a *Arm) FK(q []float64) (Pose, error) {
	if len(q) != a.DOF() {
		return Pose{}, mcerr.New(mcerr.CodeConfigInvalid,
			fmt.Sprintf("FK expects %d joint angles, got %d", a.DOF(), len(q)))
	}
	if !numeric.AllFinite(q) {
		return Pose{}, mcerr.New(mcerr.CodeTargetNonFinite, "joint angles must be finite")
	}
	x, y, theta := 0.0, 0.0, 0.0
	for i := 0; i < a.DOF(); i++ {
		theta += q[i]
		x += a.links[i] * math.Cos(theta)
		y += a.links[i] * math.Sin(theta)
	}
	return Pose{X: x, Y: y, Phi: theta}, nil
}

// IK solves joint angles for the given target pose. The seed selects the
// branch closest to the current configuration (joint angles are shifted by
// multiples of 2*pi to remain near the seed). Unreachable or non-finite
// targets are rejected explicitly with CodeTargetUnreachable /
// CodeTargetNonFinite.
func (a *Arm) IK(target Pose, seed []float64) ([]float64, error) {
	if !numeric.Finite(target.X) || !numeric.Finite(target.Y) || !numeric.Finite(target.Phi) {
		return nil, mcerr.New(mcerr.CodeTargetNonFinite,
			fmt.Sprintf("target pose must be finite, got (%.6g, %.6g, %.6g)", target.X, target.Y, target.Phi))
	}
	if len(seed) != a.DOF() {
		return nil, mcerr.New(mcerr.CodeConfigInvalid,
			fmt.Sprintf("IK seed must have %d entries, got %d", a.DOF(), len(seed)))
	}
	l1, l2, l3 := a.links[0], a.links[1], a.links[2]
	// Wrist center: subtract the last link along the end-effector orientation.
	rx := target.X - l3*math.Cos(target.Phi)
	ry := target.Y - l3*math.Sin(target.Phi)
	r := math.Hypot(rx, ry)

	// Workspace of the two-link positioning chain.
	rMax := l1 + l2
	rMin := math.Abs(l1 - l2)
	// Tolerance only for boundary rounding (full extension/fold); genuine
	// out-of-workspace targets are still rejected decisively.
	const tol = 1e-9
	if r > rMax+tol || r < rMin-tol {
		return nil, mcerr.New(mcerr.CodeTargetUnreachable,
			fmt.Sprintf("wrist center distance %.6f m is outside reachable annulus [%.6f, %.6f] m",
				r, rMin, rMax))
	}
	// cosine law for the elbow angle; clamp cos to [-1,1] for boundary
	// targets to avoid spurious NaN from rounding.
	cosQ2 := numeric.Clamp((r*r-l1*l1-l2*l2)/(2*l1*l2), -1.0, 1.0)
	q2 := float64(a.elbow) * math.Acos(cosQ2)
	q1 := math.Atan2(ry, rx) - float64(a.elbow)*
		math.Atan2(l2*math.Sin(math.Abs(q2)), l1+l2*math.Cos(q2))
	q3 := target.Phi - q1 - q2

	q := []float64{q1, q2, q3}
	// Choose equivalent-angle representation nearest to the seed so that
	// joint-space motion stays small and continuity holds.
	for i := range q {
		q[i] = nearestEquivalent(q[i], seed[i])
		if !numeric.Finite(q[i]) {
			return nil, mcerr.New(mcerr.CodeTargetNonFinite, "IK produced a non-finite joint angle")
		}
	}
	return q, nil
}

// nearestEquivalent shifts angle by multiples of 2*pi to minimize distance
// to seed.
func nearestEquivalent(angle, seed float64) float64 {
	k := math.Round((seed - angle) / (2 * math.Pi))
	return angle + k*2*math.Pi
}
