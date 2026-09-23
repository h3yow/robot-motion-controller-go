// Package motion is the high-level facade combining inverse kinematics,
// trajectory execution, fault handling and command arbitration for the
// simulated planar 2-link arm.
package motion

import (
	"fmt"

	"github.com/h3yow/robot-motion-controller-go/internal/motion/arbiter"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/classify"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/config"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/engine"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/kinematics"
)

// Robot is a simulated planar 2R robot driven by a motion engine.
type Robot struct {
	cfg *config.Config
	arm kinematics.Planar2R
	eng *engine.Engine
}

// NewRobot builds a Robot. cfg must pass Validate() and define exactly two
// axes; l1/l2 are the arm link lengths (m, strictly positive).
func NewRobot(cfg *config.Config, l1, l2 float64) (*Robot, error) {
	if err := cfg.Validate(); err != nil {
		return nil, err
	}
	if cfg.AxisCount() != 2 {
		return nil, classify.New(classify.ReasonConfigField,
			fmt.Sprintf("planar 2R robot needs exactly 2 axes, got %d", cfg.AxisCount()))
	}
	if !kinematics.IsFinite(l1, l2) || l1 <= 0 || l2 <= 0 {
		return nil, classify.New(classify.ReasonConfigField,
			fmt.Sprintf("link lengths must be finite and positive, got l1=%v l2=%v", l1, l2))
	}
	return &Robot{
		cfg: cfg,
		arm: kinematics.Planar2R{L1: l1, L2: l2},
		eng: engine.New(cfg),
	}, nil
}

// MoveToPose resolves a Cartesian target via inverse kinematics and submits
// the joint motion. Non-finite targets yield TARGET_NON_FINITE,
// out-of-workspace targets TARGET_UNREACHABLE, and joint solutions outside
// travel limits OUT_OF_TRAVEL, all before anything is arbitrated or moved.
func (r *Robot) MoveToPose(source string, seq int64, prio int, p kinematics.Vec, elbowUp bool) (arbiter.Decision, error) {
	q, err := r.arm.Inverse(p, elbowUp)
	if err != nil {
		return arbiter.Decision{}, err
	}
	goal := []float64{q.Q1, q.Q2}
	for i, g := range goal {
		a := r.cfg.Axes[i]
		if g < a.MinPosition || g > a.MaxPosition {
			return arbiter.Decision{}, classify.New(classify.ReasonOutOfTravel,
				fmt.Sprintf("IK solution axis[%d] (%s)=%v outside travel [%v,%v] for pose (%v,%v)",
					i, a.Name, g, a.MinPosition, a.MaxPosition, p.X, p.Y))
		}
	}
	cmd := arbiter.Command{
		Source: source, Sequence: seq, Priority: prio,
		Op: arbiter.OpMoveTo, Goal: goal,
	}
	return r.eng.Submit(cmd)
}

// Arm returns the kinematic model.
func (r *Robot) Arm() kinematics.Planar2R { return r.arm }

// Engine exposes the underlying engine.
func (r *Robot) Engine() *engine.Engine { return r.eng }
