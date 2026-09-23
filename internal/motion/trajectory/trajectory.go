// Package trajectory generates deterministic per-control-cycle joint
// motion commands using a time-synchronized symmetric trapezoidal profile.
//
// Determinism: sampling happens strictly at t = k*Period from a fixed
// simulation clock; identical inputs (including float bit patterns) always
// produce identical samples. The result is continuous in the parameters:
// small changes in goal/limits produce small changes in sample values; the
// integer cycle count changes only at timing boundaries.
package trajectory

import (
	"fmt"
	"math"

	"github.com/h3yow/robot-motion-controller-go/internal/motion/classify"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/config"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/kinematics"
)

// Point is one control-cycle sample of the commanded trajectory.
type Point struct {
	T        float64   // time from trajectory start (s)
	Position []float64 // commanded joint position (rad)
	Velocity []float64 // commanded joint velocity (rad/s)
	Accel    []float64 // commanded joint acceleration (rad/s^2)
}

// Trajectory is a sampled, time-synchronized multi-axis trajectory.
type Trajectory struct {
	Period float64 // control period (s)
	Points []Point // Points[0] is the start; one extra sample per cycle
	Axes   int
}

// Duration returns the planned duration in seconds.
func (t *Trajectory) Duration() float64 {
	if t == nil || len(t.Points) == 0 {
		return 0
	}
	return t.Points[len(t.Points)-1].T
}

// Planner validates goals against configuration and builds trajectories.
type Planner struct {
	cfg *config.Config
}

// NewPlanner creates a Planner. The configuration must already have passed
// config.Validate.
func NewPlanner(cfg *config.Config) *Planner { return &Planner{cfg: cfg} }

func (p *Planner) reject(reason classify.Reason, format string, args ...any) error {
	return classify.New(reason, fmt.Sprintf(format, args...))
}

func sameLen(name string, vecs ...[]float64) error {
	for i, v := range vecs {
		if len(v) != len(vecs[0]) {
			return classify.New(classify.ReasonContradictoryTarget,
				fmt.Sprintf("%s: vector %d has length %d, want %d", name, i, len(v), len(vecs[0])))
		}
	}
	return nil
}

// checkState validates a joint vector against finiteness and travel limits.
func (p *Planner) checkState(tag string, q []float64) error {
	if len(q) != p.cfg.AxisCount() {
		return p.reject(classify.ReasonContradictoryTarget,
			"%s has %d components, want %d axes", tag, len(q), p.cfg.AxisCount())
	}
	for i, a := range p.cfg.Axes {
		if !kinematics.IsFinite(q[i]) {
			return p.reject(classify.ReasonTargetNonFinite,
				"%s axis[%d] (%s) is NaN/Inf: %v", tag, i, a.Name, q[i])
		}
		if q[i] < a.MinPosition || q[i] > a.MaxPosition {
			return p.reject(classify.ReasonOutOfTravel,
				"%s axis[%d] (%s) = %v outside travel [%v,%v]",
				tag, i, a.Name, q[i], a.MinPosition, a.MaxPosition)
		}
	}
	return nil
}

// checkLimits validates requested per-axis velocity/accel envelopes against
// finiteness, positivity and configured hard limits, plus the torque needed
// at peak acceleration.
func (p *Planner) checkLimits(maxVel, maxAccel []float64) error {
	if err := sameLen("motion limits", maxVel, maxAccel, make([]float64, p.cfg.AxisCount())); err != nil {
		return err
	}
	for i, a := range p.cfg.Axes {
		if !kinematics.IsFinite(maxVel[i], maxAccel[i]) || maxVel[i] <= 0 || maxAccel[i] <= 0 {
			return p.reject(classify.ReasonContradictoryTarget,
				"axis[%d] (%s) limits must be finite and > 0: vel=%v accel=%v",
				i, a.Name, maxVel[i], maxAccel[i])
		}
		if maxVel[i] > a.MaxVelocity {
			return p.reject(classify.ReasonVelocityLimit,
				"axis[%d] (%s) requested max velocity %v > limit %v",
				i, a.Name, maxVel[i], a.MaxVelocity)
		}
		if maxAccel[i] > a.MaxAccel {
			return p.reject(classify.ReasonAccelerationLimit,
				"axis[%d] (%s) requested max accel %v > limit %v",
				i, a.Name, maxAccel[i], a.MaxAccel)
		}
		// Worst-case torque magnitude at peak acceleration, including the
		// constant gravity bias.
		need := a.Inertia*maxAccel[i] + math.Abs(a.GravityTorque)
		if need > a.MaxTorque {
			return p.reject(classify.ReasonTorqueLimit,
				"axis[%d] (%s) peak required torque %v N*m > limit %v (inertia %v * accel %v + |gravity %v|)",
				i, a.Name, need, a.MaxTorque, a.Inertia, maxAccel[i], a.GravityTorque)
		}
	}
	return nil
}

// axisProfile is a symmetric trapezoidal scalar profile stretched to fill
// exactly Tn seconds while respecting vCap and aCap.
type axisProfile struct {
	start, d, r, c, v, acc float64
}

func (p *Planner) planAxis(i int, start, goal, vCap, aCap, Tn float64) (axisProfile, error) {
	delta := goal - start
	d := math.Abs(delta)
	if d == 0 {
		return axisProfile{start: start}, nil // hold
	}
	// Shortest-time parameters at the requested limits.
	var rMin float64
	if vTri := math.Sqrt(d * aCap); vTri <= vCap {
		rMin = vTri / aCap // triangular: peak never reaches vCap
	} else {
		rMin = vCap / aCap // trapezoidal plateau at vCap
	}
	if 2*rMin > Tn+1e-12 {
		return axisProfile{}, p.reject(classify.ReasonContradictoryTarget,
			"axis[%d] internal: minimal ramp %.9gs exceeds synchronized window %.9gs", i, rMin, Tn)
	}
	c := Tn - 2*rMin // cruise duration (>= 0): the slowest axis sets Tn
	v := d / (rMin + c)
	acc := v / rMin
	if v > vCap+1e-9*math.Max(1, vCap) || acc > aCap+1e-9*math.Max(1, aCap) {
		return axisProfile{}, p.reject(classify.ReasonContradictoryTarget,
			"axis[%d] internal: stretched profile exceeds limits v=%v/%v a=%v/%v", i, v, vCap, acc, aCap)
	}
	return axisProfile{start: start, d: delta, r: rMin, c: c, v: v, acc: acc}, nil
}

// sample evaluates the scalar profile at time t (0 <= t <= Tn).
func (pr axisProfile) sample(t, Tn float64) (pos, vel, acc float64) {
	if pr.d == 0 {
		return pr.start, 0, 0
	}
	sign := 1.0
	if pr.d < 0 {
		sign = -1
	}
	d := math.Abs(pr.d)
	a := pr.acc
	switch {
	case t <= pr.r:
		pos = 0.5 * a * t * t
		vel = a * t
		acc = a
	case t <= pr.r+pr.c:
		pos = 0.5*a*pr.r*pr.r + pr.v*(t-pr.r)
		vel = pr.v
		acc = 0
	default:
		pos = d - 0.5*a*(Tn-t)*(Tn-t)
		vel = a * (Tn - t)
		acc = -a
	}
	return pr.start + sign*pos, sign*vel, sign*acc
}

// segment builds the synchronized samples for one start->goal move.
func (p *Planner) segment(start, goal, maxVel, maxAccel []float64, tOffset float64) ([]Point, error) {
	if err := p.checkState("start", start); err != nil {
		return nil, err
	}
	if err := p.checkState("goal", goal); err != nil {
		return nil, err
	}
	if err := p.checkLimits(maxVel, maxAccel); err != nil {
		return nil, err
	}
	n := p.cfg.AxisCount()
	// Minimal duration per axis.
	durations := make([]float64, n)
	for i := 0; i < n; i++ {
		d := math.Abs(goal[i] - start[i])
		if d == 0 {
			continue
		}
		vTri := math.Sqrt(d * maxAccel[i])
		if vTri <= maxVel[i] {
			durations[i] = 2 * vTri / maxAccel[i]
		} else {
			r := maxVel[i] / maxAccel[i]
			cruise := (d - maxAccel[i]*r*r) / maxVel[i]
			durations[i] = 2*r + cruise
		}
	}
	T := 0.0
	for _, dt := range durations {
		if dt > T {
			T = dt
		}
	}
	cycles := int(math.Ceil(T/p.cfg.ControlPeriod - 1e-9))
	if cycles < 1 {
		cycles = 1
	}
	Tn := float64(cycles) * p.cfg.ControlPeriod
	profs := make([]axisProfile, n)
	for i := 0; i < n; i++ {
		pr, err := p.planAxis(i, start[i], goal[i], maxVel[i], maxAccel[i], Tn)
		if err != nil {
			return nil, err
		}
		profs[i] = pr
	}
	points := make([]Point, cycles+1)
	for k := 0; k <= cycles; k++ {
		t := float64(k) * p.cfg.ControlPeriod
		pt := Point{T: tOffset + t, Position: make([]float64, n), Velocity: make([]float64, n), Accel: make([]float64, n)}
		for i := 0; i < n; i++ {
			pos, vel, acc := profs[i].sample(t, Tn)
			if k == cycles {
				pos, vel, acc = goal[i], 0, 0 // exact stop; no residual float drift
			}
			pt.Position[i], pt.Velocity[i], pt.Accel[i] = pos, vel, acc
		}
		points[k] = pt
	}
	return points, nil
}

// PointToPoint plans from start to goal. Any limit violation, non-finite or
// contradictory input is rejected with a distinguishable reason; no default
// value is substituted. maxVel/maxAccel are per-axis requested envelopes and
// must not exceed configured hard limits.
func (p *Planner) PointToPoint(start, goal, maxVel, maxAccel []float64) (*Trajectory, error) {
	pts, err := p.segment(start, goal, maxVel, maxAccel, 0)
	if err != nil {
		return nil, err
	}
	return &Trajectory{Period: p.cfg.ControlPeriod, Points: pts, Axes: p.cfg.AxisCount()}, nil
}

// Path plans motion through ordered waypoints with a full stop at each.
func (p *Planner) Path(start []float64, waypoints [][]float64, maxVel, maxAccel []float64) (*Trajectory, error) {
	if p.cfg == nil {
		return nil, classify.New(classify.ReasonInvalidCommand, "planner has no configuration")
	}
	if len(waypoints) == 0 {
		return nil, classify.New(classify.ReasonContradictoryTarget, "path requires at least one waypoint")
	}
	traj := &Trajectory{Period: p.cfg.ControlPeriod, Axes: p.cfg.AxisCount()}
	cur := append([]float64(nil), start...)
	tOffset := 0.0
	for w, wp := range waypoints {
		pts, err := p.segment(cur, wp, maxVel, maxAccel, tOffset)
		if err != nil {
			r, _ := classify.AsReject(err)
			return nil, classify.New(r.Reason, fmt.Sprintf("waypoint[%d]: %s", w, r.Msg))
		}
		if w == 0 {
			traj.Points = append(traj.Points, pts[0])
		}
		traj.Points = append(traj.Points, pts[1:]...) // skip duplicated junction
		tOffset = traj.Points[len(traj.Points)-1].T
		cur = append(cur[:0], wp...)
	}
	return traj, nil
}
