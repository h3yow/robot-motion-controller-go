// Package trajectory generates per-control-period motion samples
// (position/velocity/acceleration/feed-forward torque) using synchronized
// trapezoidal joint profiles. Infeasible requests are rejected with a
// classified reason; no constraint is silently relaxed or default-filled.
//
// Per-axis construction (supports non-zero start velocity, which happens on
// preemption): an initial constant-deceleration phase brakes to rest, then
// a rest-to-rest trapezoid (or triangle) reaches the goal. All axes share
// one fixed-period sample grid: the axis needing the longest motion sets the
// duration; other axes stretch by lowering their cruise speed (peak
// acceleration is never increased). Determinism: all outputs derive only
// from the request and the fixed period — no clocks, maps or goroutines.
package trajectory

import (
	"fmt"
	"math"
	"time"

	"github.com/h3yow/robot-motion-controller-go/internal/mcerr"
	"github.com/h3yow/robot-motion-controller-go/internal/numeric"
)

// AxisLimits are the per-joint physical limits used by the planner.
type AxisLimits struct {
	Name                                   string
	MinPosition, MaxPosition               float64 // rad
	MaxVelocity, MaxAccel, MaxTorque       float64
	Inertia, GravityTorque, FrictionTorque float64
}

// Point is one sampled trajectory point. All slices have axis cardinality.
type Point struct {
	Time     float64   // seconds from trajectory start
	Position []float64 // rad
	Velocity []float64 // rad/s
	Accel    []float64 // rad/s^2
	Torque   []float64 // N·m feed-forward
}

// Trajectory is an immutable, fixed-period sequence of points.
type Trajectory struct {
	period time.Duration
	points []Point
}

// DT returns the fixed sampling period.
func (tr *Trajectory) DT() time.Duration { return tr.period }

// Len returns the number of sampled points (including the start point).
func (tr *Trajectory) Len() int { return len(tr.points) }

// At returns point i (0-based, i must be in [0, Len)).
func (tr *Trajectory) At(i int) Point { return tr.points[i] }

// Final returns the last point.
func (tr *Trajectory) Final() Point { return tr.points[len(tr.points)-1] }

// seg is one constant-acceleration phase of an axis profile.
type seg struct {
	t0, t1 float64 // phase interval (seconds)
	a      float64 // signed constant acceleration
	xs, vs float64 // state at t0
}

// axisPlan is the per-axis continuous profile.
type axisPlan struct {
	segs       []seg
	T          float64 // total profile duration (seconds)
	goal       float64 // value returned by evaluate when there are no phases
	start      float64 // start position at t=0
	reqGoal    float64 // requested goal position
	accelBound float64 // max |acceleration| for this axis
}

// Request is a single start->goal joint-space move.
type Request struct {
	StartPosition []float64
	StartVelocity []float64
	GoalPosition  []float64
	Limits        []AxisLimits
	Period        time.Duration
}

func validateCardinality(name string, startPos, startVel, goal []float64, limits []AxisLimits) (int, error) {
	n := len(limits)
	if n == 0 {
		return 0, mcerr.New(mcerr.CodeTargetContradictory, "planner requires at least one axis limit")
	}
	if len(startPos) != n || len(goal) != n {
		return 0, mcerr.New(mcerr.CodeTargetContradictory,
			fmt.Sprintf("%s: start/goal cardinality (%d/%d) does not match limits %d",
				name, len(startPos), len(goal), n))
	}
	if len(startVel) != n {
		return 0, mcerr.New(mcerr.CodeTargetContradictory,
			fmt.Sprintf("%s: start velocity cardinality %d does not match limits %d", name, len(startVel), n))
	}
	return n, nil
}

// Plan builds a synchronized multi-axis trapezoidal trajectory.
func Plan(req *Request) (*Trajectory, error) {
	if req == nil {
		return nil, mcerr.New(mcerr.CodeTargetContradictory, "nil plan request")
	}
	if req.Period <= 0 {
		return nil, mcerr.New(mcerr.CodeConfigInvalid, "period must be strictly positive")
	}
	n, err := validateCardinality("plan", req.StartPosition, req.StartVelocity, req.GoalPosition, req.Limits)
	if err != nil {
		return nil, err
	}
	if !numeric.AllFinite(req.StartPosition) || !numeric.AllFinite(req.StartVelocity) || !numeric.AllFinite(req.GoalPosition) {
		return nil, mcerr.New(mcerr.CodeTargetNonFinite, "start and goal must all be finite")
	}
	dt := req.Period.Seconds()
	plans := make([]axisPlan, n)
	tMinMax := 0.0
	for i := 0; i < n; i++ {
		p, tmin, perr := buildMinimal(req.StartPosition[i], req.StartVelocity[i], req.GoalPosition[i], req.Limits[i])
		if perr != nil {
			return nil, perr
		}
		plans[i] = p
		if tmin > tMinMax {
			tMinMax = tmin
		}
	}
	// Common sample grid: one point at t=0 and the goal at t=N*dt.
	N := int(math.Ceil(tMinMax/dt - 1e-12))
	if N < 1 {
		N = 1
	}
	T := float64(N) * dt
	for i := range plans {
		if err := plans[i].stretch(T); err != nil {
			return nil, err
		}
		if err := plans[i].checkTorque(req.Limits[i]); err != nil {
			return nil, err
		}
	}
	bounds := make([]map[int]float64, n)
	for i := 0; i < n; i++ {
		bounds[i] = map[int]float64{0: req.StartPosition[i], N: req.GoalPosition[i]}
	}
	return sampleAll(plans, req.Limits, req.Period, dt, N, bounds), nil
}

// buildMinimal validates bounds and builds the minimal-time continuous
// profile for one axis. It returns the profile and its minimal duration.
func buildMinimal(x0, v0, goal float64, lim AxisLimits) (axisPlan, float64, error) {
	if !(lim.MinPosition < lim.MaxPosition) || !(lim.MaxVelocity > 0) || !(lim.MaxAccel > 0) || !(lim.MaxTorque > 0) {
		return axisPlan{}, 0, mcerr.New(mcerr.CodeConfigInvalid,
			"axis "+lim.Name+" has non-positive or inverted limit parameters")
	}
	if x0 < lim.MinPosition || x0 > lim.MaxPosition {
		return axisPlan{}, 0, mcerr.New(mcerr.CodeInfeasibleStart,
			fmt.Sprintf("axis %s: start %.6f rad outside travel [%.6f, %.6f]", lim.Name, x0, lim.MinPosition, lim.MaxPosition))
	}
	if math.Abs(v0) > lim.MaxVelocity+1e-12 {
		return axisPlan{}, 0, mcerr.New(mcerr.CodeInfeasibleStart,
			fmt.Sprintf("axis %s: start speed %.6f exceeds max velocity %.6f", lim.Name, v0, lim.MaxVelocity))
	}
	if !(lim.Inertia > 0) || lim.GravityTorque < 0 || lim.FrictionTorque < 0 {
		return axisPlan{}, 0, mcerr.New(mcerr.CodeConfigInvalid,
			"axis "+lim.Name+": inertia must be > 0 and gravity/friction >= 0")
	}
	if goal < lim.MinPosition || goal > lim.MaxPosition {
		return axisPlan{}, 0, mcerr.New(mcerr.CodeJointLimitExceeded,
			fmt.Sprintf("axis %s: goal %.6f rad outside travel [%.6f, %.6f]", lim.Name, goal, lim.MinPosition, lim.MaxPosition))
	}
	A := lim.MaxAccel
	p := axisPlan{goal: x0, start: x0, reqGoal: goal, accelBound: A}
	xb := x0
	if v0 != 0 {
		// Phase 1: brake to rest at maximum deceleration.
		s0 := math.Copysign(1, v0)
		t0 := math.Abs(v0) / A
		aBrake := -s0 * A
		p.segs = append(p.segs, seg{t0: 0, t1: t0, a: aBrake, xs: x0, vs: v0})
		xb = x0 + v0*math.Abs(v0)/(2*A)
		if xb < lim.MinPosition || xb > lim.MaxPosition {
			return axisPlan{}, 0, mcerr.New(mcerr.CodeJointLimitExceeded,
				fmt.Sprintf("axis %s: safe brake from %.6f rad/s reaches %.6f rad, outside travel",
					lim.Name, v0, xb))
		}
	}
	// A no-motion axis with zero velocity gets no phases at all; evaluate
	// returns the start/goal and the first moving leg still covers [0,T].
	D := goal - xb
	tmin := p.duration()
	if math.Abs(D) > 0 {
		tmin += restRestMinTime(math.Abs(D), lim.MaxVelocity, A)
	}
	// Extrema on the profile are x0, xb and goal (rest-to-rest leg is
	// monotonic); all are already bounds-checked.
	return p, tmin, nil
}

// restRestMinTime returns the minimum rest-to-rest time for distance D.
func restRestMinTime(D, V, A float64) float64 {
	if D >= V*V/A {
		return V/A + D/V // trap
	}
	return 2 * math.Sqrt(D/A) // triangle
}

// duration sums current phase durations.
func (p *axisPlan) duration() float64 {
	if len(p.segs) == 0 {
		return 0
	}
	return p.segs[len(p.segs)-1].t1
}

// stretch extends the profile to total duration T by lowering cruise speed;
// the initial brake phase keeps its duration and acceleration.
func (p *axisPlan) stretch(T float64) error {
	tBrake := p.duration()
	if T < tBrake-1e-12 {
		return mcerr.New(mcerr.CodeAccelerationLimit,
			fmt.Sprintf("profile needs %.6f s to brake but %.6f s was allocated", tBrake, T))
	}
	// State at end of brake (rest point): computed from the brake phase,
	// or simply the stored start when the axis had zero initial velocity.
	xb := p.start
	if tBrake > 0 {
		xb, _ = p.evaluate(tBrake)
	}
	D := p.reqGoal - xb
	avail := T - tBrake
	if math.Abs(D) <= 1e-13 {
		// Do not append a [0,T] idle segment on a fully static axis: it
		// would shadow any later leg segments. If a real brake phase ended
		// at rest before T, append only the tail hold after it.
		if avail > 1e-12 && tBrake > 0 {
			p.segs = append(p.segs, seg{t0: tBrake, t1: T, a: 0, xs: xb, vs: 0})
		}
		p.T = T
		return nil
	}
	p.goal = p.reqGoal
	s := math.Copysign(1, D)
	d := math.Abs(D)
	A := p.accelBound
	u := peakSpeed(d, A, avail)
	t1 := u / A
	segAcc := seg{t0: tBrake, t1: tBrake + t1, a: s * A, xs: xb, vs: 0}
	p.segs = append(p.segs, segAcc)
	tCruise := d/u - u/A
	if tCruise < -1e-10 {
		return mcerr.New(mcerr.CodeAccelerationLimit, "synchronized profile construction failed (negative cruise)")
	}
	if tCruise < 0 {
		tCruise = 0
	}
	if tCruise > 0 {
		xAt := xb + s*0.5*u*t1
		p.segs = append(p.segs, seg{t0: tBrake + t1, t1: tBrake + t1 + tCruise, a: 0, xs: xAt, vs: s * u})
		xAt += s * u * tCruise
		p.segs = append(p.segs, seg{t0: tBrake + t1 + tCruise, t1: T, a: -s * A, xs: xAt, vs: s * u})
	} else {
		xAt := xb + s*0.5*u*t1
		p.segs = append(p.segs, seg{t0: tBrake + t1, t1: T, a: -s * A, xs: xAt, vs: s * u})
	}
	p.T = T
	return nil
}

// peakSpeed solves the rest-to-rest cruise speed that realizes distance d in
// exactly time tau, using acceleration bound A. The slow root is chosen
// (lower speed) and it never exceeds sqrt(d*A) (triangle peak).
func peakSpeed(d, A, tau float64) float64 {
	tri := 2 * math.Sqrt(d/A)
	if tau <= tri+1e-11 {
		return math.Sqrt(d * A)
	}
	disc := tau*tau - 4*d/A
	if disc < 0 {
		disc = 0
	}
	return A * (tau - math.Sqrt(disc)) / 2
}

// startX records the constant position of a fully static axis.
func (p *axisPlan) startX() float64 {
	if len(p.segs) == 0 {
		return p.goal // set by callers to the current start when no motion
	}
	return p.segs[0].xs
}

// evaluate returns (position, velocity) at time t.
func (p *axisPlan) evaluate(t float64) (float64, float64) {
	if len(p.segs) == 0 {
		return p.goal, 0
	}
	// Choose the earliest segment containing t (boundaries belong to the
	// segment that starts there).
	for _, s := range p.segs {
		if t >= s.t0-1e-12 && t <= s.t1+1e-12 {
			dt := t - s.t0
			if dt < 0 {
				dt = 0
			}
			return s.xs + s.vs*dt + 0.5*s.a*dt*dt, s.vs + s.a*dt
		}
	}
	if t < p.segs[0].t0 {
		return p.segs[0].xs, p.segs[0].vs
	}
	last := p.segs[len(p.segs)-1]
	return last.xs + last.vs*(last.t1-last.t0), 0
}

// checkTorque verifies the worst-case feed-forward torque over the profile.
func (p *axisPlan) checkTorque(lim AxisLimits) error {
	apeak := 0.0
	for _, s := range p.segs {
		if a := math.Abs(s.a); a > apeak {
			apeak = a
		}
	}
	need := lim.Inertia*apeak + lim.GravityTorque + lim.FrictionTorque
	if need > lim.MaxTorque+1e-9 {
		return mcerr.New(mcerr.CodeTorqueLimit,
			fmt.Sprintf("axis %s: required torque %.4f N·m (J*a=%.4f + gravity %.4f + friction %.4f) exceeds max %.4f",
				lim.Name, need, lim.Inertia*apeak, lim.GravityTorque, lim.FrictionTorque, lim.MaxTorque))
	}
	return nil
}

// sampleAll evaluates every axis on the common grid and computes
// feed-forward torque tau = J*a + G*sin(q) + F*sign(v). snapTimes (in
// sample counts) forces exact boundary/goal states.
func sampleAll(plans []axisPlan, limits []AxisLimits, period time.Duration, dt float64, N int, boundaries []map[int]float64) *Trajectory {
	n := len(plans)
	points := make([]Point, 0, N+1)
	for k := 0; k <= N; k++ {
		t := float64(k) * dt
		pt := Point{
			Time:     t,
			Position: make([]float64, n),
			Velocity: make([]float64, n),
			Accel:    make([]float64, n),
			Torque:   make([]float64, n),
		}
		for i := 0; i < n; i++ {
			x, v := plans[i].evaluate(t)
			a := accelAt(&plans[i], t)
			// Snap to exact joint values at waypoint boundaries (k=0 and
			// every segment end) so the grid carries the exact targets.
			if bx, ok := boundaries[i][k]; ok {
				x = bx
				v = 0
				a = 0
			}
			pt.Position[i] = x
			pt.Velocity[i] = v
			pt.Accel[i] = a
			fr := 0.0
			if v > 0 {
				fr = limits[i].FrictionTorque
			} else if v < 0 {
				fr = -limits[i].FrictionTorque
			}
			pt.Torque[i] = limits[i].Inertia*a + limits[i].GravityTorque*math.Sin(x) + fr
		}
		points = append(points, pt)
	}
	return &Trajectory{period: period, points: points}
}

// accelAt returns the active acceleration of the phase covering t.
func accelAt(p *axisPlan, t float64) float64 {
	for _, s := range p.segs {
		if t >= s.t0-1e-12 && t <= s.t1+1e-12 {
			return s.a
		}
	}
	return 0
}

// PathRequest is a multi-waypoint joint-space path (stop at each point).
type PathRequest struct {
	StartPosition []float64
	StartVelocity []float64
	Waypoints     [][]float64
	Limits        []AxisLimits
	Period        time.Duration
}

// PlanPath builds a trajectory visiting every waypoint in order, stopping at
// each one. Empty duplicate consecutive waypoints are skipped.
//
// Consumer caveat: the intermediate rest points are INTERNAL samples of the
// returned trajectory; there is no per-waypoint completion callback. A
// controller consuming the whole trajectory signals completion only at the
// final sample, after every intermediate point has already been crossed. An
// executor that needs per-waypoint accounting, continuation after
// pause/queue/append from the true position, and a guarantee that reached
// points are never revisited must therefore plan ONE leg at a time
// (measured state -> next waypoint), re-arming when that leg completes -
// which is what the motion service does. Planning the whole remaining tail
// as one trajectory and replanning it after the single completion signal
// would drive the axes back through already-visited middle points.
func PlanPath(req *PathRequest) (*Trajectory, error) {
	if req == nil {
		return nil, mcerr.New(mcerr.CodeTargetContradictory, "nil path request")
	}
	if req.Period <= 0 {
		return nil, mcerr.New(mcerr.CodeConfigInvalid, "period must be strictly positive")
	}
	if len(req.Waypoints) == 0 {
		return nil, mcerr.New(mcerr.CodeTargetContradictory, "path requires at least one waypoint")
	}
	n := len(req.Limits)
	if len(req.StartPosition) != n || len(req.StartVelocity) != n {
		return nil, mcerr.New(mcerr.CodeTargetContradictory, "path start cardinality does not match limits")
	}
	for j, wp := range req.Waypoints {
		if len(wp) != n {
			return nil, mcerr.New(mcerr.CodeTargetContradictory,
				fmt.Sprintf("waypoint %d cardinality %d does not match axes %d", j, len(wp), n))
		}
	}
	if !numeric.AllFinite(req.StartPosition) || !numeric.AllFinite(req.StartVelocity) {
		return nil, mcerr.New(mcerr.CodeTargetNonFinite, "path start must be finite")
	}
	for j, wp := range req.Waypoints {
		if !numeric.AllFinite(wp) {
			return nil, mcerr.New(mcerr.CodeTargetNonFinite,
				fmt.Sprintf("waypoint %d contains a non-finite value", j))
		}
	}
	dt := req.Period.Seconds()
	plans := make([]axisPlan, n)
	bounds := make([]map[int]float64, n)
	for i := range plans {
		plans[i].goal = req.StartPosition[i]
		plans[i].accelBound = req.Limits[i].MaxAccel
		bounds[i] = map[int]float64{0: req.StartPosition[i]}
	}
	// First leg carries the initial velocity (brake-first); later legs
	// start from rest. Each leg is independently synchronized on the grid.
	curPos := append([]float64(nil), req.StartPosition...)
	curVel := append([]float64(nil), req.StartVelocity...)
	kOffset := 0
	for _, wp := range req.Waypoints {
		if identical(curPos, wp) {
			continue
		}
		legPlans := make([]axisPlan, n)
		legMax := 0.0
		for i := 0; i < n; i++ {
			lp, tmin, err := buildMinimal(curPos[i], curVel[i], wp[i], req.Limits[i])
			if err != nil {
				return nil, err
			}
			legPlans[i] = lp
			if tmin > legMax {
				legMax = tmin
			}
		}
		Nj := int(math.Ceil(legMax/dt - 1e-12))
		if Nj < 1 {
			Nj = 1
		}
		Tj := float64(Nj) * dt
		legStart := float64(kOffset) * dt
		for i := range legPlans {
			if err := legPlans[i].stretch(Tj); err != nil {
				return nil, err
			}
			if err := legPlans[i].checkTorque(req.Limits[i]); err != nil {
				return nil, err
			}
			if len(legPlans[i].segs) == 0 {
				// Static for this leg: hold the value over this leg window.
				plans[i].segs = append(plans[i].segs, seg{
					t0: legStart, t1: legStart + Tj, a: 0,
					xs: curPos[i], vs: 0,
				})
			} else {
				for _, sg := range legPlans[i].segs {
					plans[i].segs = append(plans[i].segs, seg{
						t0: sg.t0 + legStart,
						t1: sg.t1 + legStart,
						a:  sg.a, xs: sg.xs, vs: sg.vs,
					})
				}
			}
			plans[i].goal = wp[i]
		}
		for i := 0; i < n; i++ {
			bounds[i][kOffset+Nj] = wp[i]
		}
		kOffset += Nj
		curPos = append([]float64(nil), wp...)
		curVel = make([]float64, n)
	}
	N := kOffset
	for i := range plans {
		plans[i].T = float64(N) * dt
	}
	return sampleAll(plans, req.Limits, req.Period, dt, N, bounds), nil
}

func identical(a, b []float64) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}
