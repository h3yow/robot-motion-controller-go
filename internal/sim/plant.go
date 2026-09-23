// Package sim provides a deterministic local actuator/encoder plant:
// rigid-body integrator with Coulomb friction, gravity load, hard joint
// stops, external disturbance and actuator torque-loss injection. No real
// hardware, network or credentials are involved.
//
// Model (per axis i, SI units):
//
//	J_i * dv_i/dt = tau_scale_i * tau_cmd_i - F_i*sign(v_i) - G_i*sin(q_i) + tau_load_i
//	dq_i/dt      = v_i
//
// tau_cmd is clamped to +/-MaxTorque (the controller detects saturation),
// position is clamped to the hard travel limits and velocity is zeroed on
// impact with a stop. Every call advances exactly one fixed control period,
// so identical command sequences reproduce identical states.
package sim

import (
	"fmt"
	"math"

	"github.com/h3yow/robot-motion-controller-go/internal/config"
	"github.com/h3yow/robot-motion-controller-go/internal/mcerr"
	"github.com/h3yow/robot-motion-controller-go/internal/numeric"
)

// Plant is the simulated mechanism. It is safe for single-goroutine use
// (the motion service serializes access).
type Plant struct {
	axes      []config.AxisConfig
	dt        float64
	q, v      []float64
	load      []float64
	scale     []float64
	tickCount int64
}

// NewPlant creates a plant from validated configuration (home positions are
// the initial joint configuration).
func NewPlant(cfg *config.Config) *Plant {
	n := len(cfg.Axes)
	p := &Plant{
		axes:  append([]config.AxisConfig(nil), cfg.Axes...),
		dt:    cfg.Period.Seconds(),
		q:     make([]float64, n),
		v:     make([]float64, n),
		load:  make([]float64, n),
		scale: make([]float64, n),
	}
	for i := range p.axes {
		p.q[i] = cfg.Axes[i].HomePosition
		p.scale[i] = 1
	}
	return p
}

// Reset places every axis at the given positions with zero velocity and
// clears injections.
func (p *Plant) Reset(positions []float64) {
	if len(positions) != len(p.axes) {
		panic(fmt.Sprintf("sim.Reset: expected %d positions, got %d", len(p.axes), len(positions)))
	}
	for i := range p.axes {
		p.q[i] = positions[i]
		p.v[i] = 0
		p.load[i] = 0
		p.scale[i] = 1
	}
	p.tickCount = 0
}

// SetLoadTorque injects an external disturbance torque per axis (N·m).
func (p *Plant) SetLoadTorque(tau []float64) {
	for i := 0; i < len(p.axes) && i < len(tau); i++ {
		p.load[i] = tau[i]
	}
}

// SetTorqueScale multiplies the applied actuator torque per axis
// (1.0 = healthy, 0 = total torque loss).
func (p *Plant) SetTorqueScale(scale []float64) {
	for i := 0; i < len(p.axes) && i < len(scale); i++ {
		p.scale[i] = scale[i]
	}
}

// Step advances one fixed control period under the commanded torques and
// returns encoder positions/velocities. A non-finite command is rejected
// and the plant state is left untouched (safety: never act on NaN).
func (p *Plant) Step(commandTorque []float64) (positions, velocities []float64, err error) {
	if len(commandTorque) != len(p.axes) {
		return nil, nil, mcerr.New(mcerr.CodeCommandNonFinite,
			fmt.Sprintf("plant expects %d torque commands, got %d", len(p.axes), len(commandTorque)))
	}
	if !numeric.AllFinite(commandTorque) {
		return nil, nil, mcerr.New(mcerr.CodeCommandNonFinite, "commanded torque contains NaN/Inf")
	}
	for i, a := range p.axes {
		tauCmd := numeric.Clamp(commandTorque[i], -a.MaxTorque, a.MaxTorque)
		tau := p.scale[i]*tauCmd - a.GravityTorque*math.Sin(p.q[i]) + p.load[i]
		// Coulomb friction opposes motion; at standstill it cancels only the
		// net driving force (|tau| <= F means static friction holds).
		drive := tau
		if p.v[i] != 0 {
			tau -= a.FrictionTorque * sign(p.v[i])
		} else if abs(drive) <= a.FrictionTorque {
			tau = 0
		} else {
			tau -= a.FrictionTorque * sign(drive)
		}
		p.v[i] += tau / a.Inertia * p.dt
		p.q[i] += p.v[i] * p.dt
		// Hard joint stops.
		if p.q[i] < a.MinPosition {
			p.q[i] = a.MinPosition
			if p.v[i] < 0 {
				p.v[i] = 0
			}
		} else if p.q[i] > a.MaxPosition {
			p.q[i] = a.MaxPosition
			if p.v[i] > 0 {
				p.v[i] = 0
			}
		}
	}
	p.tickCount++
	return append([]float64(nil), p.q...), append([]float64(nil), p.v...), nil
}

// State returns the current encoder positions and velocities.
func (p *Plant) State() (positions, velocities []float64) {
	return append([]float64(nil), p.q...), append([]float64(nil), p.v...)
}

// Tick returns the number of completed integration steps.
func (p *Plant) Tick() int64 { return p.tickCount }

func sign(x float64) float64 {
	if x > 0 {
		return 1
	}
	if x < 0 {
		return -1
	}
	return 0
}

func abs(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}
