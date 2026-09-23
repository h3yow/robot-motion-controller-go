// Package engine ties together planner, simulated plant, controller and
// arbiter behind a single mutex, providing deterministic stepping and
// consistent state snapshots under concurrent command entry points.
package engine

import (
	"github.com/h3yow/robot-motion-controller-go/internal/motion/arbiter"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/classify"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/config"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/controller"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/plant"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/trajectory"
)

// State is the engine lifecycle state.
type State string

const (
	StateIdle      State = "IDLE"
	StateRunning   State = "RUNNING"
	StateStopping  State = "STOPPING"
	StateFault     State = "FAULT"
	StateFatal     State = "FATAL"
)

// AxisSnapshot is one axis' externally visible state.
type AxisSnapshot struct {
	PositionCommand float64
	VelocityCommand float64
	PositionActual  float64
	VelocityActual  float64
	TorqueCommand   float64
	Saturated       bool
}

// Snapshot is a consistent (single-lock) view of the whole engine.
type Snapshot struct {
	State        State
	ActiveID     arbiter.ID
	Cycle        int64
	Fault        *controller.Fault
	LastResult   classify.Reason
	Axes         []AxisSnapshot
}

// Result reports how a submitted command ultimately resolved.
type Result struct {
	ID       arbiter.ID
	Done     bool
	Rejected classify.Reason
	Fault    *controller.Fault
}

// Engine is the motion execution engine.
type Engine struct {
	cfg *config.Config
}

// New constructs an Engine from validated configuration.
func New(cfg *config.Config) *Engine { return &Engine{cfg: cfg} }

// Submit enqueues/arbitrates one command. It returns immediately with the
// arbitration decision; completion is observed via Snapshot/Result.
func (e *Engine) Submit(cmd arbiter.Command) (arbiter.Decision, error) {
	return arbiter.Decision{}, nil
}

// Step advances exactly one fixed control period.
func (e *Engine) Step() Result { return Result{} }

// Run executes cycles until the active motion completes, faults, or is
// cancelled. Deterministic: no wall clock is used.
func (e *Engine) Run() Result { return Result{} }

// RunTicked runs in a background goroutine, ticking every control period
// until Shutdown. All access remains serialized through the engine mutex.
func (e *Engine) RunTicked() {}

// Shutdown stops the background ticker.
func (e *Engine) Shutdown() {}

// Snapshot returns a consistent copy of engine state.
func (e *Engine) Snapshot() Snapshot { return Snapshot{} }

// Recover requests recovery from a recoverable fault after preconditions
// (feedback restored, tracking error within tolerance, stopped) are met.
func (e *Engine) Recover() error { return nil }

// Reconfigure replaces configuration after a FATAL fault (hard reset);
// invalid configurations are rejected and state is unchanged.
func (e *Engine) Reconfigure(cfg *config.Config) error { return nil }

// The following keep imports used in the skeleton.
var (
	_ = plant.New
	_ = trajectory.NewPlanner
)
