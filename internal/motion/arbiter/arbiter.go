// Package arbiter arbitrates concurrent, possibly out-of-order motion
// commands using per-source sequence numbers and fixed priorities.
package arbiter

import "github.com/h3yow/robot-motion-controller-go/internal/motion/classify"

// Op is the command operation type.
type Op string

const (
	OpMoveTo Op = "MOVE_TO"
	OpPath   Op = "PATH"
	OpCancel Op = "CANCEL"
	OpReset  Op = "RESET"
)

// Command is an arbitrated command envelope. Sequence numbers are scoped to
// Source; strictly increasing per source. Higher Priority wins.
type Command struct {
	Source    string
	Sequence  int64
	Priority  int
	Op        Op
	Goal      []float64
	Waypoints [][]float64
	// CancelSeq, when >= 0, identifies the previously accepted command that
	// CANCEL targets.
	CancelSeq int64
}

// ID uniquely identifies an accepted command.
type ID struct {
	Source   string
	Sequence int64
}

// Decision tells the engine how a submission was adjudicated.
type Decision struct {
	Accepted bool
	ID       ID
	// Preempted is set when acceptance aborts an already active command.
	Preempted bool
	// PreemptedID identifies the command that was preempted.
	PreemptedID ID
	Reason      classify.Reason
}

// Arbiter keeps the arbitration state.
type Arbiter struct{}

// New creates an Arbiter.
func New() *Arbiter { return &Arbiter{} }

// Submit applies ordering/dedup/priority/cancel rules to one command.
// active reports whether a motion command is currently executing and its id.
func (a *Arbiter) Submit(cmd Command, active bool, activeID ID) Decision {
	return Decision{}
}

// Complete marks the currently active command as finished (normal end or
// preempt/cancel) so its sequence number remains dedup-protected.
func (a *Arbiter) Complete(id ID) {}

// Reset clears all arbitration state (used only after an unrecoverable
// fault has been cleared by reconfiguration).
func (a *Arbiter) Reset() {}
