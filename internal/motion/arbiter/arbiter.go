// Package arbiter arbitrates concurrent, possibly duplicated, late or
// out-of-order motion commands.
//
// Rules (all evaluated under one mutex):
//
//   - Sequence numbers are scoped to Source and must be strictly increasing.
//     A (Source,Sequence) pair already observed is rejected as DUPLICATE; an
//     older sequence as STALE. Neither can cause repeated execution or state
//     rollback, even after preemption/completion.
//   - A new motion command while another is active preempts only with
//     strictly higher Priority; ties lose (PRIORITY_DENIED).
//   - CANCEL only affects the currently active command identified by the
//     cancel source and CancelSeq.
//   - While a fault is active, motion commands are rejected FAULT_ACTIVE;
//     RESET is accepted only then.
package arbiter

import (
	"fmt"
	"sync"

	"github.com/h3yow/robot-motion-controller-go/internal/motion/classify"
)

// Op is the command operation type.
type Op string

const (
	OpMoveTo Op = "MOVE_TO"
	OpPath   Op = "PATH"
	OpCancel Op = "CANCEL"
	OpReset  Op = "RESET"
)

// Command is an arbitrated command envelope.
type Command struct {
	Source    string
	Sequence  int64
	Priority  int // higher wins; ties keep the incumbent
	Op        Op
	Goal      []float64
	Waypoints [][]float64
	// CancelSeq identifies the accepted command targeted by CANCEL.
	CancelSeq int64
}

// ID uniquely identifies an accepted command.
type ID struct {
	Source   string
	Sequence int64
}

func (id ID) String() string { return fmt.Sprintf("%s#%d", id.Source, id.Sequence) }

// Decision tells the engine how a submission was adjudicated.
type Decision struct {
	Accepted bool
	ID       ID
	// Preempted marks that acceptance aborted an already active command.
	Preempted   bool
	PreemptedID ID
	Reason      classify.Reason
	Message     string
}

// Arbiter keeps the arbitration state.
type Arbiter struct {
	mu      sync.Mutex
	lastSeq map[string]int64
	seen    map[ID]bool
}

// New creates an Arbiter.
func New() *Arbiter {
	return &Arbiter{lastSeq: map[string]int64{}, seen: map[ID]bool{}}
}

// Submit applies all arbitration rules under one lock.
//
//	active         - a motion command is currently executing
//	activeID       - its id
//	activePriority - its priority
//	faultActive    - a fault state is currently latched (motion barred)
func (a *Arbiter) Submit(cmd Command, active bool, activeID ID, activePriority int, faultActive bool) Decision {
	a.mu.Lock()
	defer a.mu.Unlock()

	if cmd.Source == "" {
		return Decision{Reason: classify.ReasonInvalidCommand, Message: "empty command source"}
	}
	id := ID{Source: cmd.Source, Sequence: cmd.Sequence}

	switch cmd.Op {
	case OpCancel, OpReset:
		// Dedup is recorded in the watermark; malformed retries (wrong
		// target, no active fault) are NOT recorded so an identical retry
		// with a correct precondition can still take effect.
		if d, ok := a.dedupLocked(id, cmd.Sequence); !ok {
			return d
		}
		if cmd.Op == OpCancel {
			if !active || activeID.Source != cmd.Source || activeID.Sequence != cmd.CancelSeq {
				return Decision{ID: id, Reason: classify.ReasonInvalidCommand,
					Message: fmt.Sprintf("cancel targets %s#%d but active is %s (active=%v)",
						cmd.Source, cmd.CancelSeq, activeID, active)}
			}
			a.recordLocked(id, cmd.Sequence)
			return Decision{Accepted: true, ID: id, Preempted: true, PreemptedID: activeID,
				Reason: classify.ReasonCancelled}
		}
		if !faultActive {
			return Decision{ID: id, Reason: classify.ReasonInvalidCommand,
				Message: "reset submitted without an active fault"}
		}
		a.recordLocked(id, cmd.Sequence)
		return Decision{Accepted: true, ID: id}

	case OpMoveTo, OpPath:
		if d, ok := a.dedupLocked(id, cmd.Sequence); !ok {
			return d
		}
		if faultActive {
			return Decision{ID: id, Reason: classify.ReasonFaultActive,
				Message: "motion rejected while fault state is latched; reset/reconfigure first"}
		}
		if (cmd.Op == OpMoveTo && len(cmd.Goal) == 0) ||
			(cmd.Op == OpPath && len(cmd.Waypoints) == 0) {
			return Decision{ID: id, Reason: classify.ReasonInvalidCommand,
				Message: fmt.Sprintf("%s missing target data", cmd.Op)}
		}
		if active && cmd.Priority <= activePriority {
			return Decision{ID: id, Reason: classify.ReasonPriorityDenied,
				Message: fmt.Sprintf("priority %d does not strictly exceed incumbent priority %d (%s)",
					cmd.Priority, activePriority, activeID)}
		}
		a.recordLocked(id, cmd.Sequence)
		d := Decision{Accepted: true, ID: id}
		if active {
			d.Preempted = true
			d.PreemptedID = activeID
		}
		return d

	default:
		return Decision{Reason: classify.ReasonInvalidCommand, Message: "unknown op " + string(cmd.Op)}
	}
}

func (a *Arbiter) dedupLocked(id ID, seq int64) (Decision, bool) {
	if a.seen[id] {
		return Decision{ID: id, Reason: classify.ReasonDuplicateCommand,
			Message: fmt.Sprintf("command %s already observed", id)}, false
	}
	if last, ok := a.lastSeq[id.Source]; ok && seq <= last {
		return Decision{ID: id, Reason: classify.ReasonStaleCommand,
			Message: fmt.Sprintf("command %s older/equal to watermark %d", id, last)}, false
	}
	return Decision{}, true
}

func (a *Arbiter) recordLocked(id ID, seq int64) {
	a.seen[id] = true
	a.lastSeq[id.Source] = seq
}

// Complete retains the watermark/seen-set after a command finishes so late
// replays remain rejected.
func (a *Arbiter) Complete(id ID) {
	a.mu.Lock()
	defer a.mu.Unlock()
	a.seen[id] = true
}

// Reset clears all arbitration state (only after an unrecoverable fault has
// been cleared through reconfiguration).
func (a *Arbiter) Reset() {
	a.mu.Lock()
	defer a.mu.Unlock()
	a.lastSeq = map[string]int64{}
	a.seen = map[ID]bool{}
}
