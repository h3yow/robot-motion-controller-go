// Package arbiter decides which motion command is effective when several
// entrances submit, preempt or cancel commands concurrently.
//
// Rules (deterministic, no time dependency):
//
//   - strictly higher priority preempts the active command immediately;
//     the preempted command is terminal and never resumes;
//   - equal or lower priority while a command is active enters a FIFO
//     queue; ties (including priority) are admitted in arrival order;
//   - per-entrance Seq must be strictly increasing; a lower/equal Seq is
//     rejected as stale and is never executed;
//   - reusing an ID with the same (entrance, seq) is idempotent — the
//     recorded outcome is returned, so retries never execute twice;
//   - reusing an ID with different content is rejected as a duplicate;
//   - terminal outcomes are retained forever for idempotent lookup.
package arbiter

import (
	"fmt"
	"sort"
	"sync"

	"github.com/h3yow/robot-motion-controller-go/internal/mcerr"
)

// Priority is the command precedence. Higher wins.
type Priority uint8

const (
	PriorityLow Priority = iota
	PriorityNormal
	PriorityHigh
	PriorityEmergency
)

// CmdState is the lifecycle state of a command.
type CmdState string

const (
	StateQueued       CmdState = "queued"
	StateRunning      CmdState = "running"
	StateStopping     CmdState = "stopping"
	StateCompleted    CmdState = "completed"
	StateCanceled     CmdState = "canceled"
	StatePreempted    CmdState = "preempted"
	StateRejected     CmdState = "rejected"
	StateFaulted      CmdState = "faulted"             // fatal, terminal
	StateFaultedRecov CmdState = "faulted_recoverable" // recoverable, non-terminal
)

// Command is one motion request from one entrance.
type Command struct {
	ID       uint64
	Entrance string
	Seq      uint64 // strictly monotonic per entrance
	Priority Priority
	Goal     []float64
}

// entry is the retained per-command record.
type entry struct {
	cmd     Command
	state   CmdState
	reason  string
	arrival int // monotonic arrival counter for FIFO tie-breaking
}

// Outcome is the terminal (or current) result of a command.
type Outcome struct {
	ID     uint64
	State  CmdState
	Reason string
}

// SubmitResult reports what arbitration decided for a submission.
type SubmitResult struct {
	State       CmdState
	PreemptedID uint64 // non-zero when the new command preempts the active one
	Duplicate   bool   // true when an identical command already exists
}

// Arbiter is a mutex-protected, purely logical decision maker.
type Arbiter struct {
	mu       sync.Mutex
	entries  map[uint64]*entry
	activeID uint64
	queue    []uint64
	lastSeq  map[string]uint64
	arrival  int
	// capacity bounds the number of simultaneously pending (active + queued)
	// commands. Zero means unbounded (used by package tests only); the motion
	// service always constructs with its configured capacity.
	capacity int
}

// New creates an empty arbiter with no pending-command cap. Production code
// uses NewWithCapacity so an over-long queue is rejected decisively.
func New() *Arbiter {
	return NewWithCapacity(0)
}

// NewWithCapacity creates an empty arbiter that rejects new queued/active
// admissions (queue_full) once pending commands reach capacity. capacity must
// be >= 1; zero means unbounded.
func NewWithCapacity(capacity int) *Arbiter {
	return &Arbiter{
		entries:  map[uint64]*entry{},
		lastSeq:  map[string]uint64{},
		capacity: capacity,
	}
}

// Submit validates and admits a command. See package doc for the exact
// precedence, staleness and idempotency rules.
func (a *Arbiter) Submit(cmd *Command) (*SubmitResult, error) {
	if cmd == nil {
		return nil, mcerr.New(mcerr.CodeTargetContradictory, "nil command")
	}
	if cmd.Entrance == "" {
		return nil, mcerr.New(mcerr.CodeTargetContradictory, "command requires a non-empty entrance")
	}
	if cmd.Priority > PriorityEmergency {
		return nil, mcerr.New(mcerr.CodeConfigInvalid,
			fmt.Sprintf("unsupported priority %d", cmd.Priority))
	}
	if cmd.ID == 0 {
		return nil, mcerr.New(mcerr.CodeTargetContradictory, "command ID must be non-zero")
	}
	a.mu.Lock()
	defer a.mu.Unlock()

	if prior, ok := a.entries[cmd.ID]; ok {
		if prior.cmd.Entrance != cmd.Entrance || prior.cmd.Seq != cmd.Seq {
			return nil, mcerr.New(mcerr.CodeDuplicateCmd,
				fmt.Sprintf("command ID %d reused with different entrance/seq (existing %s#%d)",
					cmd.ID, prior.cmd.Entrance, prior.cmd.Seq))
		}
		// Identical retry: report the current/terminal outcome.
		return &SubmitResult{State: prior.state, Duplicate: true}, nil
	}
	if last, ok := a.lastSeq[cmd.Entrance]; ok && cmd.Seq <= last {
		// A rejected (including capacity-rejected) command commits no sequence
		// number, so a client that retries with the same seq after a rejection
		// is not permanently poisoned; genuinely stale seqs are still refused.
		return nil, mcerr.New(mcerr.CodeStaleSequence,
			fmt.Sprintf("entrance %s seq %d is not greater than last seen %d", cmd.Entrance, cmd.Seq, last))
	}

	// Capacity: count the active slot (running/stopping/faulted-recoverable)
	// and the FIFO queue. A command that preempts the active one does not
	// grow the pending count and is therefore admitted even when full.
	if a.capacity > 0 && a.activeID != 0 {
		active := a.entries[a.activeID]
		if !(cmd.Priority > active.cmd.Priority) {
			pending := 1 + len(a.queue)
			if pending >= a.capacity {
				return nil, mcerr.New(mcerr.CodeQueueFull,
					fmt.Sprintf("pending command limit %d reached (1 active + %d queued); wait, cancel or preempt",
						a.capacity, len(a.queue)))
			}
		}
	}

	a.arrival++
	e := &entry{cmd: *cmd, arrival: a.arrival}
	a.entries[cmd.ID] = e
	a.lastSeq[cmd.Entrance] = cmd.Seq

	res := &SubmitResult{}
	if a.activeID == 0 {
		a.activeID = cmd.ID
		e.state = StateRunning
		res.State = StateRunning
		return res, nil
	}
	active := a.entries[a.activeID]
	if cmd.Priority > active.cmd.Priority {
		// Immediate preemption.
		active.state = StatePreempted
		active.reason = fmt.Sprintf("preempted by command %d (priority %d > %d)",
			cmd.ID, cmd.Priority, active.cmd.Priority)
		res.PreemptedID = a.activeID
		a.activeID = cmd.ID
		e.state = StateRunning
		res.State = StateRunning
		return res, nil
	}
	// Same or lower priority: queue in arrival order.
	e.state = StateQueued
	a.queue = append(a.queue, cmd.ID)
	res.State = StateQueued
	return res, nil
}

// CheckSeq validates an entrance sequence for an operation that does not
// itself create a command (such as appending waypoints to an admitted path).
// On success the sequence is committed, so the same entrance cannot later
// reuse it for a new command. A rejected check commits nothing.
func (a *Arbiter) CheckSeq(entrance string, seq uint64) error {
	if entrance == "" {
		return mcerr.New(mcerr.CodeTargetContradictory, "operation requires a non-empty entrance")
	}
	a.mu.Lock()
	defer a.mu.Unlock()
	if last, ok := a.lastSeq[entrance]; ok && seq <= last {
		return mcerr.New(mcerr.CodeStaleSequence,
			fmt.Sprintf("entrance %s seq %d is not greater than last seen %d", entrance, seq, last))
	}
	a.lastSeq[entrance] = seq
	return nil
}

// Cancel marks a command canceled. A running command moves to stopping (the
// executor performs the safe ramp and then calls Finish); a queued command
// is removed and finished immediately; an already terminal command reports
// its outcome idempotently. Unknown IDs are rejected.
func (a *Arbiter) Cancel(id uint64) (*Command, error) {
	a.mu.Lock()
	defer a.mu.Unlock()
	e, ok := a.entries[id]
	if !ok {
		return nil, mcerr.New(mcerr.CodeCommandUnknown, fmt.Sprintf("unknown command %d", id))
	}
	switch e.state {
	case StateCompleted, StateCanceled, StatePreempted, StateFaulted, StateRejected:
		out := e.cmd
		return &out, nil
	case StateQueued:
		a.removeFromQueue(id)
		e.state = StateCanceled
		e.reason = "canceled while queued"
	case StateRunning:
		e.state = StateStopping
		e.reason = "cancel requested"
	case StateStopping:
		// already stopping; idempotent
	}
	out := e.cmd
	return &out, nil
}

func (a *Arbiter) removeFromQueue(id uint64) {
	for i, qid := range a.queue {
		if qid == id {
			a.queue = append(a.queue[:i], a.queue[i+1:]...)
			return
		}
	}
}

// Active returns a copy of the currently effective command or nil.
func (a *Arbiter) Active() *Command {
	a.mu.Lock()
	defer a.mu.Unlock()
	if a.activeID == 0 {
		return nil
	}
	cmd := a.entries[a.activeID].cmd
	return &cmd
}

// PopQueued returns and removes the highest-priority queued command (FIFO on
// ties), or nil when the queue is empty.
func (a *Arbiter) PopQueued() *Command {
	a.mu.Lock()
	defer a.mu.Unlock()
	if len(a.queue) == 0 {
		return nil
	}
	best := 0
	for i := 1; i < len(a.queue); i++ {
		cand := a.entries[a.queue[i]]
		win := a.entries[a.queue[best]]
		if cand.cmd.Priority > win.cmd.Priority ||
			(cand.cmd.Priority == win.cmd.Priority && cand.arrival < win.arrival) {
			best = i
		}
	}
	id := a.queue[best]
	a.queue = append(a.queue[:best], a.queue[best+1:]...)
	cmd := a.entries[id].cmd
	return &cmd
}

// SetActive installs id as the running command (used after PopQueued).
func (a *Arbiter) SetActive(id uint64) {
	a.mu.Lock()
	defer a.mu.Unlock()
	if _, ok := a.entries[id]; !ok {
		return
	}
	a.activeID = id
	a.entries[id].state = StateRunning
}

// MarkRunning transitions a queued/stopping command to running.
func (a *Arbiter) MarkRunning(id uint64) {
	a.mu.Lock()
	defer a.mu.Unlock()
	if e, ok := a.entries[id]; ok && (e.state == StateQueued || e.state == StateStopping) {
		e.state = StateRunning
	}
}

// Finish records a terminal outcome. Repeated calls are ignored, so the
// first terminal transition wins (no state rollback). A recoverable fault
// is not terminal; use MarkRecovered to resume it.
func (a *Arbiter) Finish(id uint64, state CmdState, reason string) {
	a.mu.Lock()
	defer a.mu.Unlock()
	e, ok := a.entries[id]
	if !ok {
		return
	}
	switch e.state {
	case StateCompleted, StateCanceled, StatePreempted, StateFaulted, StateRejected:
		return
	}
	e.state = state
	e.reason = reason
	if a.activeID == id && state != StateFaultedRecov {
		a.activeID = 0
	}
}

// MarkRecovered moves a recoverable-faulted command back to running after
// the controller successfully recovered. It is a no-op in other states so
// late/duplicate recovery notifications cannot cause state rollback.
func (a *Arbiter) MarkRecovered(id uint64) bool {
	a.mu.Lock()
	defer a.mu.Unlock()
	e, ok := a.entries[id]
	if !ok || e.state != StateFaultedRecov {
		return false
	}
	e.state = StateRunning
	e.reason = "recovered after fault preconditions held"
	if a.activeID == 0 {
		a.activeID = id
	}
	return true
}

// MarkFatal escalates a recoverable fault to a terminal fatal fault.
func (a *Arbiter) MarkFatal(id uint64, reason string) {
	a.mu.Lock()
	defer a.mu.Unlock()
	e, ok := a.entries[id]
	if !ok || e.state != StateFaultedRecov {
		return
	}
	e.state = StateFaulted
	e.reason = reason
	if a.activeID == id {
		a.activeID = 0
	}
}

// ActiveID returns the active command ID (0 if none).
func (a *Arbiter) ActiveID() uint64 {
	a.mu.Lock()
	defer a.mu.Unlock()
	return a.activeID
}

// Outcome returns the recorded outcome for a command.
func (a *Arbiter) Outcome(id uint64) (*Outcome, bool) {
	a.mu.Lock()
	defer a.mu.Unlock()
	e, ok := a.entries[id]
	if !ok {
		return nil, false
	}
	return &Outcome{ID: id, State: e.state, Reason: e.reason}, true
}

// Snapshot returns a consistent copy of all command states, ordered by ID.
func (a *Arbiter) Snapshot() map[uint64]CmdState {
	a.mu.Lock()
	defer a.mu.Unlock()
	out := make(map[uint64]CmdState, len(a.entries))
	for id, e := range a.entries {
		out[id] = e.state
	}
	return out
}

// QueuedIDs returns queued command IDs in priority/FIFO order.
func (a *Arbiter) QueuedIDs() []uint64 {
	a.mu.Lock()
	defer a.mu.Unlock()
	ids := append([]uint64(nil), a.queue...)
	sort.SliceStable(ids, func(i, j int) bool {
		ei, ej := a.entries[ids[i]], a.entries[ids[j]]
		if ei.cmd.Priority != ej.cmd.Priority {
			return ei.cmd.Priority > ej.cmd.Priority
		}
		return ei.arrival < ej.arrival
	})
	return ids
}

// PendingCount reports how many commands currently occupy the pending slots:
// one for the active command (0 when idle) plus the FIFO queue length.
func (a *Arbiter) PendingCount() (active, queued int) {
	a.mu.Lock()
	defer a.mu.Unlock()
	if a.activeID != 0 {
		active = 1
	}
	return active, len(a.queue)
}

// Capacity returns the configured pending-command capacity (0 = unbounded).
func (a *Arbiter) Capacity() int {
	return a.capacity
}
