// Package mcerr defines the shared, classifiable error vocabulary for the
// motion controller. Every failure path returns an *Error carrying a stable
// Code so that callers can distinguish rejection/fault reasons without
// parsing message text.
package mcerr

import "errors"

// Code is a stable, machine-readable failure classification.
type Code string

const (
	CodeConfigInvalid            Code = "config_invalid"
	CodeConfigVersionUnsupported Code = "config_version_unsupported"

	CodeTargetNonFinite     Code = "target_non_finite"
	CodeTargetUnreachable   Code = "target_unreachable"
	CodeTargetContradictory Code = "target_contradictory"
	CodeJointLimitExceeded  Code = "joint_limit_exceeded"

	CodeWorkspaceViolation Code = "workspace_violation"
	CodeVelocityLimit      Code = "velocity_limit_exceeded"
	CodeAccelerationLimit  Code = "acceleration_limit_exceeded"
	CodeTorqueLimit        Code = "torque_limit_exceeded"
	CodeInfeasibleStart    Code = "infeasible_start"

	CodeFeedbackLost         Code = "feedback_lost"
	CodeFeedbackDead         Code = "feedback_dead"
	CodeTrackingDiverged     Code = "tracking_diverged"
	CodeDriveSaturated       Code = "drive_saturated"
	CodeCommandNonFinite     Code = "command_non_finite"
	CodeNotConverged         Code = "not_converged"
	CodeFaultActive          Code = "fault_active"
	CodeRecoveryPrecondition Code = "recovery_precondition_unmet"

	CodeStaleSequence  Code = "stale_sequence"
	CodeDuplicateCmd   Code = "duplicate_command"
	CodePriorityTooLow Code = "priority_too_low"
	CodeCommandUnknown Code = "command_unknown"

	// CodeQueueFull rejects a new command/path/waypoint when admitting it
	// would exceed the configured max pending command budget.
	CodeQueueFull Code = "queue_full"
	// CodeOperationInvalid rejects an operation whose target is not in a
	// state where that operation applies (e.g. pause a queued command,
	// append to an already finished path).
	CodeOperationInvalid Code = "operation_invalid"
)

// Error is the classified error type used across all packages.
type Error struct {
	Code    Code
	Message string
	Cause   error
}

func (e *Error) Error() string {
	if e.Cause != nil {
		return string(e.Code) + ": " + e.Message + ": " + e.Cause.Error()
	}
	return string(e.Code) + ": " + e.Message
}

func (e *Error) Unwrap() error { return e.Cause }

// New creates a classified error.
func New(code Code, message string) *Error {
	return &Error{Code: code, Message: message}
}

// Wrap creates a classified error wrapping cause.
func Wrap(code Code, message string, cause error) *Error {
	return &Error{Code: code, Message: message, Cause: cause}
}

// HasCode reports whether err (or any error in its chain) carries code.
func HasCode(err error, code Code) bool {
	var me *Error
	if errors.As(err, &me) {
		return me.Code == code
	}
	return false
}

// CodeOf returns the classified code carried by err, or "" if none.
func CodeOf(err error) Code {
	var me *Error
	if errors.As(err, &me) {
		return me.Code
	}
	return ""
}

// ErrNotImplemented is only used by skeleton stubs.
var ErrNotImplemented = errors.New("not implemented")
