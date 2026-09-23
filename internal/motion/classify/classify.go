// Package classify defines machine-readable, mutually distinguishable
// rejection reasons and fault kinds used across the motion controller.
package classify

import "errors"

// Reason is a stable, machine-readable code identifying why an input was
// rejected or why a command did not take effect.
type Reason string

const (
	// ReasonOK means no rejection.
	ReasonOK Reason = ""
	// ReasonConfigVersion is returned when the configuration schema version
	// is outside the supported compatibility range.
	ReasonConfigVersion Reason = "CONFIG_VERSION_UNSUPPORTED"
	// ReasonConfigField is returned when a configuration field is out of its
	// allowed range or otherwise invalid.
	ReasonConfigField Reason = "CONFIG_FIELD_INVALID"
	// ReasonTargetNonFinite is returned when a target pose/vector contains
	// NaN or +-Inf.
	ReasonTargetNonFinite Reason = "TARGET_NON_FINITE"
	// ReasonTargetUnreachable is returned when a target pose has no inverse
	// kinematics solution.
	ReasonTargetUnreachable Reason = "TARGET_UNREACHABLE"
	// ReasonOutOfTravel is returned when a joint position is outside the
	// configured travel (position) limit.
	ReasonOutOfTravel Reason = "OUT_OF_TRAVEL"
	// ReasonVelocityLimit is returned when requested motion exceeds a
	// velocity limit.
	ReasonVelocityLimit Reason = "VELOCITY_LIMIT_EXCEEDED"
	// ReasonAccelerationLimit is returned when requested motion exceeds an
	// acceleration limit.
	ReasonAccelerationLimit Reason = "ACCELERATION_LIMIT_EXCEEDED"
	// ReasonTorqueLimit is returned when a trajectory/request cannot be
	// tracked within the torque limit.
	ReasonTorqueLimit Reason = "TORQUE_LIMIT_EXCEEDED"
	// ReasonContradictoryTarget is returned when a request is
	// self-contradictory (e.g. zero duration with nonzero displacement,
	// mismatched vector lengths).
	ReasonContradictoryTarget Reason = "CONTRADICTORY_TARGET"
	// ReasonStaleCommand is returned when a command's sequence number is
	// older than a previously observed command from the same source.
	ReasonStaleCommand Reason = "STALE_COMMAND"
	// ReasonDuplicateCommand is returned when exactly the same
	// (source, sequence) command is submitted after it was already seen.
	ReasonDuplicateCommand Reason = "DUPLICATE_COMMAND"
	// ReasonCancelled is returned when a motion was aborted by a cancel.
	ReasonCancelled Reason = "CANCELLED"
	// ReasonFaultActive is returned when a command cannot be accepted
	// because a fault state is currently active.
	ReasonFaultActive Reason = "FAULT_ACTIVE"
	// ReasonResetNotSatisfied is returned when a reset/recovery is requested
	// while recovery preconditions are not met.
	ReasonResetNotSatisfied Reason = "RESET_PRECONDITIONS_NOT_MET"
	// ReasonInvalidCommand is returned for malformed command envelopes.
	ReasonInvalidCommand Reason = "INVALID_COMMAND"
	// ReasonPriorityDenied is returned when a command cannot preempt the
	// incumbent because its priority is not strictly higher.
	ReasonPriorityDenied Reason = "PRIORITY_DENIED"
)

// RejectError is the single structured error type for input/command
// rejections. The Reason code is stable and machine-readable; Msg gives the
// human explanation with the concrete offending values.
type RejectError struct {
	Reason Reason
	Msg    string
}

func (e *RejectError) Error() string {
	if e == nil {
		return ""
	}
	return string(e.Reason) + ": " + e.Msg
}

// New builds a RejectError.
func New(reason Reason, msg string) *RejectError {
	return &RejectError{Reason: reason, Msg: msg}
}

// AsReject reports whether err carries a RejectError (directly or wrapped)
// and returns it.
func AsReject(err error) (*RejectError, bool) {
	if err == nil {
		return nil, false
	}
	var r *RejectError
	if errors.As(err, &r) {
		return r, true
	}
	return nil, false
}
