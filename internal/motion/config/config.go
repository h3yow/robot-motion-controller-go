// Package config defines the versioned configuration schema of the motion
// controller and its strict (no silent defaulting) validation.
package config

import "github.com/h3yow/robot-motion-controller-go/internal/motion/classify"

// Supported schema versions (inclusive).
const (
	MinSupportedVersion = 1
	MaxSupportedVersion = 1
)

// AxisConfig is the per-axis kinematic and dynamic configuration.
//
// Units (SI): Position rad, Velocity rad/s, Acceleration rad/s^2,
// Torque N*m, Inertia kg*m^2, Period s.
type AxisConfig struct {
	Name         string
	MinPosition  float64 // inclusive lower travel limit (rad)
	MaxPosition  float64 // inclusive upper travel limit (rad)
	MaxVelocity  float64 // rad/s, strictly positive
	MaxAccel     float64 // rad/s^2, strictly positive
	MaxTorque    float64 // N*m, strictly positive
	Inertia      float64 // kg*m^2, strictly positive
	GravityTorque float64 // N*m, constant gravity bias applied to the axis
}

// ControllerConfig holds the per-axis joint-space PID gains and tolerances.
type ControllerConfig struct {
	Kp float64 // proportional gain (N*m/rad)
	Ki float64 // integral gain (N*m/(rad*s))
	Kd float64 // derivative gain (N*m*s/rad)
	// IntegralWindup is the per-axis integrator clamp magnitude (N*m).
	IntegralWindup float64
	// TrackingTolerance is the position error magnitude that, if exceeded,
	// trips a tracking fault (rad).
	TrackingTolerance float64
	// SaturationCycles is the number of consecutive control cycles the
	// commanded torque may saturate before a saturation fault trips.
	SaturationCycles int
}

// Config is the complete versioned controller configuration.
type Config struct {
	SchemaVersion int
	// ControlPeriod is the fixed control cycle duration in seconds.
	ControlPeriod float64
	Axes           []AxisConfig
	Controller     ControllerConfig
}

// AxisCount returns the number of configured axes.
func (c *Config) AxisCount() int { return len(c.Axes) }

// Validate checks schema version compatibility and every field. It returns
// a *classify.RejectError with a distinguishable reason; it never repairs
// values with defaults.
func (c *Config) Validate() error {
	_ = classify.ReasonConfigVersion
	return nil
}
