// Package config defines the versioned configuration schema of the motion
// controller and its strict (no silent defaulting) validation.
package config

import (
	"fmt"
	"math"

	"github.com/h3yow/robot-motion-controller-go/internal/motion/classify"
)

// Supported schema versions (inclusive). A configuration whose version is
// outside this range is rejected; it is never silently down/up-graded.
const (
	MinSupportedVersion = 1
	MaxSupportedVersion = 1
)

// AxisConfig is the per-axis kinematic and dynamic configuration.
//
// Units (SI):
//
//	Position rad, Velocity rad/s, Acceleration rad/s^2,
//	Torque N*m, Inertia kg*m^2, Period s.
type AxisConfig struct {
	Name          string
	MinPosition   float64 // inclusive lower travel limit (rad)
	MaxPosition   float64 // inclusive upper travel limit (rad)
	MaxVelocity   float64 // rad/s, strictly positive
	MaxAccel      float64 // rad/s^2, strictly positive
	MaxTorque     float64 // N*m, strictly positive
	Inertia       float64 // kg*m^2, strictly positive
	GravityTorque float64 // N*m, constant gravity bias applied to the axis
}

// ControllerConfig holds the joint-space PID gains and tolerances. The same
// gains are applied to every axis.
type ControllerConfig struct {
	Kp float64 // proportional gain (N*m/rad), >= 0
	Ki float64 // integral gain (N*m/(rad*s)), >= 0
	Kd float64 // derivative gain (N*m*s/rad), >= 0
	// IntegralWindup is the per-axis integrator clamp magnitude (N*m), >= 0.
	IntegralWindup float64
	// TrackingTolerance is the position-error magnitude (rad) that, if
	// exceeded, trips a tracking fault. Strictly positive.
	TrackingTolerance float64
	// SaturationCycles is the number of consecutive control cycles the
	// commanded torque may saturate before a saturation fault trips. >= 1.
	SaturationCycles int
}

// Config is the complete versioned controller configuration.
type Config struct {
	SchemaVersion int
	// ControlPeriod is the fixed control cycle duration in seconds.
	ControlPeriod float64
	Axes          []AxisConfig
	Controller    ControllerConfig
}

// AxisCount returns the number of configured axes.
func (c *Config) AxisCount() int { return len(c.Axes) }

func finite(x float64) bool { return !math.IsNaN(x) && !math.IsInf(x, 0) }

// Validate checks schema version compatibility and every field. It returns
// a *classify.RejectError with a distinguishable reason; it never repairs
// values with defaults.
func (c *Config) Validate() error {
	if c.SchemaVersion < MinSupportedVersion || c.SchemaVersion > MaxSupportedVersion {
		return classify.New(classify.ReasonConfigVersion,
			fmt.Sprintf("schema version %d not in supported range [%d,%d]",
				c.SchemaVersion, MinSupportedVersion, MaxSupportedVersion))
	}
	if !finite(c.ControlPeriod) || c.ControlPeriod <= 0 {
		return classify.New(classify.ReasonConfigField,
			fmt.Sprintf("control_period must be a finite positive number, got %v", c.ControlPeriod))
	}
	if len(c.Axes) == 0 {
		return classify.New(classify.ReasonConfigField, "at least one axis must be configured")
	}
	names := make(map[string]bool, len(c.Axes))
	for i, a := range c.Axes {
		tag := fmt.Sprintf("axis[%d]", i)
		if a.Name == "" {
			return classify.New(classify.ReasonConfigField, tag+" name must not be empty")
		}
		if names[a.Name] {
			return classify.New(classify.ReasonConfigField, tag+" duplicate axis name "+a.Name)
		}
		names[a.Name] = true
		pos := func(v float64, field string) error {
			if !finite(v) || v <= 0 {
				return classify.New(classify.ReasonConfigField,
					fmt.Sprintf("%s %s must be finite and > 0, got %v", tag, field, v))
			}
			return nil
		}
		if !finite(a.MinPosition) || !finite(a.MaxPosition) {
			return classify.New(classify.ReasonConfigField, tag+" travel limits must be finite")
		}
		if a.MinPosition >= a.MaxPosition {
			return classify.New(classify.ReasonConfigField,
				fmt.Sprintf("%s min_position %v must be < max_position %v", tag, a.MinPosition, a.MaxPosition))
		}
		if err := pos(a.MaxVelocity, "max_velocity"); err != nil {
			return err
		}
		if err := pos(a.MaxAccel, "max_accel"); err != nil {
			return err
		}
		if err := pos(a.MaxTorque, "max_torque"); err != nil {
			return err
		}
		if err := pos(a.Inertia, "inertia"); err != nil {
			return err
		}
		if !finite(a.GravityTorque) {
			return classify.New(classify.ReasonConfigField, tag+" gravity_torque must be finite")
		}
	}
	cc := c.Controller
	if !finite(cc.Kp) || cc.Kp < 0 {
		return classify.New(classify.ReasonConfigField,
			fmt.Sprintf("controller kp must be finite and >= 0, got %v", cc.Kp))
	}
	if !finite(cc.Ki) || cc.Ki < 0 {
		return classify.New(classify.ReasonConfigField,
			fmt.Sprintf("controller ki must be finite and >= 0, got %v", cc.Ki))
	}
	if !finite(cc.Kd) || cc.Kd < 0 {
		return classify.New(classify.ReasonConfigField,
			fmt.Sprintf("controller kd must be finite and >= 0, got %v", cc.Kd))
	}
	if !finite(cc.IntegralWindup) || cc.IntegralWindup < 0 {
		return classify.New(classify.ReasonConfigField,
			fmt.Sprintf("integral_windup must be finite and >= 0, got %v", cc.IntegralWindup))
	}
	if !finite(cc.TrackingTolerance) || cc.TrackingTolerance <= 0 {
		return classify.New(classify.ReasonConfigField,
			fmt.Sprintf("tracking_tolerance must be finite and > 0, got %v", cc.TrackingTolerance))
	}
	if cc.SaturationCycles < 1 {
		return classify.New(classify.ReasonConfigField,
			fmt.Sprintf("saturation_cycles must be >= 1, got %d", cc.SaturationCycles))
	}
	return nil
}
