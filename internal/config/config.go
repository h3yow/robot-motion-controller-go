// Package config defines the validated configuration schema. Unknown fields,
// values outside the supported compatibility range and unsupported versions
// are rejected explicitly; no silent defaults are substituted for required
// physical parameters.
package config

import (
	"bytes"
	"encoding/json"
	"fmt"
	"io"
	"os"
	"strconv"
	"strings"
	"time"

	"github.com/h3yow/robot-motion-controller-go/internal/mcerr"
)

// Versioning constants. Major version differences are incompatible.
const (
	CurrentVersion = "1.0.0"
	MinMajor       = 1
	MaxMajor       = 1

	// SupportedDOF is the only supported planar arm joint count.
	SupportedDOF = 3
)

// PIDGains are per-axis joint-space PID gains.
//
// Units (joint space):
//
//	Kp: N·m / rad
//	Ki: N·m / (rad · s)
//	Kd: N·m / (rad / s) = N·m·s / rad
//	IntegralLimit: N·m (anti-windup bound on the integral term)
type PIDGains struct {
	Kp            float64 `json:"kp"`
	Ki            float64 `json:"ki"`
	Kd            float64 `json:"kd"`
	IntegralLimit float64 `json:"integral_limit"`
}

// AxisConfig describes one revolute joint. All angles are radians.
type AxisConfig struct {
	Name           string   `json:"name"`
	HomePosition   float64  `json:"home"`
	MinPosition    float64  `json:"min"`
	MaxPosition    float64  `json:"max"`
	MaxVelocity    float64  `json:"max_velocity"`    // rad/s
	MaxAccel       float64  `json:"max_accel"`       // rad/s^2
	MaxTorque      float64  `json:"max_torque"`      // N·m
	Inertia        float64  `json:"inertia"`         // kg·m^2 effective inertia
	GravityTorque  float64  `json:"gravity_torque"`  // N·m worst-case gravity load
	FrictionTorque float64  `json:"friction_torque"` // N·m Coulomb friction
	Gains          PIDGains `json:"gains"`
}

// Tolerance defines end-of-move convergence criteria.
type Tolerance struct {
	Position  float64 `json:"position"` // rad
	Velocity  float64 `json:"velocity"` // rad/s
	HoldTicks int     `json:"hold_ticks"`
}

// FaultPolicy configures detection thresholds and safe-stop behaviour.
type FaultPolicy struct {
	MaxPositionError   float64 `json:"max_position_error"`   // rad: exceed -> recoverable tracking fault
	DivergeError       float64 `json:"diverge_error"`        // rad: exceed -> fatal divergence fault
	DropoutMaxTicks    int     `json:"dropout_max_ticks"`    // tolerated consecutive feedback dropouts
	SaturationMaxTicks int     `json:"saturation_max_ticks"` // sustained saturation ticks -> fault
	EndGraceTicks      int     `json:"end_grace_ticks"`      // ticks after trajectory end to converge
	RecoveryHoldTicks  int     `json:"recovery_hold_ticks"`  // healthy ticks required to clear fault
	SafeDecel          float64 `json:"safe_decel"`           // rad/s^2 used for safe stopping
}

// duration wraps time.Duration with JSON unmarshalling ("2ms").
type duration time.Duration

func (d *duration) UnmarshalJSON(b []byte) error {
	var s string
	if err := json.Unmarshal(b, &s); err != nil {
		return mcerr.New(mcerr.CodeConfigInvalid, "duration must be a string such as \"2ms\"")
	}
	td, err := time.ParseDuration(s)
	if err != nil {
		return mcerr.Wrap(mcerr.CodeConfigInvalid, "invalid duration "+strconv.Quote(s), err)
	}
	if td <= 0 {
		return mcerr.New(mcerr.CodeConfigInvalid, "duration must be strictly positive, got "+s)
	}
	*d = duration(td)
	return nil
}

// Config is the validated root configuration.
type Config struct {
	Version   string        `json:"version"`
	Period    time.Duration `json:"-"`
	periodRaw duration
	Links     []float64    `json:"links"` // planar link lengths in meters
	Elbow     string       `json:"elbow"` // "up" or "down"
	Axes      []AxisConfig `json:"axes"`
	Tolerance Tolerance    `json:"tolerance"`
	Fault     FaultPolicy  `json:"fault"`
}

// rawConfig avoids reflection coupling around the period wrapper.
type rawConfig struct {
	Version   string       `json:"version"`
	Period    duration     `json:"period"`
	Links     []float64    `json:"links"`
	Elbow     string       `json:"elbow"`
	Axes      []AxisConfig `json:"axes"`
	Tolerance Tolerance    `json:"tolerance"`
	Fault     FaultPolicy  `json:"fault"`
}

// Parse reads and validates a JSON configuration stream. Unknown fields are
// rejected and every required physical parameter must be present explicitly.
func Parse(r io.Reader) (*Config, error) {
	data, err := io.ReadAll(r)
	if err != nil {
		return nil, mcerr.Wrap(mcerr.CodeConfigInvalid, "cannot read configuration", err)
	}
	var raw rawConfig
	dec := json.NewDecoder(bytes.NewReader(data))
	dec.DisallowUnknownFields()
	if err := dec.Decode(&raw); err != nil {
		return nil, mcerr.Wrap(mcerr.CodeConfigInvalid, "configuration JSON rejected", err)
	}
	if dec.More() {
		return nil, mcerr.New(mcerr.CodeConfigInvalid, "configuration must contain exactly one JSON object")
	}
	cfg := &Config{
		Version:   raw.Version,
		Period:    time.Duration(raw.Period),
		periodRaw: raw.Period,
		Links:     raw.Links,
		Elbow:     raw.Elbow,
		Axes:      raw.Axes,
		Tolerance: raw.Tolerance,
		Fault:     raw.Fault,
	}
	if err := cfg.Validate(); err != nil {
		return nil, err
	}
	return cfg, nil
}

// Load reads, parses and validates a JSON configuration file.
func Load(path string) (*Config, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, mcerr.Wrap(mcerr.CodeConfigInvalid, "cannot open configuration file "+path, err)
	}
	defer f.Close()
	return Parse(f)
}

// Validate checks all compatibility and physical constraints.
func (c *Config) Validate() error {
	major, _, _, err := parseVersion(c.Version)
	if err != nil {
		return mcerr.Wrap(mcerr.CodeConfigInvalid, "invalid version", err)
	}
	if major < MinMajor || major > MaxMajor {
		return mcerr.New(mcerr.CodeConfigVersionUnsupported,
			fmt.Sprintf("configuration version %q is outside the compatible range %d.x (current %s)",
				c.Version, MaxMajor, CurrentVersion))
	}
	if c.Period <= 0 {
		return mcerr.New(mcerr.CodeConfigInvalid, "period must be set and strictly positive")
	}
	if c.Elbow != "up" && c.Elbow != "down" {
		return mcerr.New(mcerr.CodeConfigInvalid, "elbow must be \"up\" or \"down\", got "+strconv.Quote(c.Elbow))
	}
	if len(c.Links) != SupportedDOF {
		return mcerr.New(mcerr.CodeConfigInvalid,
			fmt.Sprintf("exactly %d planar links are supported, got %d", SupportedDOF, len(c.Links)))
	}
	for i, l := range c.Links {
		if !(l > 0) {
			return mcerr.New(mcerr.CodeConfigInvalid,
				fmt.Sprintf("links[%d] must be strictly positive, got %v", i, l))
		}
	}
	if len(c.Axes) != SupportedDOF {
		return mcerr.New(mcerr.CodeConfigInvalid,
			fmt.Sprintf("exactly %d axes are supported, got %d", SupportedDOF, len(c.Axes)))
	}
	names := map[string]bool{}
	for i := range c.Axes {
		a := &c.Axes[i]
		if strings.TrimSpace(a.Name) == "" {
			return mcerr.New(mcerr.CodeConfigInvalid, fmt.Sprintf("axes[%d] requires a name", i))
		}
		if names[a.Name] {
			return mcerr.New(mcerr.CodeConfigInvalid, "duplicate axis name "+a.Name)
		}
		names[a.Name] = true
		if !(a.MinPosition < a.MaxPosition) {
			return mcerr.New(mcerr.CodeConfigInvalid,
				fmt.Sprintf("axis %s: min (%v) must be strictly less than max (%v)", a.Name, a.MinPosition, a.MaxPosition))
		}
		if a.HomePosition < a.MinPosition || a.HomePosition > a.MaxPosition {
			return mcerr.New(mcerr.CodeJointLimitExceeded,
				fmt.Sprintf("axis %s: home %v outside [%v, %v]", a.Name, a.HomePosition, a.MinPosition, a.MaxPosition))
		}
		if !(a.MaxVelocity > 0) {
			return mcerr.New(mcerr.CodeConfigInvalid, fmt.Sprintf("axis %s: max_velocity must be > 0", a.Name))
		}
		if !(a.MaxAccel > 0) {
			return mcerr.New(mcerr.CodeConfigInvalid, fmt.Sprintf("axis %s: max_accel must be > 0", a.Name))
		}
		if !(a.MaxTorque > 0) {
			return mcerr.New(mcerr.CodeConfigInvalid, fmt.Sprintf("axis %s: max_torque must be > 0", a.Name))
		}
		if !(a.Inertia > 0) {
			return mcerr.New(mcerr.CodeConfigInvalid, fmt.Sprintf("axis %s: inertia must be > 0", a.Name))
		}
		if a.GravityTorque < 0 || a.FrictionTorque < 0 {
			return mcerr.New(mcerr.CodeConfigInvalid,
				fmt.Sprintf("axis %s: gravity_torque and friction_torque must be >= 0", a.Name))
		}
		if a.GravityTorque+a.FrictionTorque >= a.MaxTorque {
			return mcerr.New(mcerr.CodeConfigInvalid,
				fmt.Sprintf("axis %s: passive load (%v+%v) must be strictly below max_torque %v or no motion is possible",
					a.Name, a.GravityTorque, a.FrictionTorque, a.MaxTorque))
		}
		if !(a.Gains.Kp > 0) {
			return mcerr.New(mcerr.CodeConfigInvalid, fmt.Sprintf("axis %s: gains.kp must be > 0", a.Name))
		}
		if a.Gains.Ki < 0 || a.Gains.Kd < 0 {
			return mcerr.New(mcerr.CodeConfigInvalid, fmt.Sprintf("axis %s: gains.ki/kd must be >= 0", a.Name))
		}
		if !(a.Gains.IntegralLimit > 0) {
			return mcerr.New(mcerr.CodeConfigInvalid, fmt.Sprintf("axis %s: gains.integral_limit must be > 0", a.Name))
		}
	}
	t := c.Tolerance
	if !(t.Position > 0) || !(t.Velocity > 0) || t.HoldTicks <= 0 {
		return mcerr.New(mcerr.CodeConfigInvalid, "tolerance.position/velocity must be > 0 and hold_ticks > 0")
	}
	f := c.Fault
	if !(f.MaxPositionError > 0) {
		return mcerr.New(mcerr.CodeConfigInvalid, "fault.max_position_error must be > 0")
	}
	if !(f.DivergeError > f.MaxPositionError) {
		return mcerr.New(mcerr.CodeConfigInvalid,
			fmt.Sprintf("fault.diverge_error (%v) must be strictly greater than max_position_error (%v)",
				f.DivergeError, f.MaxPositionError))
	}
	if f.DropoutMaxTicks <= 0 || f.SaturationMaxTicks <= 0 || f.EndGraceTicks <= 0 || f.RecoveryHoldTicks <= 0 {
		return mcerr.New(mcerr.CodeConfigInvalid,
			"fault dropout/saturation/grace/recovery tick counts must all be > 0")
	}
	if !(f.SafeDecel > 0) {
		return mcerr.New(mcerr.CodeConfigInvalid, "fault.safe_decel must be > 0")
	}
	return nil
}

// parseVersion accepts strict "MAJOR.MINOR.PATCH" numeric form.
func parseVersion(v string) (major, minor, patch int, err error) {
	parts := strings.Split(v, ".")
	if len(parts) != 3 {
		return 0, 0, 0, fmt.Errorf("version must be MAJOR.MINOR.PATCH, got %q", v)
	}
	out := make([]int, 3)
	for i, p := range parts {
		n, perr := strconv.Atoi(p)
		if perr != nil || n < 0 {
			return 0, 0, 0, fmt.Errorf("version component %q is not a non-negative integer", p)
		}
		out[i] = n
	}
	return out[0], out[1], out[2], nil
}

// MarshalJSON is provided so a validated Config can be re-serialized for
// inspection (the period uses the standard duration string form).
func (c *Config) MarshalJSON() ([]byte, error) {
	return json.Marshal(rawConfig{
		Version:   c.Version,
		Period:    c.periodRaw,
		Links:     c.Links,
		Elbow:     c.Elbow,
		Axes:      c.Axes,
		Tolerance: c.Tolerance,
		Fault:     c.Fault,
	})
}
