package config_test

import (
	"testing"

	"github.com/h3yow/robot-motion-controller-go/internal/motion/classify"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/config"
	"github.com/h3yow/robot-motion-controller-go/internal/motion/mctest"
)

func base() *config.Config {
	return &config.Config{
		SchemaVersion: 1, ControlPeriod: 0.01,
		Axes: []config.AxisConfig{
			{Name: "j1", MinPosition: -1, MaxPosition: 1, MaxVelocity: 2, MaxAccel: 4, MaxTorque: 10, Inertia: 1},
		},
		Controller: config.ControllerConfig{Kp: 1, Kd: 1, IntegralWindup: 2, TrackingTolerance: 0.05, SaturationCycles: 3},
	}
}

func TestValidate(t *testing.T) {
	mt := mctest.Wrap(t)
	mt.CheckNoErr("base", "default", base().Validate())

	type tc struct {
		name   string
		mutate func(c *config.Config)
		reason classify.Reason
	}
	cases := []tc{
		{"version too old", func(c *config.Config) { c.SchemaVersion = 0 }, classify.ReasonConfigVersion},
		{"version too new", func(c *config.Config) { c.SchemaVersion = 2 }, classify.ReasonConfigVersion},
		{"zero period", func(c *config.Config) { c.ControlPeriod = 0 }, classify.ReasonConfigField},
		{"nan period", func(c *config.Config) { c.ControlPeriod = nan() }, classify.ReasonConfigField},
		{"no axes", func(c *config.Config) { c.Axes = nil }, classify.ReasonConfigField},
		{"bad travel", func(c *config.Config) { c.Axes[0].MinPosition = 2 }, classify.ReasonConfigField},
		{"zero velocity", func(c *config.Config) { c.Axes[0].MaxVelocity = 0 }, classify.ReasonConfigField},
		{"neg accel", func(c *config.Config) { c.Axes[0].MaxAccel = -1 }, classify.ReasonConfigField},
		{"zero torque", func(c *config.Config) { c.Axes[0].MaxTorque = 0 }, classify.ReasonConfigField},
		{"zero inertia", func(c *config.Config) { c.Axes[0].Inertia = 0 }, classify.ReasonConfigField},
		{"dup name", func(c *config.Config) {
			c.Axes = append(c.Axes, c.Axes[0])
		}, classify.ReasonConfigField},
		{"bad tol", func(c *config.Config) { c.Controller.TrackingTolerance = 0 }, classify.ReasonConfigField},
		{"bad sat cyc", func(c *config.Config) { c.Controller.SaturationCycles = 0 }, classify.ReasonConfigField},
		{"neg kp", func(c *config.Config) { c.Controller.Kp = -1 }, classify.ReasonConfigField},
	}
	for _, c := range cases {
		cfg := base()
		c.mutate(cfg)
		err := cfg.Validate()
		mt.CheckReason(c.name, c.name, err, c.reason)
	}
}

func nan() float64 { return nanImpl() }
