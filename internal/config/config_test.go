package config

import (
	"strings"
	"testing"

	"github.com/h3yow/robot-motion-controller-go/internal/mcerr"
)

const validBase = `{
  "version": "1.0.0",
  "period": "2ms",
  "links": [0.4, 0.3, 0.1],
  "elbow": "up",
  "axes": [
    {"name":"j1","home":0.0,"min":-3.14,"max":3.14,"max_velocity":2.0,"max_accel":4.0,"max_torque":20.0,"inertia":1.0,"gravity_torque":1.0,"friction_torque":0.5,
     "gains":{"kp":20.0,"ki":2.0,"kd":4.0,"integral_limit":5.0}},
    {"name":"j2","home":0.0,"min":-2.5,"max":2.5,"max_velocity":2.5,"max_accel":5.0,"max_torque":18.0,"inertia":0.8,"gravity_torque":1.0,"friction_torque":0.5,
     "gains":{"kp":18.0,"ki":2.0,"kd":3.5,"integral_limit":5.0}},
    {"name":"j3","home":0.0,"min":-3.14,"max":3.14,"max_velocity":3.0,"max_accel":6.0,"max_torque":12.0,"inertia":0.4,"gravity_torque":0.5,"friction_torque":0.3,
     "gains":{"kp":12.0,"ki":1.0,"kd":2.5,"integral_limit":3.0}}
  ],
  "tolerance": {"position": 0.002, "velocity": 0.01, "hold_ticks": 5},
  "fault": {"max_position_error":0.2,"diverge_error":0.6,"dropout_max_ticks":3,
            "saturation_max_ticks":20,"end_grace_ticks":50,"recovery_hold_ticks":3,
            "safe_decel":6.0}
}`

// with performs literal find/replace pairs on the valid base JSON.
func with(pairs ...string) string {
	if len(pairs)%2 != 0 {
		panic("with() needs find/replace pairs")
	}
	out := validBase
	for i := 0; i < len(pairs); i += 2 {
		out = strings.ReplaceAll(out, pairs[i], pairs[i+1])
	}
	return out
}

func TestParseValid(t *testing.T) {
	cfg, err := Parse(strings.NewReader(validBase))
	if err != nil {
		t.Fatalf("valid config rejected: %v\ninput=%s", err, validBase)
	}
	if cfg.Period.String() != "2ms" {
		t.Errorf("period = %s, want 2ms", cfg.Period)
	}
	if len(cfg.Axes) != SupportedDOF {
		t.Errorf("axes = %d, want %d", len(cfg.Axes), SupportedDOF)
	}
}

func TestRejectReasons(t *testing.T) {
	cases := []struct {
		name string
		json string
		want mcerr.Code
	}{
		{"unsupported major version", with(`"version": "1.0.0"`, `"version": "2.0.0"`), mcerr.CodeConfigVersionUnsupported},
		{"old major version", with(`"version": "1.0.0"`, `"version": "0.9.1"`), mcerr.CodeConfigVersionUnsupported},
		{"unknown field", with(`"elbow": "up",`, `"elbow": "up",
  "surprise": 1,`), mcerr.CodeConfigInvalid},
		{"bad elbow", with(`"elbow": "up"`, `"elbow": "sideways"`), mcerr.CodeConfigInvalid},
		{"zero period", with(`"period": "2ms"`, `"period": "0s"`), mcerr.CodeConfigInvalid},
		{"home out of range", with(`"name":"j1","home":0.0`, `"name":"j1","home":3.5`), mcerr.CodeJointLimitExceeded},
		{"passive load >= torque", with(`"name":"j1","home":0.0,"min":-3.14,"max":3.14,"max_velocity":2.0,"max_accel":4.0,"max_torque":20.0`,
			`"name":"j1","home":0.0,"min":-3.14,"max":3.14,"max_velocity":2.0,"max_accel":4.0,"max_torque":1.5`), mcerr.CodeConfigInvalid},
		{"diverge below tracking", with(`"diverge_error":0.6`, `"diverge_error":0.1`), mcerr.CodeConfigInvalid},
		{"non-positive gain", with(`"gains":{"kp":20.0`, `"gains":{"kp":0`), mcerr.CodeConfigInvalid},
		{"empty object", `{}`, mcerr.CodeConfigInvalid},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			_, err := Parse(strings.NewReader(tc.json))
			if err == nil {
				t.Fatalf("expected %s rejection, input accepted\ninput=%s", tc.want, tc.json)
			}
			if !mcerr.HasCode(err, tc.want) {
				t.Fatalf("error code=%s want %s; err=%v\ninput=%s", errCode(err), tc.want, err, tc.json)
			}
		})
	}
}

func errCode(err error) mcerr.Code {
	var me *mcerr.Error
	if asErr(err, &me) {
		return me.Code
	}
	return ""
}

func asErr(err error, target **mcerr.Error) bool {
	for err != nil {
		if e, ok := err.(*mcerr.Error); ok {
			*target = e
			return true
		}
		type unwrapper interface{ Unwrap() error }
		u, ok := err.(unwrapper)
		if !ok {
			return false
		}
		err = u.Unwrap()
	}
	return false
}

func TestNoSilentDefaults(t *testing.T) {
	if _, err := Parse(strings.NewReader(`{}`)); err == nil {
		t.Fatal("empty config accepted; required parameters must be explicit")
	}
}
