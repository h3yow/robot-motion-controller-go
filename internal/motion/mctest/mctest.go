// Package mctest provides test assertion helpers that always print the
// concrete input and the decision basis when an assertion fails.
package mctest

import (
	"fmt"
	"math"
	"testing"

	"github.com/h3yow/robot-motion-controller-go/internal/motion/classify"
)

// T wraps testing.T.
type T struct{ *testing.T }

// Wrap creates a contextual test helper.
func Wrap(t *testing.T) T { return T{t} }

// fail prints the assertion name, the full input and the got/want basis.
func (t T) fail(name string, input any, format string, args ...any) {
	t.Helper()
	msg := fmt.Sprintf(format, args...)
	t.Fatalf("assertion %q failed\n  input: %#v\n  basis: %s", name, input, msg)
}

// CheckEq fails unless got == want.
func (t T) CheckEq(name string, input, got, want any) {
	t.Helper()
	if fmt.Sprintf("%#v", got) != fmt.Sprintf("%#v", want) {
		t.fail(name, input, "got %#v, want %#v", got, want)
	}
}

// CheckTrue fails unless ok.
func (t T) CheckTrue(name string, input any, ok bool, basis string) {
	t.Helper()
	if !ok {
		t.fail(name, input, "%s", basis)
	}
}

// CheckApprox fails unless |got-want| <= tol.
func (t T) CheckApprox(name string, input any, got, want, tol float64) {
	t.Helper()
	if math.IsNaN(got) || math.Abs(got-want) > tol {
		t.fail(name, input, "got %v, want %v (±%v)", got, want, tol)
	}
}

// CheckNoErr fails if err != nil.
func (t T) CheckNoErr(name string, input any, err error) {
	t.Helper()
	if err != nil {
		t.fail(name, input, "unexpected error: %v", err)
	}
}

// CheckReason fails unless err is a *RejectError whose Reason equals want.
// On failure it prints the input and the actual error/basis.
func (t T) CheckReason(name string, input any, err error, want classify.Reason) {
	t.Helper()
	r, ok := classify.AsReject(err)
	if !ok {
		t.fail(name, input, "error %v is not a classified RejectError; want reason %q", err, want)
	}
	if r.Reason != want {
		t.fail(name, input, "got reason %q (%s), want %q", r.Reason, r.Msg, want)
	}
}

// ReasonOf extracts the reason code or "" for a nil error.
func ReasonOf(err error) classify.Reason {
	if r, ok := classify.AsReject(err); ok {
		return r.Reason
	}
	return classify.ReasonOK
}
