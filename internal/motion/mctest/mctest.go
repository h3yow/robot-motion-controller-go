// Package mctest provides test assertion helpers that always print the
// concrete input and the decision basis when an assertion fails.
package mctest

import (
	"fmt"
	"testing"
)

// T wraps testing.T with contextual input reporting.
type T struct{ *testing.T }

// Wrap creates a contextual test helper.
func Wrap(t *testing.T) T { return T{t} }

// CheckEq fails the test reporting input, got and want.
func (t T) CheckEq(name string, input any, got, want any) {
	t.Helper()
	if !deepEqual(got, want) {
		t.Fatalf("assertion %q failed\n  input: %#v\n  got:   %#v\n  want:  %#v", name, input, got, want)
	}
}

// CheckReason fails unless the error's reason code equals want.
func (t T) CheckReason(name string, input any, got error, want string) {
	t.Helper()
	if !deepEqual(reasonOf(got), want) {
		t.Fatalf("assertion %q failed\n  input: %#v\n  got error: %v\n  want reason: %s", name, input, got, want)
	}
}

func reasonOf(err error) string {
	if err == nil {
		return ""
	}
	return err.Error()
}

func deepEqual(a, b any) bool {
	return fmt.Sprintf("%#v", a) == fmt.Sprintf("%#v", b)
}
