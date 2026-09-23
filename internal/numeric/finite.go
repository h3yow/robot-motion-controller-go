// Package numeric provides small deterministic floating-point helpers.
package numeric

import "math"

// Finite reports whether x is neither NaN nor +/-Inf.
func Finite(x float64) bool {
	return !math.IsNaN(x) && !math.IsInf(x, 0)
}

// AllFinite reports whether every element of xs is finite.
func AllFinite(xs []float64) bool {
	for _, x := range xs {
		if !Finite(x) {
			return false
		}
	}
	return true
}

// Clamp returns x constrained to [lo, hi].
func Clamp(x, lo, hi float64) float64 {
	if x < lo {
		return lo
	}
	if x > hi {
		return hi
	}
	return x
}
