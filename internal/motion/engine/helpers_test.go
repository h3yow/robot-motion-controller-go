package engine_test

import "math"

func nanE() float64   { return math.NaN() }
func absE(x float64) float64 { if x < 0 { return -x }; return x }
func finiteE(x float64) bool { return !math.IsNaN(x) && !math.IsInf(x, 0) }
