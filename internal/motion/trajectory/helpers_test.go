package trajectory_test

import (
	"math"
	"strconv"
)

func float64NaN() float64        { return math.NaN() }
func fmtSprint(x float64) string { return strconv.FormatFloat(x, 'g', -1, 64) }
