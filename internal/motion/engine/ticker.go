package engine

import "time"

// ticker is the minimal time source used by RunTicked, isolated so tests
// stay on the deterministic Step path.
type ticker interface {
	C() <-chan time.Time
	Stop()
}

type wallTicker struct{ t *time.Ticker }

func (w wallTicker) C() <-chan time.Time { return w.t.C }
func (w wallTicker) Stop()               { w.t.Stop() }

func newTicker(periodSeconds float64) ticker {
	return wallTicker{t: time.NewTicker(time.Duration(periodSeconds * float64(time.Second)))}
}
