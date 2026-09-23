package config_test

import (
	"path/filepath"
	"testing"

	"github.com/h3yow/robot-motion-controller-go/internal/config"
)

// TestExampleConfigLoads keeps the shipped example configuration valid.
func TestExampleConfigLoads(t *testing.T) {
	path := filepath.Join("..", "..", "configs", "arm3.json")
	cfg, err := config.Load(path)
	if err != nil {
		t.Fatalf("shipped example config %s must be valid: %v", path, err)
	}
	if cfg.Elbow != "up" || len(cfg.Axes) != config.SupportedDOF || len(cfg.Links) != config.SupportedDOF {
		t.Fatalf("unexpected parsed config: elbow=%s axes=%d links=%d",
			cfg.Elbow, len(cfg.Axes), len(cfg.Links))
	}
}
