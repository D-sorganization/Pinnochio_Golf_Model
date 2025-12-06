"""Validation tests for model correctness."""

from __future__ import annotations

from pathlib import Path


class TestModelValidation:
    """Test model correctness and physics accuracy."""

    def test_canonical_yaml_exists(self) -> None:
        """Test that canonical YAML specification exists."""
        yaml_path = Path("models/spec/golfer_canonical.yaml")
        assert yaml_path.exists(), "Canonical YAML specification not found"

    def test_yaml_structure(self) -> None:
        """Test that YAML has required structure."""
        import yaml

        yaml_path = Path("models/spec/golfer_canonical.yaml")
        if yaml_path.exists():
            with yaml_path.open() as f:
                spec = yaml.safe_load(f)
            assert "root" in spec, "Missing root segment"
            assert "segments" in spec, "Missing segments"
            assert "constraints" in spec, "Missing constraints"
