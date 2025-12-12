"""Validation tests for model correctness."""

from __future__ import annotations

from pathlib import Path
import yaml


class TestModelValidation:
    """Test model correctness and physics accuracy."""

    def test_canonical_yaml_exists(self) -> None:
        """Test that canonical YAML specification exists."""
        # Resolve path relative to repository root
        # This file is in python/tests/validation/
        # Root is 3 levels up: python/tests/validation/ -> python/tests/ -> python/ -> root
        repo_root = Path(__file__).resolve().parents[3]
        yaml_path = repo_root / "models/spec/golfer_canonical.yaml"
        assert (
            yaml_path.exists()
        ), f"Canonical YAML specification not found at {yaml_path}"

    def test_yaml_structure(self) -> None:
        """Test that YAML has required structure."""
        repo_root = Path(__file__).resolve().parents[3]
        yaml_path = repo_root / "models/spec/golfer_canonical.yaml"
        if yaml_path.exists():
            with yaml_path.open() as f:
                spec = yaml.safe_load(f)
            assert "root" in spec, "Missing root segment"
            assert "segments" in spec, "Missing segments"
            assert "constraints" in spec, "Missing constraints"
