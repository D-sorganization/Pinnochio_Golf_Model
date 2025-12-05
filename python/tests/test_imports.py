"""Test that all modules can be imported."""
import pytest
import sys
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

def test_import_pinocchio_golf():
    """Test importing the main package."""
    import python.pinocchio_golf
    assert python.pinocchio_golf is not None

def test_import_gui():
    """Test importing the GUI module."""
    from python.pinocchio_golf import gui
    assert gui is not None

def test_import_coppelia_bridge():
    """Test importing the Coppelia bridge."""
    from python.pinocchio_golf import coppelia_bridge
    assert coppelia_bridge is not None

def test_import_torque_fitting():
    """Test importing the torque fitting module."""
    from python.pinocchio_golf import torque_fitting
    assert torque_fitting is not None
