# copilot-instructions.md — GITHUB COPILOT STRICT RULES
**I am GitHub Copilot. I generate COMPLETE, TESTED, DETERMINISTIC code. NO PLACEHOLDERS. NO APPROXIMATIONS.**

--------------------------------------------------------------------------------
## MANDATORY REQUIREMENTS FOR EVERY SUGGESTION
1. **SHOW EXACT COMMANDS AND THEIR FULL OUTPUT** (never summaries)
2. **INCLUDE NEGATIVE TESTS** (what happens with bad input) 
3. **EXPLAIN EVERY NUMBER WITH SOURCE** (no magic numbers ever)
4. **IF YOU WRITE "approximately" or use placeholder, START OVER**

--------------------------------------------------------------------------------
## HARD STOPS (NON-NEGOTIABLE)
- If on `main` branch → STOP, create feature branch first
- If value unknown → STOP, never guess or approximate  
- If file >100 lines → NEVER truncate, use targeted edits
- If scientific calculation → REQUIRE source + units for every constant
- BANNED: `TODO`, `FIXME`, `...`, `pass`, `NotImplementedError`, `<placeholder>`, "approximately"

--------------------------------------------------------------------------------
## FILE PRESERVATION
- **NEVER DELETE** working code unless explicit "remove X"
- **NEVER TRUNCATE** with "rest remains the same" 
- **ALWAYS PRESERVE** all imports, functions, comments
- For large files: Make surgical edits, show exact line ranges

--------------------------------------------------------------------------------
## SCIENTIFIC COMPUTING REQUIREMENTS
```python
# EVERY constant needs citation + units (NO EXCEPTIONS):
GRAVITY_M_S2: Final[float] = 9.80665  # [m/s²] ISO 80000-3:2006 standard gravity
GOLF_BALL_MASS_KG: Final[float] = 0.04593  # [kg] USGA Rule 5-1 (1.620 oz)
GOLF_BALL_DIAMETER_M: Final[float] = 0.04267  # [m] USGA Rule 5-2 (1.680 in minimum)
AIR_DENSITY_KG_M3: Final[float] = 1.225  # [kg/m³] ISA sea level, 15°C

# NEVER write this:
velocity = distance * 3.6  # NO! What is 3.6?

# ALWAYS write this:
MPS_TO_KPH: Final[float] = 3.6  # [m/s → km/h] = (1000m/1km)/(3600s/1h)
velocity_kph = velocity_mps * MPS_TO_KPH

# EVERY calculation needs validation + error handling:
def calculate_velocity(energy_j: float, mass_kg: float) -> float:
    """Calculate velocity from kinetic energy.
    
    Formula: v = sqrt(2E/m) from E = (1/2)mv²
    
    Args:
        energy_j: Kinetic energy [J]
        mass_kg: Mass [kg]
        
    Returns:
        Velocity [m/s]
        
    Raises:
        ValueError: If energy negative or mass non-positive
    """
    if energy_j < 0:
        raise ValueError(f"Energy cannot be negative: {energy_j} J")
    if mass_kg <= 0:
        raise ValueError(f"Mass must be positive: {mass_kg} kg")
    
    return math.sqrt(2 * energy_j / mass_kg)

# TEST immediately after function:
def test_calculate_velocity():
    """Test with known values."""
    # 1 J energy, 2 kg mass → v = 1 m/s
    assert calculate_velocity(1.0, 2.0) == pytest.approx(1.0)
    
def test_calculate_velocity_invalid():
    """Test error handling."""
    with pytest.raises(ValueError, match="Energy cannot be negative"):
        calculate_velocity(-1.0, 1.0)
    with pytest.raises(ValueError, match="Mass must be positive"):
        calculate_velocity(1.0, 0.0)

# DETERMINISM IS MANDATORY (set all seeds):
np.random.seed(42)
random.seed(42)
if 'torch' in sys.modules:
    torch.manual_seed(42)
    torch.cuda.manual_seed_all(42)
```

--------------------------------------------------------------------------------
## COMPLETENESS CHECKLIST (ALL REQUIRED)
Before ANY completion, verify and show output:
- [X] Runs: `python -m py_compile <file>` → no errors
- [X] Types: All functions have type hints + returns
- [X] Docs: Docstrings with units for all functions
- [X] Errors: Try/except for all I/O and calculations  
- [X] Constants: All numbers defined with source citation
- [X] Tests: Positive + negative + edge cases exist
- [X] Quality: `python scripts/quality-check.py` → PASSED

Show this output:
```bash
$ python scripts/quality-check.py
✅ Quality check PASSED
```

--------------------------------------------------------------------------------
## CODE GENERATION PATTERNS

### 1. ALWAYS START WITH IMPORTS + CONSTANTS
```python
import logging
from pathlib import Path
from typing import Final, Literal, TypeAlias
import numpy as np

# Constants with citations
GRAVITY_M_S2: Final[float] = 9.80665  # [m/s²] ISO 80000-3:2006
RNG_SEED: Final[int] = 42  # Reproducibility

# Set determinism
np.random.seed(RNG_SEED)
logger = logging.getLogger(__name__)
```

### 2. ALWAYS VALIDATE INPUTS
```python
def process_data(file_path: str, tolerance: float = 1e-6) -> dict:
    """Process data with validation."""
    if not Path(file_path).exists():
        raise FileNotFoundError(f"File not found: {file_path}")
    if tolerance <= 0:
        raise ValueError(f"Tolerance must be positive: {tolerance}")
    
    # Process...
```

### 3. ALWAYS HANDLE ERRORS
```python
try:
    result = complex_calculation(data)
except ValueError as e:
    logger.error(f"Invalid data: {e}")
    raise
except Exception as e:
    logger.error(f"Unexpected error: {e}")
    raise RuntimeError(f"Processing failed: {e}")
```

### 4. ALWAYS TEST IMMEDIATELY
```python
def test_process_data():
    """Test data processing."""
    result = process_data("test.csv", 1e-6)
    assert "data" in result
    
def test_process_data_invalid():
    """Test error handling."""
    with pytest.raises(FileNotFoundError):
        process_data("nonexistent.csv")
```

--------------------------------------------------------------------------------
## FORBIDDEN PATTERNS (INSTANT REJECTION)

### ❌ NEVER DO THIS:
```python
def calculate():
    # TODO: implement this
    pass

def process():
    # Will implement later
    ...

def velocity():
    return distance * 3.6  # Magic number!

def gravity():
    return 9.8  # Approximately!

def test():
    # Test later
    pass
```

### ✅ ALWAYS DO THIS:
```python
def calculate_trajectory(velocity_mps: float, angle_deg: float) -> float:
    """Calculate projectile range.
    
    Formula: R = v²sin(2θ)/g from projectile motion
    
    Args:
        velocity_mps: Initial velocity [m/s]
        angle_deg: Launch angle [degrees]
        
    Returns:
        Range [m]
    """
    if velocity_mps < 0:
        raise ValueError(f"Velocity must be non-negative: {velocity_mps} m/s")
    
    angle_rad = math.radians(angle_deg)
    return (velocity_mps ** 2 * math.sin(2 * angle_rad)) / GRAVITY_M_S2

def test_calculate_trajectory():
    """Test with known values."""
    # 45° launch should give maximum range
    range_m = calculate_trajectory(10.0, 45.0)
    assert range_m > 0
    
def test_calculate_trajectory_invalid():
    """Test error handling."""
    with pytest.raises(ValueError, match="Velocity must be non-negative"):
        calculate_trajectory(-5.0, 45.0)
```

--------------------------------------------------------------------------------
## RESPONSE FORMAT (MANDATORY)

### 1. SHOW EXACT COMMANDS
```bash
$ python -m py_compile myfile.py
$ python scripts/quality-check.py
✅ Quality check PASSED
```

### 2. SHOW COMPLETE CODE (no truncation)
```python
# Complete function with all imports, validation, error handling
import math
from typing import Final

GRAVITY_M_S2: Final[float] = 9.80665  # [m/s²] ISO 80000-3:2006

def calculate_range(velocity_mps: float, angle_deg: float) -> float:
    """Calculate projectile range."""
    if velocity_mps < 0:
        raise ValueError(f"Velocity must be non-negative: {velocity_mps} m/s")
    
    angle_rad = math.radians(angle_deg)
    return (velocity_mps ** 2 * math.sin(2 * angle_rad)) / GRAVITY_M_S2
```

### 3. SHOW TESTS
```python
def test_calculate_range():
    """Test range calculation."""
    range_m = calculate_range(10.0, 45.0)
    assert range_m > 0

def test_calculate_range_invalid():
    """Test error handling."""
    with pytest.raises(ValueError, match="Velocity must be non-negative"):
        calculate_range(-5.0, 45.0)
```

### 4. VERIFY QUALITY
```bash
$ python scripts/quality-check.py
✅ Quality check PASSED
Checked 15 Python files
```

--------------------------------------------------------------------------------
## FINAL CHECKLIST
Before completing ANY task:

1. **NO PLACEHOLDERS** ✓
2. **NO MAGIC NUMBERS** ✓  
3. **ALL CONSTANTS CITED** ✓
4. **ALL FUNCTIONS TYPED** ✓
5. **ALL FUNCTIONS TESTED** ✓
6. **ALL ERRORS HANDLED** ✓
7. **QUALITY CHECK PASSES** ✓
8. **CODE COMPILES** ✓

If ANY item fails → STOP and fix before proceeding.

**I will NEVER generate incomplete code, placeholders, or approximations. Every suggestion will be production-ready with full validation and testing.**
