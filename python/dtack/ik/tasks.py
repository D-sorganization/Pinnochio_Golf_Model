"""Task definitions for Pink IK."""

from __future__ import annotations

from typing import Union

import numpy as np
import pinocchio as pin
import pink.tasks
from pink import AbstractTask

# Type alias for transformation matrices
Transform = Union[pin.SE3, np.ndarray]


def create_frame_task(
    body_name: str, 
    position_cost: float = 1.0, 
    orientation_cost: float = 1.0, 
    lm_damping: float = 0.0
) -> pink.tasks.FrameTask:
    """Create a FrameTask (SE3 target) for a specific body frame.
    
    Args:
        body_name: Name of the frame in the model
        position_cost: Weight for position error
        orientation_cost: Weight for orientation error
        lm_damping: Levenberg-Marquardt damping
        
    Returns:
        Configured FrameTask
    """
    task = pink.tasks.FrameTask(body_name, position_cost=position_cost, orientation_cost=orientation_cost)
    task.lm_damping = lm_damping
    return task


def create_posture_task(
    cost: float = 1e-3, 
    q_ref: np.ndarray = None
) -> pink.tasks.PostureTask:
    """Create a PostureTask to regularize joint configuration.
    
    Args:
        cost: Weight for the posture regularization
        q_ref: Reference configuration (target). If None, must be set later.
        
    Returns:
        Configured PostureTask
    """
    task = pink.tasks.PostureTask(cost=cost)
    if q_ref is not None:
        task.set_target(q_ref)
    return task


def create_joint_coupling_task(
    joint_names: list[str],
    ratios: list[float],
    cost: float = 100.0
) -> pink.tasks.LinearHolonomicTask:
    """Create a task to enforce linear coupling between joints.
    
    Useful for anatomical constraints (e.g. Scapula moving with Shoulder).
    Equation: sum(ratio_i * q_i) = constant (or 0)
    
    Args:
        joint_names: List of joint names involved
        ratios: Coefficients for each joint
        cost: Task weight
        
    Returns:
        LinearHolonomicTask (Note: Requires building matrix A and vector b)
    """
    # This is a placeholder. Pink's LinearHolonomicTask takes A, b in A * q = b
    # Implementation depends on how we want to construct 'A' from names.
    # Typically A is shape (k, nq).
    # For now, we return usage instructions or a base implementation if feasible.
    raise RuntimeError("Joint coupling task requires mapping names to joint indices dynamically.")
