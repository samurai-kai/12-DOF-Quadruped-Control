# safety.py
import math
import numpy as np
from jacobian_numeric import jacobian_numeric


# Joint limits in DEGREES (easy to read & specify) 
HIP_MIN_DEG    = -90
HIP_MAX_DEG    = 90

THIGH_MIN_DEG  = 0
THIGH_MAX_DEG  = 360

KNEE_MIN_DEG   = 0     
KNEE_MAX_DEG   = 135

# Convert ONCE to radians for internal use
HIP_MIN,   HIP_MAX   = math.radians(HIP_MIN_DEG),   math.radians(HIP_MAX_DEG)
THIGH_MIN, THIGH_MAX = math.radians(THIGH_MIN_DEG), math.radians(THIGH_MAX_DEG)
KNEE_MIN,  KNEE_MAX  = math.radians(KNEE_MIN_DEG),  math.radians(KNEE_MAX_DEG)

# Singularity thresholds 
GEOM_THRESHOLD  = math.radians(3)      # geometric: thigh+knee straight-ish
MANIP_THRESHOLD = 0.0008               # Jacobian manipulability threshold

def within_limits(q):
    """Check joint limits. q is in radians; limits are converted from degrees."""
    h, t, k = q
    return (
        HIP_MIN   <= h <= HIP_MAX   and
        THIGH_MIN <= t <= THIGH_MAX and
        KNEE_MIN  <= k <= KNEE_MAX
    )

def geometric_singularity(q):
    """Thigh + knee ≈ 0 → straight leg → geometric singularity."""
    _, t, k = q
    return abs(t + k) < GEOM_THRESHOLD

def manipulability(J):
    """Yoshikawa manipulability measure."""
    Jv = J[0:3, :]
    JJt = Jv @ Jv.T
    det_val = np.linalg.det(JJt)
    return math.sqrt(max(det_val, 0.0))


def jacobian_singularity(q):
    J = jacobian_numeric(q)
    w = manipulability(J)
    return w < MANIP_THRESHOLD, w

def validate_command(q):
    """Main safety entry point. q is a 3-element radian joint vector."""
    
    if not within_limits(q):
        return False, "Joint limit violation"
    
    if geometric_singularity(q):
        return False, "Leg too straight (geometric singularity)"
    
    near_jac, w = jacobian_singularity(q)
    if near_jac:
        return False, f"Jacobian singularity (manipulability={w:.4f})"
    
    return True, "OK"
