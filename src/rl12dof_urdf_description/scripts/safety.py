import math

# Script for validating joint commands to avoid singularities

# Joint limits (radians)
HIP_MIN   = -1.5
HIP_MAX   =  1.5

THIGH_MIN = -1.4
THIGH_MAX =  1.4

# Prevent full extension (singularity)
KNEE_MIN  = 0.10          # ~6 deg bent, avoids straight singularity
KNEE_MAX  = 2.50

SINGULARITY_THRESHOLD = math.radians(3)  # 3° from straight

def within_joint_limits(cmd):
    hip, thigh, knee = cmd
    if not (HIP_MIN <= hip <= HIP_MAX): return False
    if not (THIGH_MIN <= thigh <= THIGH_MAX): return False
    if not (KNEE_MIN <= knee <= KNEE_MAX): return False
    return True


def is_near_singularity(cmd):
    """
    Detect standard quadruped leg extension singularity.
    Assumes joint order: [hip, thigh, knee].
    """
    _, thigh, knee = cmd
    return abs(thigh + knee) < SINGULARITY_THRESHOLD


def validate_command(cmd):
    """
    Returns (valid, message)
    """
    if not within_joint_limits(cmd):
        return False, "Command violates joint limits"
    if is_near_singularity(cmd):
        return False, "Command approaches singularity"
    return True, "OK"
