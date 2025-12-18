# jacobian_numeric.py
import numpy as np
import sympy as sp

# ---- Symbolic variables ----
theta1, theta2, theta3 = sp.symbols('theta1 theta2 theta3')
L1, L2, L3, L4 = sp.symbols('L1 L2 L3 L4')

# ---- DH Transform ----
def dh_transform_sym(theta, d, a, alpha):
    ct, st = sp.cos(theta), sp.sin(theta)
    ca, sa = sp.cos(alpha), sp.sin(alpha)
    return sp.Matrix([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [ 0,     sa,     ca,   d],
        [ 0,      0,      0,   1]
    ])

def frame_tf(Tn, Tm):
    return Tm @ Tn

def output_z_p(T):
    z = T[0:3, 2]
    p = T[0:3, 3]
    return z, p

def jacobian_sym(z_list, p_list, p_e):
    Jp = sp.Matrix.hstack(*[z_list[i].cross(p_e - p_list[i]) for i in range(len(z_list))])
    Jo = sp.Matrix.hstack(*[z_list[i] for i in range(len(z_list))])
    return sp.Matrix.vstack(Jp, Jo)

# ---- Build symbolic Jacobian once ----

T01 = dh_transform_sym(np.pi/2, 0, 0, np.pi/2)
T12 = dh_transform_sym(theta1 - sp.pi/2, L1, 0, -sp.pi/2)
T23 = dh_transform_sym(theta2 - sp.pi/2, L2, L3, 0)
T3e = dh_transform_sym(theta3, 0, L4, 0)

T02 = frame_tf(T01, T12)
T03 = frame_tf(T02, T23)
T0e = frame_tf(T03, T3e)

z1, p1 = output_z_p(T01)
z2, p2 = output_z_p(T02)
z3, p3 = output_z_p(T03)
_, pe = output_z_p(T0e)

J_sym = jacobian_sym([z1, z2, z3], [p1, p2, p3], pe)

# ---- Lambdify: turns symbolic J into FAST numeric function ----
J_func = sp.lambdify(
    (theta1, theta2, theta3, L1, L2, L3, L4),
    J_sym,
    "numpy"
)

# ---- Robot link lengths (meters) ----
L1_VAL = 0.061
L2_VAL = 0.083
L3_VAL = 0.146
L4_VAL = 0.165

def jacobian_numeric(q):
    """Return 6x3 numpy Jacobian for joint angles q=[hip, thigh, knee]."""
    q1, q2, q3 = q
    J = J_func(q1, q2, q3, L1_VAL, L2_VAL, L3_VAL, L4_VAL)
    return np.array(J, dtype=float)
