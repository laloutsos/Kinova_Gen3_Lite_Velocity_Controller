import numpy as np
import sympy as sp

# -------------------------------
# CONFIG
# -------------------------------

DOF = 6

q1, q2, q3, q4, q5, q6 = sp.symbols('q1 q2 q3 q4 q5 q6')
spi = sp.pi

# -------------------------------
# DH PARAMETERS
# -------------------------------

DH_params = []

DH_params.append([0.2433, q1, 0, spi/2])
DH_params.append([0.030, q2 + spi/2, 0.280, spi])
DH_params.append([0.020, q3 + spi/2, 0, spi/2])
DH_params.append([0.245, q4 + spi/2, 0, spi/2])
DH_params.append([0.057, q5 + spi, 0, spi/2])
DH_params.append([0.235, q6 + spi/2, 0, 0])

# -------------------------------
# DH TRANSFORMATION MATRIX
# -------------------------------

def DH_trans_matrix(params):
    
    d, theta, a, alpha = params
    
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),  sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
        [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
        [0,              sp.sin(alpha),               sp.cos(alpha),                d],
        [0,              0,                           0,                            1]
    ])

# -------------------------------
# CUMULATIVE TRANSFORMS (OPTIMIZED)
# -------------------------------

def joint_transforms(DH_params):
    
    transforms = []
    
    T = sp.eye(4)
    transforms.append(T)  # Base frame
    
    for el in DH_params:
        T = T * DH_trans_matrix(el)
        transforms.append(T)

    # ---------------------------------
    # FIXED TOOL / GRIPPER TRANSFORM
    # ---------------------------------

    T_tool = sp.Matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0.13],   # example tool offset
        [0, 0, 0, 1]
    ])
    
    T = T * T_tool
    transforms.append(T)

    return transforms

# -------------------------------
# JACOBIAN (SYMBOLIC - ONE TIME)
# -------------------------------

def jacobian_expr(DH_params):

    transforms = joint_transforms(DH_params)

    trans_EF = transforms[-1]
    pos_EF = trans_EF[0:3, 3]

    J = sp.zeros(6, DOF)

    for joint in range(DOF):

        trans_joint = transforms[joint]

        z_axis = trans_joint[0:3, 2]
        pos_joint = trans_joint[0:3, 3]

        Jv = z_axis.cross(pos_EF - pos_joint)
        Jw = z_axis

        J[0:3, joint] = Jv
        J[3:6, joint] = Jw

    return J

# -------------------------------
# FAST NUMERIC FUNCTION (REAL-TIME)
# -------------------------------

def jacobian_numeric_func(J_sym):
    return sp.lambdify((q1, q2, q3, q4, q5, q6), J_sym, "numpy")

# -------------------------------
# INITIALIZATION (RUN ONCE)
# -------------------------------

J_sym = jacobian_expr(DH_params)
J_func = jacobian_numeric_func(J_sym)

# -------------------------------
# REAL-TIME USAGE
# -------------------------------

def compute_jacobian(joints):
    return np.array(J_func(*joints), dtype=float)

# Example usage:
joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
J = compute_jacobian(joints)
dq = [0, 0, 0.01, 0, 0, 0]
dx_pred = J @ dq
print(J)
print(dx_pred)