import math
import numpy as np
from numpy.core.numeric import ones
import time

start = time.time()

np.set_printoptions(precision=4, suppress=True)

# Parameters
n_states = 3
stance_legs = ['rf', 'rh', 'lh']
width = 0.5
length = 1.0
base_position = np.array([0.0, 0.0]); # DO m(2, 1)
yaw_0 = 0.5; # rad
plot_margin = 1.5
offsets_lf = np.array([0.0, 0.0]) # DO (2, 1)
offsets_rf = np.array([0.0, 0.0]) # (2, 1)
offsets_rh = np.array([0.0, 0.0]) # (2, 1)
offsets_lh = np.array([0.0, 0.0]) # (2, 1)

# Setup
n_stance_leg = len(stance_legs)

# Nominal foot distances from base center
distances_lf = np.array([length,  width])
distances_rf = np.array([length, -width])
distances_rh = np.array([-length, -width])
distances_lh = np.array([-length,  width])
hips = ['lf', 'rf', 'rh', 'lh']
n_hips = len(hips)

# Foot positions
R_0 = np.array([[math.cos(yaw_0), -math.sin(yaw_0)], 
                [math.sin(yaw_0), math.cos(yaw_0)]])

foot_positions_lf = base_position + R_0 @ np.array(distances_lf + offsets_lf)
foot_positions_rf = base_position + R_0 @ np.array(distances_rf + offsets_rf)
foot_positions_rh = base_position + R_0 @ np.array(distances_rh + offsets_rh)
foot_positions_lh = base_position + R_0 @ np.array(distances_lh + offsets_lh)
feet = ['lf', 'rf', 'rh', 'lh']
n_feet = len(feet)

# Support polygon
support = np.zeros((n_stance_leg+1, 2))
for i, v in enumerate(stance_legs):
    if v == 'lf':
        support[i] = foot_positions_lf
    if v == 'rf':
        support[i] = foot_positions_rf
    if v == 'rh':
        support[i] = foot_positions_rh
    if v == 'lh':
        support[i] = foot_positions_lh

# Add extra point to close polygon
if stance_legs[0] == 'lf':
    support[n_stance_leg] = foot_positions_lf
if stance_legs[0] == 'rf':
    support[n_stance_leg] = foot_positions_rf
if stance_legs[0] == 'rh':
    support[n_stance_leg] = foot_positions_rh
if stance_legs[0] == 'lh':
    support[n_stance_leg] = foot_positions_lh
# print(support)
# Problem definition
# % min Ax - b, Gx <= h

# Objective
A = np.zeros((2 * n_feet, n_states))
b = np.zeros((2 * n_feet, 1))
R_star = np.array([[0, -1],
                   [1, 0]])

A[0:2] = np.c_[np.eye(2), R_0 @ R_star @ distances_lf]
t = foot_positions_lf - (R_0 @ distances_lf.T)
b[0] = t[0]
b[1] = t[1]

A[2:4] = np.c_[np.eye(2), R_0 @ R_star @ distances_rf]
t = foot_positions_rf - (R_0 @ distances_rf.T)
b[2] = t[0]
b[3] = t[1]

A[4:6] = np.c_[np.eye(2), R_0 @ R_star @ distances_rh]
t = foot_positions_rh - (R_0 @ distances_rh.T)
b[4] = t[0]
b[5] = t[1]

A[6:8] = np.c_[np.eye(2), R_0 @ R_star @ distances_lh]
t = foot_positions_lh - (R_0 @ distances_lh.T)
b[6] = t[0]
b[7] = t[1]

from pyhull.convex_hull import ConvexHull
import numpy.matlib

def vert2con(support_):
    k = ConvexHull(support_)
    k = k.vertices

    t = np.unique(k)
    c = np.mean(support_[t], axis=0)
    print((support_[0, 0] + support_[1, 0] + support_[2, 0])/3)
    print((support_[0, 0] + support_[1, 0] + support_[2, 0])/3)
    print('c', c)
    V = support_ - np.matlib.repmat(c, len(support_), 1)
    print('V', V)
    A = np.empty((len(k), len(V[0])))
    A[:] = np.nan

    rc = 0
    for i in range(len(k)):
        F = V[k[i]]
        rank = numpy.linalg.matrix_rank(F, tol=1e-5)
        if rank == len(F):
            A[rc] = np.asarray(np.linalg.solve(F, np.ones(len(F))))
            rc += 1

    A = A[:rc]
    b = np.ones(len(A))
    b = b + A @ c
    print('@', A, b)
    return A, b

# 0.05, -0.025
# -0.0166, -0.1583
# 0.00443556, 0.01776889 = 0.02220445

# -0.0388, -0.2027
# 0.00788544, 0.03157729 = 0.03946273

G, h = vert2con(support)
G = np.c_[G, np.zeros(len(G))]

# Formulation as QP
# min 1/2 x'Px + q'x + r

P = 2 * A.T @ A
q = -2 * A.T @ b
r = b.T @ b

# Solve
import quadprog

def quadprog_solve_qp(P, q, G=None, h=None, A=None, b=None):
    qp_G = .5 * (P + P.T)   # make sure P is symmetric
    qp_a = -q
    if A is not None:
        qp_C = -numpy.vstack([A, G]).T
        qp_b = -numpy.hstack([b, h])
        meq = A.shape[0]
    else:  # no equality constraint
        qp_C = -G.T
        qp_b = -h
        meq = 0
    return quadprog.solve_qp(qp_G, qp_a, qp_C, qp_b, meq)[0]

q = q.reshape(3)
solve = quadprog_solve_qp(P, q, G, h)

# R = R_0 @ np.array([[math.cos(solve[2]), math.sin(solve[2])],
#                     [-math.sin(solve[2]), math.cos(solve[2])]])

R = R_0 @ (np.eye(2) + R_star * solve[2])
T = np.array([[R[0, 0], R[0, 1], 0, solve[0]],
              [R[1, 0], R[1, 1], 0, solve[1]],
              [0,       0,       1, 0],
              [0,       0,       0, 1]])
        
b = b.reshape(len(b))
residual = A @ solve - b

# Compute hip positions
hip_positions_lf = solve[0:2] + R_0 @ (np.eye(2) + R_star * solve[2]) @ distances_lf
hip_positions_rf = solve[0:2] + R_0 @ (np.eye(2) + R_star * solve[2]) @ distances_rf
hip_positions_rh = solve[0:2] + R_0 @ (np.eye(2) + R_star * solve[2]) @ distances_rh
hip_positions_lh = solve[0:2] + R_0 @ (np.eye(2) + R_star * solve[2]) @ distances_lh

print(hip_positions_lf)
print(hip_positions_rf)
print(hip_positions_rh)
print(hip_positions_lh)

print("time :", time.time() - start)