"""Standalone IK test – geen plots, puur tekstoutput voor Tasks 2.1/2.2/2.3."""
import numpy as np
import scipy.spatial.transform as tf

frames = [
    {'parent': 'world',     'child': 'base',          'translation': np.array([0.0,      0.0,    0.0   ]), 'rotation': np.array([0.0,  0.0,      3.14159])},
    {'parent': 'base',      'child': 'shoulder',       'translation': np.array([0.0,     -0.0452, 0.0165]), 'rotation': np.array([0.0,  0.0,      0.0    ])},
    {'parent': 'shoulder',  'child': 'upper_arm',      'translation': np.array([0.0,     -0.0306, 0.1025]), 'rotation': np.array([0.0, -1.57079,  0.0    ])},
    {'parent': 'upper_arm', 'child': 'lower_arm',      'translation': np.array([0.11257, -0.028,  0.0   ]), 'rotation': np.array([0.0,  0.0,      0.0    ])},
    {'parent': 'lower_arm', 'child': 'wrist',          'translation': np.array([0.0052,  -0.1349, 0.0   ]), 'rotation': np.array([0.0,  0.0,      1.57079])},
    {'parent': 'wrist',     'child': 'gripper',        'translation': np.array([-0.0601,  0.0,    0.0   ]), 'rotation': np.array([0.0, -1.57079,  0.0    ])},
    {'parent': 'gripper',   'child': 'gripper_center', 'translation': np.array([0.0,      0.0,    0.075 ]), 'rotation': np.array([0.0,  0.0,      0.0    ])},
]
actuators = {
    'shoulder':  {'lim_low': -2,       'lim_high': 2       },
    'upper_arm': {'lim_low': -np.pi/2, 'lim_high': np.pi/2 },
    'lower_arm': {'lim_low': -np.pi/2, 'lim_high': np.pi/2 },
    'wrist':     {'lim_low': -np.pi/2, 'lim_high': np.pi/2 },
    'gripper':   {'lim_low': -np.pi,   'lim_high': np.pi   },
}

def propagate(q):
    running = np.eye(4)
    transforms = {}
    j = 0
    for frame in frames:
        T = np.eye(4)
        T[:3, :3] = tf.Rotation.from_euler('xyz', frame['rotation']).as_matrix()
        T[:3, 3]  = frame['translation']
        if frame['child'] in actuators:
            R = np.eye(4)
            R[:3, :3] = tf.Rotation.from_euler('z', q[j]).as_matrix()
            T = T @ R
            j += 1
        running = running @ T
        transforms[frame['child']] = running.copy()
    return transforms

def jacobian(q):
    T = propagate(q)
    ee = T['gripper_center'][:3, 3]
    J = np.zeros((6, 5))
    for i, name in enumerate(actuators):
        p = T[name][:3, 3]
        z = T[name][:3, 2]
        J[:3, i] = np.cross(z, ee - p)
        J[3:, i] = z
    return J

def ee_pos(q): return propagate(q)['gripper_center'][:3, 3]
def ee_att(q): return tf.Rotation.from_matrix(propagate(q)['gripper_center'][:3, :3]).as_euler('xyz')

def ik(target_pos, target_rot=None, q0=None, max_iter=600, pos_tol=1e-3, step=0.05):
    target_pos = np.asarray(target_pos, dtype=float)
    state = np.zeros(5) if q0 is None else np.asarray(q0, dtype=float).copy()
    names = list(actuators.keys())
    target_R = tf.Rotation.from_euler('xyz', target_rot).as_matrix() if target_rot is not None else None

    for _ in range(max_iter):
        Tmap = propagate(state)
        pos_err = target_pos - Tmap['gripper_center'][:3, 3]
        if np.linalg.norm(pos_err) < pos_tol:
            return state
        J = jacobian(state)
        if target_R is not None:
            R_err = target_R @ Tmap['gripper_center'][:3, :3].T
            rot_err = tf.Rotation.from_matrix(R_err).as_rotvec()
            dq = np.linalg.pinv(J) @ np.concatenate([pos_err, rot_err])
        else:
            dq = np.linalg.pinv(J[:3, :]) @ pos_err
        state += step * dq
        for i, n in enumerate(names):
            state[i] = np.clip(state[i], actuators[n]['lim_low'], actuators[n]['lim_high'])

    if np.linalg.norm(target_pos - ee_pos(state)) < pos_tol:
        return state
    return None

# ── Task 2.1 ─────────────────────────────────────────────────────────────────
POSES = [
    ("I",   [0.2,  0.2,    0.2  ], [0.000,  1.570,  0.650]),
    ("II",  [0.2,  0.1,    0.4  ], [0.000,  0.000, -1.570]),
    ("III", [0.0,  0.0,    0.4  ], [0.000, -0.785,  1.570]),
    ("IV",  [0.0,  0.0,    0.07 ], [3.141,  0.000,  0.000]),
    ("V",   [0.0,  0.0452, 0.45 ], [-0.785, 0.000,  3.141]),
]

print("=" * 60)
print("TASK 2.1 – IK feasibility")
print("=" * 60)
results = []
rng_21 = np.random.default_rng(0)
FIXED_STARTS = [
    np.zeros(5),
    np.array([0.0,  0.6, -1.3,  0.7,  0.0]),
    np.array([0.5, -0.3,  0.8, -0.3,  0.0]),
    np.array([-0.5, 0.3, -0.5,  0.3,  1.0]),
    np.array([1.0,  0.3, -0.8,  0.5,  0.0]),
    np.array([0.0,  0.3,  0.8,  1.3,  3.14]),  # high-z arm config
]
for label, pos, rot in POSES:
    pos = np.array(pos); rot = np.array(rot)
    q_sol = None
    for q0 in FIXED_STARTS:
        q_sol = ik(pos, rot, q0=q0)
        if q_sol is not None:
            break
    if q_sol is not None:
        err = np.linalg.norm(pos - ee_pos(q_sol))
        print(f"\nPose {label}: FEASIBLE  (pos error = {err*1000:.1f} mm)")
        print(f"  q          = {np.round(q_sol, 4)}")
        print(f"  achieved   = {np.round(ee_pos(q_sol), 4)}")
        print(f"  orient att = {np.round(ee_att(q_sol), 4)}")
        results.append((label, True, q_sol, pos, rot))
    else:
        print(f"\nPose {label}: INFEASIBLE – IK did not converge")
        results.append((label, False, None, pos, rot))

# ── Task 2.2 ─────────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("TASK 2.2 – Multiple IK solutions")
print("=" * 60)
rng = np.random.default_rng(42)
for label, feasible, _, pos, rot in results:
    if not feasible:
        print(f"\nPose {label}: skipped (infeasible)")
        continue
    pos = np.array(pos); rot = np.array(rot)
    solutions = []
    for _ in range(20):
        q0 = np.array([
            rng.uniform(-2,       2       ),
            rng.uniform(-np.pi/2, np.pi/2 ),
            rng.uniform(-np.pi/2, np.pi/2 ),
            rng.uniform(-np.pi/2, np.pi/2 ),
            rng.uniform(-np.pi,   np.pi   ),
        ])
        q = ik(pos, rot, q0=q0)
        if q is None:
            continue
        if all(np.linalg.norm(q - s) > 0.25 for s in solutions):
            solutions.append(q)
    print(f"\nPose {label}: {len(solutions)} distinct solution(s)")
    for i, q in enumerate(solutions):
        print(f"  Sol {i+1}: q = {np.round(q, 3)}")

# ── Task 2.3 ─────────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("TASK 2.3 – Circle trajectory")
print("=" * 60)

# Pick circle center = EE position at a stable config
q_stable = np.array([0.0, 0.3, -0.6, 0.3, 0.0])
center = ee_pos(q_stable)
print(f"Circle center (from FK at stable config): {np.round(center, 4)}")

radius = 0.06
n_pts  = 60
times  = np.linspace(0, 12.0, n_pts, endpoint=False)
wps    = [center + radius * np.array([np.cos(a), np.sin(a), 0.0])
          for a in 2 * np.pi * times / 12.0]

q_prev = q_stable.copy()
failed = 0
for wp in wps:
    q = ik(wp, target_rot=None, q0=q_prev)
    if q is None:
        failed += 1
        # don't update q_prev so next point starts from last good solution
    else:
        q_prev = q

print(f"Result: {n_pts - failed}/{n_pts} waypoints solved successfully")
print(f"Circle: center={np.round(center,3)}, radius={radius} m, {n_pts} points over 12 s")
