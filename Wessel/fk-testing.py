import numpy as np
import scipy.spatial.transform as tf
from matplotlib import pyplot as plt


def create_homogeneous_transform(translation, rotation):
    # Create a homogeneous transformation matrix from translation and rotation
    homogenous_transform = np.eye(4)
    homogenous_transform[:3, :3] = tf.Rotation.from_euler('xyz', rotation).as_matrix()
    homogenous_transform[:3, 3] = translation
    return homogenous_transform


frames = [
    {'parent': 'world', 'child': 'base', 'translation': np.array([0.0, 0.0, 0.0]), 'rotation': np.array([0.0, 0.0, 3.14159])},
    {'parent': 'base', 'child': 'shoulder', 'translation': np.array([0.0, -0.0452, 0.0165]), 'rotation': np.array([0.0, 0.0, 0.0])},
    {'parent': 'shoulder', 'child': 'upper_arm', 'translation': np.array([0.0, -0.0306, 0.1025]), 'rotation': np.array([0.0, -1.57079, 0.0])},
    {'parent': 'upper_arm', 'child': 'lower_arm', 'translation': np.array([0.11257, -0.028, 0.0]), 'rotation': np.array([0.0, 0.0, 0.0])},
    {'parent': 'lower_arm', 'child': 'wrist', 'translation': np.array([0.0052, -0.1349, 0.0]), 'rotation': np.array([0.0, 0.0, 1.57079])},
    {'parent': 'wrist', 'child': 'gripper', 'translation': np.array([-0.0601, 0.0, 0.0]), 'rotation': np.array([0.0, -1.57079, 0.0])},
    {'parent': 'gripper', 'child': 'gripper_center', 'translation': np.array([0.0, 0.0, 0.075]), 'rotation': np.array([0.0, 0.0, 0.0])},
    # {'parent': 'gripper', 'child': 'jaw', 'translation': np.array([-0.0202, 0.0, 0.0244]), 'rotation': np.array([1.57079, 3.14158, 0.0])}
]

# transforms the joints apply in what coordinate frame
actuators = {
    'shoulder':  {'axis': 'z', 'lim_low': -2, 'lim_high': 2},
    'upper_arm': {'axis': 'z', 'lim_low': -np.pi/2, 'lim_high': np.pi/2},
    'lower_arm': {'axis': 'z', 'lim_low': -np.pi/2, 'lim_high': np.pi/2},
    'wrist':     {'axis': 'z', 'lim_low': -np.pi/2, 'lim_high': np.pi/2},
    'gripper':   {'axis': 'z', 'lim_low': -np.pi, 'lim_high': np.pi},
    # 'jaw':       {'axis': 'z', 'lim_low': -0.2, 'lim_high': 2},
}

actuators_no_lim = {
    'shoulder':  {'axis': 'z', 'lim_low': -np.pi, 'lim_high': np.pi},
    'upper_arm': {'axis': 'z', 'lim_low': -np.pi, 'lim_high': np.pi},
    'lower_arm': {'axis': 'z', 'lim_low': -np.pi, 'lim_high': np.pi},
    'wrist':     {'axis': 'z', 'lim_low': -np.pi, 'lim_high': np.pi},
    'gripper':   {'axis': 'z', 'lim_low': -np.pi, 'lim_high': np.pi},
    # 'jaw':       {'axis': 'z', 'lim_low': -0.2, 'lim_high': 2},
}

actuators_for_ws = {
    'shoulder':  {'axis': 'z', 'lim_low': -2, 'lim_high': 2},
    'upper_arm': {'axis': 'z', 'lim_low': -np.pi/2, 'lim_high': np.pi/2},
    'lower_arm': {'axis': 'z', 'lim_low': np.pi/2, 'lim_high': np.pi/2}, # straight with upper arm
    'wrist':     {'axis': 'z', 'lim_low': 0, 'lim_high': 0}, # zero to keep stretched out
    'gripper':   {'axis': 'z', 'lim_low': 0, 'lim_high': 0}, # dont care
    # 'jaw':       {'axis': 'z', 'lim_low': 0, 'lim_high': 0}, # dont care
}


def propagate_transform(frames, actuators, joint_angles):
    running_transform = np.eye(4)
    joint_id = 0
    transforms = {}
    for frame in frames:
        transforms[frame['child']] = {}
        local_transform_rot = tf.Rotation.from_euler('xyz', frame['rotation']).as_matrix()
        local_transform = np.eye(4)
        local_transform[:3, :3] = local_transform_rot
        local_transform[:3, 3] = frame['translation']
        if frame['child'] in actuators.keys():
            actuator_transform_rot = tf.Rotation.from_euler('xyz', [0.0, 0.0, joint_angles[joint_id]]).as_matrix()
            actuator_transform = np.eye(4)
            actuator_transform[:3, :3] = actuator_transform_rot
            local_transform = local_transform @ actuator_transform
            joint_id += 1
        transforms[frame['child']][frame['parent']] = local_transform
        running_transform = running_transform @ local_transform
        transforms[frame['child']]['world'] = running_transform
    return transforms

def create_jacobian(frames, actuators, joint_angles):
    transforms = propagate_transform(frames, actuators, joint_angles)
    end_effector_pos = transforms['gripper_center']['world'][:3, 3]
    jacobian = np.zeros((6, len(actuators)))
    for i, (actuator_name, actuator_data) in enumerate(actuators.items()):
        # find the frame associated with this actuator
        for frame in frames:
            if frame['child'] == actuator_name:
                joint_frame = frame
                break
        else:
            continue  # skip if no frame found for this actuator
        joint_pos = transforms[joint_frame['child']]['world'][:3, 3]
        joint_axis = transforms[joint_frame['child']]['world'][:3, 2]
        # compute the contribution of this joint to the end effector velocity
        linear_contribution = np.cross(joint_axis, end_effector_pos - joint_pos)
        angular_contribution = joint_axis
        jacobian[:3, i] = linear_contribution
        jacobian[3:, i] = angular_contribution
    return jacobian

def best_inverse(jacobian):
    try:
        return np.linalg.inv(jacobian)
    except np.linalg.LinAlgError:
        return np.linalg.pinv(jacobian)

def plot_state(joint_state) -> None:
    transforms = propagate_transform(
        frames, actuators,
        joint_state)
    xs, ys, zs = [], [], []
    for key, transform in transforms.items():
        origin = transform['world'][:3, 3]
        xs.append(origin[0])
        ys.append(origin[1])
        zs.append(origin[2])
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(xs, ys, zs, marker='o', linestyle='-')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    # set box aspect to equal-like scaling
    ax.set_box_aspect([np.ptp(xs) or 1, np.ptp(ys) or 1, np.ptp(zs) or 1])
    plt.show()
    pass

def plot_workspace(actuators):
    n=1000
    random_states = []
    for name, data in actuators.items():
        random_states.append(np.random.uniform(data['lim_low'], data['lim_high'], n))
    random_states = np.array(random_states)
    end_points = np.zeros((3, n))
    for i in range (n):
        transforms = propagate_transform(
            frames, actuators, random_states[:, i]
        )
        end_points[:, i] = transforms['gripper_center']['world'][:3, 3]
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(end_points[0, :], end_points[1, :], end_points[2, :], marker='o')
    plt.show()
    pass

def ee_pos(state):
    transforms = propagate_transform(frames, actuators, state)
    return transforms['gripper_center']['world'][:3, 3]

def ee_att(state):
    transforms = propagate_transform(frames, actuators, state)
    return tf.Rotation.from_matrix(transforms['gripper_center']['world'][:3, :3]).as_euler('xyz')

def ik_vel_test():
    target_velocity = np.array([0.0, -0.2, 0.2, 0.0, 0.0, 0.0])
    state = np.array([0.0, 0.6, -1.3, 0.7, 0.0])
    print(ee_pos(state))
    plot_state(state)
    total_time = 1.0
    n_steps = 1000
    for i in range(n_steps):
        jacobian = create_jacobian(frames, actuators, state)
        jacobian_inverse = best_inverse(jacobian)
        joint_velocities = jacobian_inverse @ target_velocity
        state += joint_velocities * (total_time / n_steps)
    print(ee_pos(state))
    plot_state(state)

def ik_vel_test_dont_care_orient():
    target_velocity = np.array([0.2, 0.0, 0.0, 0.0, 0.0, 0.0])
    state = np.array([0.0, 0.6, -1.3, 0.7, 0.0])
    print(ee_pos(state))
    plot_state(state)
    total_time = 1.0
    n_steps = 1000
    for i in range(n_steps):
        jacobian = create_jacobian(frames, actuators, state)
        jacobian_pos = jacobian[:3, :]
        jacobian_inverse = best_inverse(jacobian_pos)
        joint_velocities = jacobian_inverse @ target_velocity[:3]
        state += joint_velocities * (total_time / n_steps)
    print(ee_pos(state))
    plot_state(state)

def fk_test():
    state = np.array([1.0, 0.0, -1.0, 0.0, 0.0])

    print(ee_pos(state))
    print(ee_att(state))
    jac = create_jacobian(frames, actuators, state)
    plot_state(state)


# =============================================================================
# IK solver (iterative Jacobian / gradient descent)
# =============================================================================

def ik(target_pos, target_rot=None, q0=None, max_iter=5000, pos_tol=1e-3, step=0.05):
    """
    Iterative Jacobian-based inverse kinematics.

    Parameters
    ----------
    target_pos : array-like (3,)   desired end-effector position [x, y, z]
    target_rot : array-like (3,) or None
        Desired XYZ Euler angles.  Pass None for position-only IK, which
        leaves orientation free (better conditioned for Task 2.3).
    q0         : array-like (5,) or None   initial joint angles (default: zeros)
    max_iter   : int    maximum number of gradient steps
    pos_tol    : float  convergence threshold on position error [m]
    step       : float  gradient-descent step size

    Returns
    -------
    joint angles (5,) on success, or None if the solver did not converge.
    """
    target_pos = np.asarray(target_pos, dtype=float)
    state = np.zeros(5) if q0 is None else np.asarray(q0, dtype=float).copy()

    target_R = None
    if target_rot is not None:
        target_R = tf.Rotation.from_euler('xyz', target_rot).as_matrix()

    joint_names = list(actuators.keys())

    for _ in range(max_iter):
        transforms = propagate_transform(frames, actuators, state)
        current_T = transforms['gripper_center']['world']
        current_pos = current_T[:3, 3]

        pos_error = target_pos - current_pos
        if np.linalg.norm(pos_error) < pos_tol:
            return state

        J = create_jacobian(frames, actuators, state)

        if target_R is not None:
            # Full 6-DOF error: position + orientation (via rotation vector)
            current_R = current_T[:3, :3]
            R_err = target_R @ current_R.T
            rot_error = tf.Rotation.from_matrix(R_err).as_rotvec()
            error = np.concatenate([pos_error, rot_error])
            dq = np.linalg.pinv(J) @ error
        else:
            # Position-only: use only the top 3 rows of J
            dq = np.linalg.pinv(J[:3, :]) @ pos_error

        state += step * dq

        # Clamp to joint limits
        for i, name in enumerate(joint_names):
            state[i] = np.clip(state[i], actuators[name]['lim_low'], actuators[name]['lim_high'])

    # One final position check (solver may have converged on the last step)
    transforms = propagate_transform(frames, actuators, state)
    if np.linalg.norm(target_pos - transforms['gripper_center']['world'][:3, 3]) < pos_tol:
        return state
    return None


# =============================================================================
# Task 2.1 – IK feasibility for the five assignment poses
# =============================================================================

POSES_2_1 = [
    # (label,  [x,   y,    z  ], [rx,    ry,     rz   ])
    ("I",   [0.2,  0.2,  0.2 ], [0.000,  1.570,  0.650]),
    ("II",  [0.2,  0.1,  0.4 ], [0.000,  0.000, -1.570]),
    ("III", [0.0,  0.0,  0.4 ], [0.000, -0.785,  1.570]),
    ("IV",  [0.0,  0.0,  0.07], [3.141,  0.000,  0.000]),
    ("V",   [0.0,  0.0452, 0.45], [-0.785, 0.000, 3.141]),
]


def task_2_1(show_plots=True):
    """
    Solve IK for every pose in POSES_2_1.
    Prints a feasibility summary and plots the robot for feasible poses.

    Returns a list of (label, feasible, q_solution) tuples.
    """
    print("\n" + "="*60)
    print("TASK 2.1 – IK feasibility check")
    print("="*60)

    rng_21 = np.random.default_rng(seed=0)
    results = []
    for label, pos, rot in POSES_2_1:
        pos = np.array(pos)
        rot = np.array(rot)

        # Deterministic starts + random restarts for robustness
        fixed_starts = [
            np.zeros(5),
            np.array([0.0,  0.6, -1.3,  0.7,  0.0]),
            np.array([0.5, -0.3,  0.8, -0.3,  0.0]),
            np.array([-0.5, 0.3, -0.5,  0.3,  1.0]),
            np.array([0.0,  0.3,  0.8,  1.3,  3.14]),  # high-z arm config
            np.array([0.0, -0.3,  0.8, -1.3,  0.0]),
        ]
        random_starts = [
            np.array([
                rng_21.uniform(-2,       2       ),
                rng_21.uniform(-np.pi/2, np.pi/2 ),
                rng_21.uniform(-np.pi/2, np.pi/2 ),
                rng_21.uniform(-np.pi/2, np.pi/2 ),
                rng_21.uniform(-np.pi,   np.pi   ),
            ])
            for _ in range(30)
        ]
        q_sol = None
        for q0 in fixed_starts + random_starts:
            q_sol = ik(pos, rot, q0=q0)
            if q_sol is not None:
                break

        if q_sol is not None:
            achieved_pos = ee_pos(q_sol)
            achieved_rot = ee_att(q_sol)
            pos_err = np.linalg.norm(pos - achieved_pos)
            print(f"\nPose {label}: FEASIBLE  (pos error = {pos_err*1000:.1f} mm)")
            print(f"  Target   pos: {pos},  rot: {rot}")
            print(f"  Achieved pos: {np.round(achieved_pos,4)},  rot: {np.round(achieved_rot,4)}")
            print(f"  q = {np.round(q_sol, 4)}")
            results.append((label, True, q_sol, pos, rot))
        else:
            print(f"\nPose {label}: INFEASIBLE – IK did not converge")
            print(f"  Target pos: {pos},  rot: {rot}")
            results.append((label, False, None, pos, rot))

    if show_plots:
        feasible = [(lbl, q, pos) for lbl, ok, q, pos, rot in results if ok]
        if feasible:
            n = len(feasible)
            fig = plt.figure(figsize=(5 * n, 5))
            fig.suptitle("Task 2.1 – Feasible IK solutions", fontsize=14)
            for idx, (lbl, q, target_pos) in enumerate(feasible):
                ax = fig.add_subplot(1, n, idx + 1, projection='3d')
                _plot_robot_on_ax(ax, q)
                ax.scatter(*target_pos, color='red', s=100, zorder=5, label='Target')
                ax.set_title(f"Pose {lbl}")
                ax.legend()
            plt.tight_layout()
            plt.show()

    return results


# =============================================================================
# Task 2.2 – Multiple IK solutions (elbow-up / elbow-down etc.)
# =============================================================================

def task_2_2(task_2_1_results=None, n_starts=30, show_plots=True):
    """
    Find multiple distinct IK solutions for the feasible poses from Task 2.1.
    Prints the solutions and shows a side-by-side plot.

    Parameters
    ----------
    task_2_1_results : output of task_2_1(), or None (runs task_2_1 automatically)
    n_starts         : number of random start configurations tried per pose
    """
    if task_2_1_results is None:
        task_2_1_results = task_2_1(show_plots=False)

    print("\n" + "="*60)
    print("TASK 2.2 – Multiple IK solutions")
    print("="*60)

    rng = np.random.default_rng(seed=42)

    for label, feasible, _, pos, rot in task_2_1_results:
        if not feasible:
            print(f"\nPose {label}: skipped (infeasible in Task 2.1)")
            continue

        pos = np.array(pos)
        rot = np.array(rot)
        solutions = []

        for _ in range(n_starts):
            q0 = np.array([
                rng.uniform(-2,       2      ),
                rng.uniform(-np.pi/2, np.pi/2),
                rng.uniform(-np.pi/2, np.pi/2),
                rng.uniform(-np.pi/2, np.pi/2),
                rng.uniform(-np.pi,   np.pi  ),
            ])
            q = ik(pos, rot, q0=q0)
            if q is None:
                continue
            # Keep only sufficiently distinct solutions (> 0.25 rad apart)
            if all(np.linalg.norm(q - s) > 0.25 for s in solutions):
                solutions.append(q)

        print(f"\nPose {label}: {len(solutions)} distinct solution(s) found")
        for i, q in enumerate(solutions):
            print(f"  Solution {i+1}: q = {np.round(q, 3)}")

        if show_plots and solutions:
            n = len(solutions)
            fig = plt.figure(figsize=(5 * n, 5))
            fig.suptitle(f"Task 2.2 – Multiple solutions for Pose {label}", fontsize=14)
            for idx, q in enumerate(solutions):
                ax = fig.add_subplot(1, n, idx + 1, projection='3d')
                _plot_robot_on_ax(ax, q)
                ax.scatter(*pos, color='red', s=100, zorder=5, label='Target')
                ax.set_title(f"Solution {idx + 1}\nq={np.round(q, 2)}")
                ax.legend(fontsize=7)
            plt.tight_layout()
            plt.show()


# =============================================================================
# Task 2.3 – Shape trajectory (circle in horizontal plane)
# =============================================================================

def task_2_3(show_plots=True):
    """
    Command the end-effector to trace a circle in the horizontal (x-y) plane.

    Returns
    -------
    times            : np.ndarray (N,)  time stamps [s]
    joint_trajectory : np.ndarray (N,5) joint angles for each waypoint
    """
    print("\n" + "="*60)
    print("TASK 2.3 – Circle trajectory")
    print("="*60)

    # Circle parameters – center computed from FK at a stable configuration
    # so the entire circle lies well inside the workspace.
    q_stable = np.array([0.0, 0.3, -0.6, 0.3, 0.0])
    center = ee_pos(q_stable)          # ≈ [0, 0.335, 0.200]
    radius = 0.06        # [m]
    n_points = 60
    T = 12.0             # period [s]

    # Trace in the Y-Z plane (vertical circle in front of the robot):
    #   x = center[0]
    #   y = center[1] + r*cos(a)
    #   z = center[2] + r*sin(a)
    times = np.linspace(0, T, n_points, endpoint=False)
    angles_traj = 2 * np.pi * times / T
    waypoints = np.array([
        center + radius * np.array([0.0, np.cos(a), np.sin(a)])
        for a in angles_traj
    ])

    print(f"Circle center (FK): {np.round(center, 3)}")
    print(f"Circle: center={np.round(center,3)}, radius={radius} m, {n_points} points, T={T} s")
    print("Solving IK for each waypoint (position-only, orientation free)...")

    joint_trajectory = []
    q_prev = q_stable.copy()  # warm-start from the stable config

    failed = 0
    for i, pos in enumerate(waypoints):
        q = ik(pos, target_rot=None, q0=q_prev)  # position-only IK
        if q is None:
            print(f"  WARNING: IK failed at waypoint {i} pos={np.round(pos,3)} – using previous")
            q = q_prev.copy()
            failed += 1
        joint_trajectory.append(q.copy())
        q_prev = q

    joint_trajectory = np.array(joint_trajectory)
    print(f"Done. {n_points - failed}/{n_points} waypoints solved successfully.")

    if show_plots:
        achieved = np.array([ee_pos(q) for q in joint_trajectory])

        fig = plt.figure(figsize=(14, 5))
        fig.suptitle("Task 2.3 – Circle Trajectory", fontsize=14)

        # 3D trajectory
        ax1 = fig.add_subplot(131, projection='3d')
        ax1.plot(achieved[:, 0], achieved[:, 1], achieved[:, 2], 'b-o', markersize=3, label='EE path')
        ax1.scatter(*center, color='red', s=80, label='Center')
        ax1.set_xlabel('X'); ax1.set_ylabel('Y'); ax1.set_zlabel('Z')
        ax1.set_title('EE Trajectory (3D)')
        ax1.legend()

        # Front view (y-z) – the plane in which the circle is traced
        ax2 = fig.add_subplot(132)
        ax2.plot(achieved[:, 1], achieved[:, 2], 'b-o', markersize=4)
        ax2.scatter(center[1], center[2], color='red', s=80, label='Center')
        ax2.set_xlabel('Y [m]'); ax2.set_ylabel('Z [m]')
        ax2.set_title('Front View (Y-Z plane)')
        ax2.set_aspect('equal')
        ax2.legend()
        ax2.grid(True)

        # Joint angles over time
        ax3 = fig.add_subplot(133)
        for i in range(5):
            ax3.plot(times, joint_trajectory[:, i], label=f'q{i+1}')
        ax3.set_xlabel('Time [s]')
        ax3.set_ylabel('Joint angle [rad]')
        ax3.set_title('Joint Trajectories')
        ax3.legend()
        ax3.grid(True)

        plt.tight_layout()
        plt.show()

    return times, joint_trajectory


# =============================================================================
# Helper: plot robot skeleton on existing 3D axes
# =============================================================================

def _plot_robot_on_ax(ax, joint_state):
    transforms = propagate_transform(frames, actuators, joint_state)
    xs, ys, zs = [], [], []
    for key, transform in transforms.items():
        origin = transform['world'][:3, 3]
        xs.append(origin[0])
        ys.append(origin[1])
        zs.append(origin[2])
    ax.plot(xs, ys, zs, marker='o', linestyle='-', color='steelblue')
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    span = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs), 0.01)
    mid_x, mid_y, mid_z = np.mean(xs), np.mean(ys), np.mean(zs)
    ax.set_xlim(mid_x - span/2, mid_x + span/2)
    ax.set_ylim(mid_y - span/2, mid_y + span/2)
    ax.set_zlim(mid_z - span/2, mid_z + span/2)


if __name__ == '__main__':
    # ── Uncomment the task you want to run ──────────────────────────────────

    # Original tests
    # plot_state([0.0, 0.6, -1.3, 0.7, 0.0])
    # plot_workspace(actuators)
    # ik_vel_test()
    # ik_vel_test_dont_care_orient()
    # fk_test()

    # Assignment tasks
    results = task_2_1()
    task_2_2(results)
    task_2_3()
    pass