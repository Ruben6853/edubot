"""
Task 2.3 – Circle trajectory controller using IK.

Computes IK offline for a circular EE path, then replays it as
joint-position commands at 20 Hz.

Launch with:
    ros2 launch lerobot sim_position.launch.py          # simulation
    ros2 run python_controllers ik_traj                 # this node (separate terminal)
"""

import rclpy
import numpy as np
import scipy.spatial.transform as tf_lib
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


# =============================================================================
# Robot kinematics (mirrors fk-testing.py)
# =============================================================================

_FRAMES = [
    {'parent': 'world',      'child': 'base',         'translation': np.array([0.0,     0.0,    0.0   ]), 'rotation': np.array([0.0,  0.0,       3.14159])},
    {'parent': 'base',       'child': 'shoulder',      'translation': np.array([0.0,    -0.0452, 0.0165]), 'rotation': np.array([0.0,  0.0,       0.0    ])},
    {'parent': 'shoulder',   'child': 'upper_arm',     'translation': np.array([0.0,    -0.0306, 0.1025]), 'rotation': np.array([0.0, -1.57079,   0.0    ])},
    {'parent': 'upper_arm',  'child': 'lower_arm',     'translation': np.array([0.11257,-0.028,  0.0   ]), 'rotation': np.array([0.0,  0.0,       0.0    ])},
    {'parent': 'lower_arm',  'child': 'wrist',         'translation': np.array([0.0052, -0.1349, 0.0   ]), 'rotation': np.array([0.0,  0.0,       1.57079])},
    {'parent': 'wrist',      'child': 'gripper',       'translation': np.array([-0.0601, 0.0,    0.0   ]), 'rotation': np.array([0.0, -1.57079,   0.0    ])},
    {'parent': 'gripper',    'child': 'gripper_center','translation': np.array([0.0,     0.0,    0.075 ]), 'rotation': np.array([0.0,  0.0,       0.0    ])},
]

_ACTUATORS = {
    'shoulder':  {'lim_low': -2,        'lim_high': 2       },
    'upper_arm': {'lim_low': -np.pi/2,  'lim_high': np.pi/2 },
    'lower_arm': {'lim_low': -np.pi/2,  'lim_high': np.pi/2 },
    'wrist':     {'lim_low': -np.pi/2,  'lim_high': np.pi/2 },
    'gripper':   {'lim_low': -np.pi,    'lim_high': np.pi   },
}


def _propagate(joint_angles):
    running = np.eye(4)
    transforms = {}
    j = 0
    for frame in _FRAMES:
        T = np.eye(4)
        T[:3, :3] = tf_lib.Rotation.from_euler('xyz', frame['rotation']).as_matrix()
        T[:3, 3]  = frame['translation']
        if frame['child'] in _ACTUATORS:
            R = tf_lib.Rotation.from_euler('z', joint_angles[j]).as_matrix()
            Tj = np.eye(4); Tj[:3, :3] = R
            T = T @ Tj
            j += 1
        running = running @ T
        transforms[frame['child']] = running.copy()
    return transforms


def _jacobian(joint_angles):
    transforms = _propagate(joint_angles)
    ee_pos = transforms['gripper_center'][:3, 3]
    J = np.zeros((6, len(_ACTUATORS)))
    for i, name in enumerate(_ACTUATORS):
        p = transforms[name][:3, 3]
        z = transforms[name][:3, 2]
        J[:3, i] = np.cross(z, ee_pos - p)
        J[3:, i] = z
    return J


def _ee_pos(q):
    return _propagate(q)['gripper_center'][:3, 3]


def ik(target_pos, target_rot=None, q0=None, max_iter=3000, pos_tol=1e-3, step=0.1):
    """
    Iterative Jacobian IK.  target_rot=None → position-only (orientation free).
    Returns joint angles (5,) or None if not converged.
    """
    target_pos = np.asarray(target_pos, dtype=float)
    state = np.zeros(5) if q0 is None else np.asarray(q0, dtype=float).copy()
    names = list(_ACTUATORS.keys())
    target_R = None
    if target_rot is not None:
        target_R = tf_lib.Rotation.from_euler('xyz', target_rot).as_matrix()

    for _ in range(max_iter):
        T = _propagate(state)
        pos_err = target_pos - T['gripper_center'][:3, 3]
        if np.linalg.norm(pos_err) < pos_tol:
            return state
        J = _jacobian(state)
        if target_R is not None:
            R_err = target_R @ T['gripper_center'][:3, :3].T
            rot_err = tf_lib.Rotation.from_matrix(R_err).as_rotvec()
            dq = np.linalg.pinv(J) @ np.concatenate([pos_err, rot_err])
        else:
            dq = np.linalg.pinv(J[:3, :]) @ pos_err
        state += step * dq
        for i, n in enumerate(names):
            state[i] = np.clip(state[i], _ACTUATORS[n]['lim_low'], _ACTUATORS[n]['lim_high'])

    if np.linalg.norm(target_pos - _ee_pos(state)) < pos_tol:
        return state
    return None


# =============================================================================
# Trajectory generation
# =============================================================================

def compute_circle_trajectory(center, radius, n_points, period):
    """
    Returns (times, joint_traj) for a full circle in the horizontal x-y plane.
    Uses position-only IK (orientation free).
    """
    times = np.linspace(0.0, period, n_points, endpoint=False)
    angles = 2.0 * np.pi * times / period
    waypoints = np.array([
        center + radius * np.array([np.cos(a), np.sin(a), 0.0])
        for a in angles
    ])

    joint_traj = []
    q_prev = np.array([0.0, 0.6, -1.3, 0.7, 0.0])
    failed = 0

    for i, pos in enumerate(waypoints):
        q = ik(pos, target_rot=None, q0=q_prev)
        if q is None:
            q = q_prev.copy()
            failed += 1
        joint_traj.append(q.copy())
        q_prev = q

    return times, np.array(joint_traj), failed


# =============================================================================
# ROS 2 node
# =============================================================================

class IKTrajController(Node):

    # ── Circle parameters ─────────────────────────────────────────────────
    CIRCLE_CENTER  = np.array([0.15, 0.0, 0.25])
    CIRCLE_RADIUS  = 0.07   # [m]
    CIRCLE_POINTS  = 60
    CIRCLE_PERIOD  = 12.0   # [s] – one full revolution

    TIMER_PERIOD   = 0.05   # [s] – 20 Hz publish rate

    def __init__(self):
        super().__init__('ik_traj_ctrl')

        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        self._pub = self.create_publisher(JointTrajectory, 'joint_cmds', qos_pub)
        self._sub = self.create_subscription(
            JointState, 'joint_states', self._joint_state_cb, qos_sub)

        self._timer = self.create_timer(self.TIMER_PERIOD, self._timer_cb)

        self._trajectory = None   # np.ndarray (N, 5)
        self._times      = None   # np.ndarray (N,)
        self._start_time = None
        self._first_state_received = False

        self.get_logger().info('IK trajectory controller started. Waiting for first joint state...')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _joint_state_cb(self, msg: JointState):
        if self._first_state_received:
            return  # only need the first state to warm-start IK
        if len(msg.position) < 5:
            return

        q0 = np.array(msg.position[:5])
        self._first_state_received = True
        self.get_logger().info(f'First joint state: {np.round(q0, 3)}')
        self._compute_trajectory(q0)

    def _compute_trajectory(self, q0: np.ndarray):
        self.get_logger().info('Computing IK trajectory for circle...')

        # Move first waypoint to match warm-start from current joint state
        times, traj, failed = compute_circle_trajectory(
            self.CIRCLE_CENTER,
            self.CIRCLE_RADIUS,
            self.CIRCLE_POINTS,
            self.CIRCLE_PERIOD,
        )

        if failed > 0:
            self.get_logger().warn(f'{failed}/{self.CIRCLE_POINTS} waypoints failed IK.')

        self._times      = times
        self._trajectory = traj
        self._start_time = self.get_clock().now()
        self.get_logger().info(
            f'Trajectory ready: {self.CIRCLE_POINTS} points, T={self.CIRCLE_PERIOD} s, '
            f'center={self.CIRCLE_CENTER}, r={self.CIRCLE_RADIUS} m'
        )

    def _timer_cb(self):
        if self._trajectory is None:
            return

        now = self.get_clock().now()
        elapsed = (now - self._start_time).nanoseconds * 1e-9
        t_mod = elapsed % self.CIRCLE_PERIOD  # loop forever

        # Linear interpolation between the two nearest waypoints
        idx_hi = int(np.searchsorted(self._times, t_mod))
        idx_lo = (idx_hi - 1) % len(self._times)
        idx_hi = idx_hi % len(self._times)

        t_lo = self._times[idx_lo]
        t_hi = self._times[idx_hi] if idx_hi > idx_lo else self.CIRCLE_PERIOD
        alpha = (t_mod - t_lo) / max(t_hi - t_lo, 1e-9)
        alpha = np.clip(alpha, 0.0, 1.0)

        q_interp = (1.0 - alpha) * self._trajectory[idx_lo] + alpha * self._trajectory[idx_hi % len(self._times)]

        msg = JointTrajectory()
        msg.header.stamp = now.to_msg()
        point = JointTrajectoryPoint()
        point.positions = q_interp.tolist()
        msg.points = [point]
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IKTrajController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
