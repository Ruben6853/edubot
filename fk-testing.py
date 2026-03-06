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

def ik_vel_test():
    target_velocity = np.array([0.0, 0.0, 0.2, 0.0, 0.0, 0.0])
    state = np.array([0.0, 0.6, -1.3, 0.7-1.5, 0.0])
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

if __name__ == '__main__':
    # plot_state([0.0, 0.6, -1.3, 0.7, 0.0])
    # plot_workspace(actuators)
    ik_vel_test()
    ik_vel_test_dont_care_orient()
    pass