//
// Created by ruben on 3/7/26.
//

#include "kinematics.hpp"

std::shared_ptr<km::RobotPart> km::Link::clone() const {
    return std::make_shared<km::Link>(get_name(), translation, rotation);
}

void km::Joint::set_position(double position) {
    #if KM_ENFORCE_JOINT_LIMITS == 1
        // Strictly enforce limits by throwing an exception if out of range
        if (position < lim_low || position > lim_high) {
            std::ostringstream oss;
            oss << "Joint '" << get_name() << "' position " << position
                << " out of limits [" << lim_low << ", " << lim_high << "].";
            throw std::out_of_range(oss.str());
        }
    #elif KM_ENFORCE_JOINT_LIMITS == 2
        // Clamp the position to the limits and print a warning
        if (position <= lim_low) {
            // std::cerr << "Warning: Joint '" << get_name() << "' position " << position
            //           << " below lower limit " << lim_low << ". Clamping to limit." << std::endl;
            position = lim_low;
        } else if (position >= lim_high) {
            // std::cerr << "Warning: Joint '" << get_name() << "' position " << position
            //           << " above upper limit " << lim_high << ". Clamping to limit." << std::endl;
            position = lim_high;
        }
    #endif
        this->position = position;
}

void km::Joint::set_velocity(double velocity) {
    // todo safety limits
    if (this->position >= lim_high && velocity > 0) {
        std::cerr << "Warning: Joint '" << get_name() << "' at upper limit " << lim_high
                  << " with positive velocity " << velocity << ". Clamping velocity to zero." << std::endl;
        velocity = 0.0;
    } else if (this->position <= lim_low && velocity < 0) {
        std::cerr << "Warning: Joint '" << get_name() << "' at lower limit " << lim_low
                  << " with negative velocity " << velocity << ". Clamping velocity to zero." << std::endl;
        velocity = 0.0;
    }

    this->velocity = velocity;
}

void km::Joint::set_random_position() {
    static thread_local std::mt19937 rng(std::random_device{}());
    static thread_local std::uniform_real_distribution<double> dist(lim_low, lim_high);
    set_position(dist(rng));
}

Eigen::Transform<double, 3, Eigen::Isometry> km::Revolute::get_transform() const {
    Eigen::AngleAxisd rot(position, axis);
    Eigen::Transform<double, 3, Eigen::Isometry> transform = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
    transform.linear() = rot.toRotationMatrix();
    return transform;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> km::Revolute::get_jacobian_col(const Eigen::Vector3d &ee_pos) const {
    Eigen::Vector3d joint_pos = world_transform.translation();
    Eigen::Vector3d rot_axis = world_transform.linear() * axis;
    Eigen::Vector3d linear_part = rot_axis.cross(ee_pos - joint_pos);
    return {linear_part, rot_axis};
}

std::shared_ptr<km::RobotPart> km::Revolute::clone() const {
    return std::make_shared<km::Revolute>(get_name(), lim_low, lim_high);
}

void km::SequentialRobot::forward_kinematics() {
    Eigen::Transform<double, 3, Eigen::Isometry> current_transform = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
    for (const auto& part : parts) {
        current_transform = current_transform * part->get_transform();
        part->world_transform = current_transform; // Cache the world transform in the part
    }
}

void km::SequentialRobot::forward_velocity() {
    auto num_joints = static_cast<Eigen::Index>(joints.size());
    Eigen::Vector3d end_effector_pos = get_end_effector_position();
    for (Eigen::Index i = 0; i < num_joints; i++) {
        auto [linear_part, angular_part] = joints[i]->get_jacobian_col(end_effector_pos);
        jacobian.block<3, 1>(0, i) = linear_part;
        jacobian.block<3, 1>(3, i) = angular_part;
    }
}

Eigen::MatrixXd km::SequentialRobot::invert_jacobian_svd(const Eigen::MatrixXd &jacobian) {
    // Use SVD for pseudo-inverse
    // unused
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = 1e-6; // Threshold for singular values
    Eigen::VectorXd singular_values_inv = svd.singularValues();
    for (int i = 0; i < singular_values_inv.size(); i++) {
        if (singular_values_inv(i) > tolerance) {
            singular_values_inv(i) = 1.0 / singular_values_inv(i);
        } else {
            singular_values_inv(i) = 0.0; // Treat small singular values as zero
        }
    }
    return svd.matrixV() * singular_values_inv.asDiagonal() * svd.matrixU().transpose();
}

Eigen::MatrixXd km::SequentialRobot::invert_jacobian_qr(const Eigen::MatrixXd &jacobian) {
    // Use Moore-Penrose pseudo-inverse
    // unused
    return jacobian.completeOrthogonalDecomposition().pseudoInverse();
}

Eigen::VectorXd km::SequentialRobot::solve_for_joint_velocities_qr(const Eigen::MatrixXd &jacobian,
                                                                   const Eigen::VectorXd &desired_ee_velocity) {
    // Solve J * q_dot = v for q_dot using QR decomposition
    // this is the method prescribed in lectures with pseudo inverse
    // but then in form that is recommended by Eigen because its faster
    // return jacobian.colPivHouseholderQr().solve(desired_ee_velocity);
    return jacobian.completeOrthogonalDecomposition().solve(desired_ee_velocity);
}

Eigen::VectorXd km::SequentialRobot::solve_for_joint_velocities_dls(const Eigen::MatrixXd &jacobian,
    const Eigen::VectorXd &desired_ee_velocity, double lambda) {
    // Damped Least Squares solution: (J^T * J + lambda^2 * I) q_dot = J^T * v
    // apparently more stable for inverse kinematics
    const Eigen::MatrixXd& J = jacobian;
    Eigen::MatrixXd JT = J.transpose();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(J.cols(), J.cols());
    Eigen::VectorXd dq =
        (JT * J + lambda * lambda * I).ldlt().solve(JT * desired_ee_velocity);
    return dq;
}

km::SequentialRobot::SequentialRobot(std::vector<std::shared_ptr<RobotPart>> parts)
: parts(std::move(parts)) {
    for (const auto& part : this->parts) {
        if (auto joint = std::dynamic_pointer_cast<Joint>(part)) {
            joints.push_back(joint);
        }
    }
    jacobian.resize(6, joints.size());
    forward_kinematics();
    forward_velocity();
}

km::SequentialRobot::SequentialRobot(const SequentialRobot &other) :
parts(km::deep_copy_vector(other.parts)) {
    for (const auto& part : this->parts) {
        if (auto joint = std::dynamic_pointer_cast<Joint>(part)) {
            joints.push_back(joint);
        }
    }
    jacobian.resize(6, joints.size());
    forward_kinematics();
    forward_velocity();
}

km::SequentialRobot km::SequentialRobot::operator=(const SequentialRobot &other) {
    if (this == &other) {
        return *this; // Handle self-assignment
    }
    parts = km::deep_copy_vector(other.parts);
    joints.clear();
    for (const auto& part : this->parts) {
        if (auto joint = std::dynamic_pointer_cast<Joint>(part)) {
            joints.push_back(joint);
        }
    }
    jacobian.resize(6, joints.size());
    forward_kinematics();
    forward_velocity();
    return *this;
}

void km::SequentialRobot::set_joint_positions(const std::vector<double> &positions) {
    if (positions.size() != joints.size()) {
        throw std::invalid_argument("Position vector size does not match number of joints.");
    }
    for (size_t i = 0; i < joints.size(); i++) {
        joints[i]->set_position(positions[i]);
    }
    forward_kinematics();
    forward_velocity();
}

void km::SequentialRobot::set_joint_positions(const Eigen::VectorXd &positions) {
    if (positions.size() != joints.size()) {
        throw std::invalid_argument("Position vector size does not match number of joints.");
    }
    for (size_t i = 0; i < joints.size(); i++) {
        joints[i]->set_position(positions[i]);
    }
    forward_kinematics();
    forward_velocity();
}

void km::SequentialRobot::set_random_joint_positions() {
    for (const auto& joint : joints) {
        joint->set_random_position();
    }
    forward_kinematics();
    forward_velocity();
}

Eigen::VectorXd km::SequentialRobot::get_joint_positions() const {
    Eigen::VectorXd positions(joints.size());
    for (Eigen::Index i = 0; i < positions.size(); i++) {
        positions[i] = joints[i]->get_position();
    }
    return positions;
}

void km::SequentialRobot::set_joint_velocities(const std::vector<double> &velocities) {
    if (velocities.size() != joints.size()) {
        throw std::invalid_argument("Velocity vector size does not match number of joints.");
    }
    for (size_t i = 0; i < joints.size(); i++) {
        joints[i]->set_velocity(velocities[i]);
    }
}

void km::SequentialRobot::set_joint_velocities(const Eigen::VectorXd &velocities) {
    if (velocities.size() != joints.size()) {
        throw std::invalid_argument("Velocity vector size does not match number of joints.");
    }
    for (size_t i = 0; i < joints.size(); i++) {
        joints[i]->set_velocity(velocities[i]);
    }
}

Eigen::VectorXd km::SequentialRobot::get_joint_velocities() const {
    Eigen::VectorXd velocities(joints.size());
    for (Eigen::Index i = 0; i < velocities.size(); i++) {
        velocities[i] = joints[i]->get_velocity();
    }
    return velocities;
}

Eigen::Transform<double, 3, Eigen::Isometry> km::SequentialRobot::get_end_effector_transform() const {
    return parts.back()->world_transform;
}

Eigen::Vector3d km::SequentialRobot::get_end_effector_position() const {
    return get_end_effector_transform().translation();
}

Eigen::Vector3d km::SequentialRobot::get_end_effector_rotation() const {
    auto mat = get_end_effector_transform().linear();
    auto out = mat.eulerAngles(2, 1, 0);
    out = {out[2], out[1], out[0]}; // Convert from ZYX to XYZ order
    return out;
}

Eigen::Vector<double, 6> km::SequentialRobot::get_end_effector_pose() const {
    Eigen::Vector<double, 6> pose;
    pose.head<3>() = get_end_effector_position();
    pose.tail<3>() = get_end_effector_rotation();
    return pose;
}

Eigen::Vector<double, 6> km::SequentialRobot::get_end_effector_velocity() const {
    return (jacobian * get_joint_velocities()).eval();
}

Eigen::VectorXd km::SequentialRobot::required_joint_velocity(const Eigen::Vector<double, 6> &desired_ee_velocity) {
    return solve_for_joint_velocities_qr(jacobian, desired_ee_velocity);
}

Eigen::VectorXd km::SequentialRobot::required_joint_velocity_only_position(const Eigen::Vector3d &desired_ee_velocity) {
    return solve_for_joint_velocities_qr(jacobian.topRows<3>(), desired_ee_velocity);
}

Eigen::VectorXd km::SequentialRobot::required_joint_velocity_only_rotation(const Eigen::Vector3d &desired_ee_velocity) {
    return solve_for_joint_velocities_qr(jacobian.bottomRows<3>(), desired_ee_velocity);
}

std::optional<Eigen::VectorXd> km::SequentialRobot::required_joint_angles(
        const Eigen::Vector<double, 6> &desired_ee_pose,
        double learning_rate,
        double tolerance,
        int max_attempts,
        int max_iterations,
        double max_step
    ) {
    auto desired_tf = create_transform(desired_ee_pose);
    for (int i = 0; i < max_attempts; i++) {
        if (i > 0) { // if i is 0, use the current set joint angles in this robot obj
            set_random_joint_positions(); // random start state
        }
        auto joint_angles = get_joint_positions();
        for (int iter = 0; iter < max_iterations; iter++) {
            auto current_tf = get_end_effector_transform();
            // position error is simple
            auto p_err = desired_tf.translation() - current_tf.translation();
            // rotation error is a mystery
            const Eigen::Matrix3d R = current_tf.linear();
            const Eigen::Matrix3d Rd = desired_tf.linear();
            const Eigen::Matrix3d E = 0.5 * (R.transpose() * Rd - Rd.transpose() * R);
            Eigen::Vector3d r_err(E(2, 1), E(0, 2), E(1, 0)); // vee operator
            // but it works (source: some AI)
            Eigen::Vector<double, 6> err;
            err.head<3>() = p_err;
            err.tail<3>() = r_err;
            auto error = err.norm();
            if (error < tolerance) {
                return joint_angles; // Converged
            }
            // auto vel = solve_for_joint_velocities_qr(jacobian, err);
            auto dq = solve_for_joint_velocities_dls(jacobian, err, 0.05);
            // Limit the step size to prevent overshooting
            dq *= learning_rate;
            dq = dq.cwiseMax(-max_step).cwiseMin(max_step);
            set_joint_positions(joint_angles + dq);
            Eigen::VectorXd real_dq = get_joint_positions() - joint_angles; // Get the actual change after applying limits
            if (real_dq.norm() < 0.1 * error * learning_rate) {
                // if the change is too small it is likely stuck
                // the threshold scales with the error to allow smaller steps when close to the target
                // but prevent 500 iterations when it stuck far from target
                // and with lr to make it invariant to the choice of learning rate
                break;
            }
            joint_angles = get_joint_positions(); // these are with limits applied
            // std::cout << "Iteration " << i << ", " << iter << ", error norm: " << error << std::endl;
        }
    }
    return std::nullopt; // Failed to converge
}