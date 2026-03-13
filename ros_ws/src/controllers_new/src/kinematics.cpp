//
// Created by ruben on 3/7/26.
//

#include "kinematics.hpp"

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
            std::cerr << "Warning: Joint '" << get_name() << "' position " << position
                      << " below lower limit " << lim_low << ". Clamping to limit." << std::endl;
            position = lim_low;
        } else if (position >= lim_high) {
            std::cerr << "Warning: Joint '" << get_name() << "' position " << position
                      << " above upper limit " << lim_high << ". Clamping to limit." << std::endl;
            position = lim_high;
        }
    #endif
        this->position = position;
}

void km::Joint::set_random_position() {
    static thread_local std::mt19937 rng(std::random_device{}());
    static thread_local std::uniform_real_distribution<double> dist(lim_low, lim_high);
    set_position(dist(rng));
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> km::Revolute::get_jacobian_col(const Eigen::Vector3d &ee_pos) const {
    Eigen::Vector3d joint_pos = world_transform.translation();
    Eigen::Vector3d rot_axis = world_transform.linear() * axis;
    Eigen::Vector3d linear_part = rot_axis.cross(ee_pos - joint_pos);
    return {linear_part, rot_axis};
}

Eigen::MatrixXd km::SequentialRobot::invert_jacobian_svd(const Eigen::MatrixXd &jacobian) {
    // Use SVD for pseudo-inverse
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
    return jacobian.completeOrthogonalDecomposition().pseudoInverse();
}

Eigen::VectorXd km::SequentialRobot::solve_for_joint_velocities_qr(const Eigen::MatrixXd &jacobian,
    const Eigen::Vector<double, 6> &desired_ee_velocity) {
    // Solve J * q_dot = v for q_dot using QR decomposition
    // return jacobian.colPivHouseholderQr().solve(desired_ee_velocity);
    return jacobian.completeOrthogonalDecomposition().solve(desired_ee_velocity);
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

Eigen::VectorXd km::SequentialRobot::required_joint_velocity(const Eigen::Vector<double, 6> &desired_ee_velocity) {
    return solve_for_joint_velocities_qr(jacobian, desired_ee_velocity);
}

Eigen::VectorXd km::SequentialRobot::required_joint_angles(const Eigen::Vector<double, 6> &desired_ee_pose) {
    // random start state
    set_random_joint_positions();
    auto joint_angles = get_joint_positions();
    double learning_rate = 0.01;
    for (int iter = 0; iter < 1000; iter++) {
        set_joint_positions(joint_angles);
        forward_kinematics();
        Eigen::Vector3d ee_pos = get_end_effector_position();
    }
}



