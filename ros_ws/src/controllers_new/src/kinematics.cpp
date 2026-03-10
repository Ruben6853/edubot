//
// Created by ruben on 3/7/26.
//

#include "kinematics.hpp"

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


void km::SequentialRobot::forward_kinematics() {
    Eigen::Transform<double, 3, Eigen::Isometry> current_transform = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
    for (const auto& part : parts) {
        current_transform = current_transform * part->get_transform();
        part->world_transform = current_transform; // Cache the world transform in the part
    }
}

Eigen::VectorXd km::SequentialRobot::required_joint_velocity(const Eigen::Vector<double, 6> &desired_ee_velocity) {
    Eigen::MatrixXd jacobian = determine_jacobian();
    return solve_for_joint_velocities_qr(jacobian, desired_ee_velocity);
}

Eigen::MatrixXd km::SequentialRobot::determine_jacobian() const {
    Eigen::Index num_joints = joints.size();
    Eigen::MatrixXd jacobian(6, num_joints);
    Eigen::Vector3d end_effector_pos = get_end_effector_position();
    for (Eigen::Index i = 0; i < num_joints; i++) {
        auto [linear_part, angular_part] = joints[i]->get_jacobian_col(end_effector_pos);
        jacobian.block<3, 1>(0, i) = linear_part;
        jacobian.block<3, 1>(3, i) = angular_part;
    }
    return jacobian;
}


