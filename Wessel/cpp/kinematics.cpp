//
// Created by ruben on 3/7/26.
//

#include <cmath>
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

Eigen::VectorXd km::SequentialRobot::clamp_to_limits(const Eigen::VectorXd& positions) const {
    Eigen::VectorXd clamped = positions;
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(joints.size()); i++) {
        clamped[i] = std::max(joints[i]->get_lim_low(),
                              std::min(joints[i]->get_lim_high(), clamped[i]));
    }
    return clamped;
}

std::optional<Eigen::VectorXd> km::SequentialRobot::solve_ik(
    const Eigen::Vector3d& target_pos,
    const std::optional<Eigen::Matrix3d>& target_rot,
    Eigen::VectorXd q0,
    int    max_iter,
    double pos_tol,
    double step)
{
    const Eigen::Index n = static_cast<Eigen::Index>(joints.size());
    if (q0.size() != n) q0 = Eigen::VectorXd::Zero(n);

    Eigen::VectorXd state = q0;

    for (int iter = 0; iter < max_iter; iter++) {
        set_joint_positions(clamp_to_limits(state));
        forward_kinematics();

        const Eigen::Vector3d pos_err = target_pos - get_end_effector_position();
        if (pos_err.norm() < pos_tol) return state;

        const Eigen::MatrixXd J = determine_jacobian();
        Eigen::VectorXd dq;

        if (target_rot.has_value()) {
            // Full 6-DOF: position + orientation error
            const Eigen::Matrix3d R_current = get_end_effector_transform().linear();
            const Eigen::Matrix3d R_err = target_rot.value() * R_current.transpose();
            const Eigen::AngleAxisd aa(R_err);
            Eigen::Vector3d rot_err = Eigen::Vector3d::Zero();
            if (aa.angle() >= 1e-9) rot_err = aa.angle() * aa.axis();

            Eigen::Matrix<double, 6, 1> error;
            error << pos_err, rot_err;
            dq = invert_jacobian_qr(J) * error;
        } else {
            // Position-only: use top 3 rows of J
            dq = invert_jacobian_qr(J.topRows(3)) * pos_err;
        }

        state = clamp_to_limits(state + step * dq);
    }

    // Final convergence check
    set_joint_positions(clamp_to_limits(state));
    forward_kinematics();
    if ((target_pos - get_end_effector_position()).norm() < pos_tol) return state;
    return std::nullopt;
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


