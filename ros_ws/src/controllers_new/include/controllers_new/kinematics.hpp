//
// Created by ruben on 3/6/26.
//
#pragma once

#include <iostream>
#include <memory>
#include <Eigen/Eigen>
#include <utility>
#include <random>

#define KM_ENFORCE_JOINT_LIMITS 0 // 0: no enforcement, 1: strict with exceptions, 2: clamp with warnings


namespace km {
    inline Eigen::Matrix3d create_xyz_rotation_matrix(const Eigen::Vector3d& euler_angles) {
        const Eigen::AngleAxisd x_rot(euler_angles[0], Eigen::Vector3d::UnitX());
        const Eigen::AngleAxisd y_rot(euler_angles[1], Eigen::Vector3d::UnitY());
        const Eigen::AngleAxisd z_rot(euler_angles[2], Eigen::Vector3d::UnitZ());

        const Eigen::Quaternion<double> quat = z_rot * y_rot * x_rot;
        return quat.toRotationMatrix();
    }

    inline Eigen::Transform<double, 3, Eigen::Isometry> create_transform(const Eigen::Vector3d& translation, const Eigen::Vector3d& rotation) {
        Eigen::Matrix3d rotation_matrix = create_xyz_rotation_matrix(rotation);
        Eigen::Transform<double, 3, Eigen::Isometry> transform;
        transform.linear() = rotation_matrix;
        transform.translation() = translation;
        return transform;
    }

    /**
     * Base class for all robot parts.
     * This class hierarchy allows polymorphism in the final robot class.
     */
    class RobotPart {
        std::string name;
    public:
        Eigen::Transform<double, 3, Eigen::Isometry> world_transform; // Cache for world transform
        // used by the robot class, but its stored here the transform is associated with the part.
        RobotPart(std::string name) : name(std::move(name)) {}
        virtual ~RobotPart() = default;
        [[nodiscard]] virtual Eigen::Transform<double, 3, Eigen::Isometry> get_transform() const = 0;
        [[nodiscard]] std::string get_name() const { return name; }
    };

    /**
     * Represents a fixed link in the robot.
     */
    class Link : public RobotPart {
        Eigen::Vector3d translation;
        Eigen::Vector3d rotation;
        Eigen::Transform<double, 3, Eigen::Isometry> transform;
    public:
        Link(std::string name, Eigen::Vector3d translation, Eigen::Vector3d rotation)
        : RobotPart(std::move(name)), translation(std::move(translation)), rotation(std::move(rotation)) {
            transform = create_transform(translation, rotation);
        }
        [[nodiscard]] Eigen::Transform<double, 3, Eigen::Isometry> get_transform() const override {
            return transform;
        }
    };

// for convenience
#define KMLINK(name, tx, ty, tz, rx, ry, rz) \
    std::make_shared<km::Link>(km::Link(name, {tx, ty, tz}, {rx, ry, rz}))


    /**
     * Base joint class.
     * Then there will be Revolute and Prismatic subclass.
     * They share Position and limit fields.
     */
    class Joint: public RobotPart {
    protected:
        double lim_low;
        double lim_high;
        double position = 0.0;
    public:
        Joint(std::string name, double lim_low, double lim_high) : RobotPart(std::move(name)),
        lim_low(lim_low), lim_high(lim_high) {}
        void set_position(double position);
        [[nodiscard]] double get_position() const {
            return position;
        }
        [[nodiscard]] virtual std::pair<Eigen::Vector3d, Eigen::Vector3d> get_jacobian_col(const Eigen::Vector3d& ee_pos) const = 0;

        void set_random_position();
    };

    class Revolute: public Joint {
        Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
    public:
        Revolute(std::string name, double lim_low, double lim_high) : Joint(std::move(name), lim_low, lim_high){}
        [[nodiscard]] Eigen::Transform<double, 3, Eigen::Isometry> get_transform() const override {
            Eigen::AngleAxisd rot(position, axis);
            Eigen::Transform<double, 3, Eigen::Isometry> transform = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
            transform.linear() = rot.toRotationMatrix();
            return transform;
        }
        [[nodiscard]] std::pair<Eigen::Vector3d, Eigen::Vector3d> get_jacobian_col(const Eigen::Vector3d& ee_pos) const override;
    };

#define KMREV(name, lim_low, lim_high) \
    std::make_shared<km::Revolute>(km::Revolute(name, lim_low, lim_high))

    // todo Prismatic joint

    class SequentialRobot {
        std::vector<std::shared_ptr<RobotPart>> parts;
        std::vector<std::shared_ptr<Joint>> joints;
        Eigen::MatrixXd jacobian;
        void forward_kinematics();
        void forward_velocity();
        static Eigen::MatrixXd invert_jacobian_svd(const Eigen::MatrixXd& jacobian);
        static Eigen::MatrixXd invert_jacobian_qr(const Eigen::MatrixXd& jacobian);
        static Eigen::VectorXd solve_for_joint_velocities_qr(const Eigen::MatrixXd& jacobian, const Eigen::VectorXd &desired_ee_velocity);
    public:
        SequentialRobot(std::vector<std::shared_ptr<RobotPart>> parts) : parts(std::move(parts)) {
            for (const auto& part : this->parts) {
                if (auto joint = std::dynamic_pointer_cast<Joint>(part)) {
                    joints.push_back(joint);
                }
            }
            jacobian.resize(6, joints.size());
            forward_kinematics();
            forward_velocity();
        }
        void set_joint_positions(const std::vector<double>& positions);
        void set_joint_positions(const Eigen::VectorXd& positions);
        void set_random_joint_positions();
        [[nodiscard]] Eigen::VectorXd get_joint_positions() const;


        [[nodiscard]] Eigen::Transform<double, 3, Eigen::Isometry> get_end_effector_transform() const {
            if (parts.empty()) {
                throw std::runtime_error("Robot has no parts.");
            }
            return parts.back()->world_transform;
        }
        [[nodiscard]] Eigen::Vector3d get_end_effector_position() const {
            return get_end_effector_transform().translation();
        }
        [[nodiscard]] Eigen::Vector3d get_end_effector_rotation() const;
        [[nodiscard]] Eigen::Vector<double, 6> get_end_effector_pose() const {
            Eigen::Vector<double, 6> pose;
            pose.head<3>() = get_end_effector_position();
            pose.tail<3>() = get_end_effector_rotation();
            return pose;
        }
        Eigen::VectorXd required_joint_velocity(const Eigen::Vector<double, 6>& desired_ee_velocity);
        Eigen::VectorXd required_joint_velocity_only_position(const Eigen::Vector3d& desired_ee_velocity);
        Eigen::VectorXd required_joint_velocity_only_rotation(const Eigen::Vector3d& desired_ee_velocity);
        Eigen::VectorXd required_joint_angles(const Eigen::Vector<double, 6> &desired_ee_pose);
    };
};