//
// Created by ruben on 3/6/26.
//
#pragma once

#include <iostream>
#include <memory>
#include <optional>
#include <Eigen/Eigen>
#include <utility>


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
        void set_joint_position(double position) {
            if (position < lim_low || position > lim_high) {
                std::ostringstream oss;
                oss << "Joint '" << get_name() << "' position " << position
                    << " out of limits [" << lim_low << ", " << lim_high << "].";
                throw std::out_of_range(oss.str());
            }
            this->position = position;
        }
        [[nodiscard]] double get_joint_position() const { return position; }
        [[nodiscard]] double get_lim_low()  const { return lim_low;  }
        [[nodiscard]] double get_lim_high() const { return lim_high; }
        [[nodiscard]] virtual std::pair<Eigen::Vector3d, Eigen::Vector3d> get_jacobian_col(const Eigen::Vector3d& ee_pos) const = 0;
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
    public:
        [[nodiscard]] Eigen::MatrixXd determine_jacobian() const;
        static Eigen::MatrixXd invert_jacobian_svd(const Eigen::MatrixXd& jacobian);
        static Eigen::MatrixXd invert_jacobian_qr(const Eigen::MatrixXd& jacobian);
        static Eigen::VectorXd solve_for_joint_velocities_qr(const Eigen::MatrixXd& jacobian, const Eigen::Vector<double, 6>& desired_ee_velocity);

        /**
         * Clamp a joint-position vector to each joint's limits.
         */
        [[nodiscard]] Eigen::VectorXd clamp_to_limits(const Eigen::VectorXd& positions) const;

        /**
         * Iterative Jacobian-based IK solver.
         *
         * @param target_pos   desired end-effector position [m]
         * @param target_rot   desired rotation matrix (nullopt → position-only)
         * @param q0           initial joint angles (empty → zero vector)
         * @param max_iter     maximum gradient steps
         * @param pos_tol      convergence threshold [m]
         * @param step         gradient step size
         * @return joint angles on success, std::nullopt if not converged
         */
        std::optional<Eigen::VectorXd> solve_ik(
            const Eigen::Vector3d& target_pos,
            const std::optional<Eigen::Matrix3d>& target_rot = std::nullopt,
            Eigen::VectorXd q0 = {},
            int    max_iter = 2000,
            double pos_tol  = 1e-3,
            double step     = 0.05);
    public:
        SequentialRobot(std::vector<std::shared_ptr<RobotPart>> parts) : parts(std::move(parts)) {
            for (const auto& part : this->parts) {
                if (auto joint = std::dynamic_pointer_cast<Joint>(part)) {
                    joints.push_back(joint);
                }
            }
        }
        void set_joint_positions(const std::vector<double>& positions) {
            if (positions.size() != joints.size()) {
                throw std::invalid_argument("Position vector size does not match number of joints.");
            }
            for (size_t i = 0; i < joints.size(); i++) {
                joints[i]->set_joint_position(positions[i]);
            }
        }
        void set_joint_positions(const Eigen::VectorXd& positions) {
            if (positions.size() != joints.size()) {
                throw std::invalid_argument("Position vector size does not match number of joints.");
            }
            for (size_t i = 0; i < joints.size(); i++) {
                joints[i]->set_joint_position(positions[i]);
            }
        }

        void forward_kinematics();

        [[nodiscard]] Eigen::Transform<double, 3, Eigen::Isometry> get_end_effector_transform() const {
            if (parts.empty()) {
                throw std::runtime_error("Robot has no parts.");
            }
            return parts.back()->world_transform;
        }
        [[nodiscard]] Eigen::Vector3d get_end_effector_position() const {
            return get_end_effector_transform().translation();
        }
        [[nodiscard]] auto get_end_effector_rotation() const {
            return get_end_effector_transform().linear().eulerAngles(2, 1, 0);
        }
        Eigen::VectorXd required_joint_velocity(const Eigen::Vector<double, 6>& desired_ee_velocity);
    };
};