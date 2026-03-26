#include "comp.hpp"

constexpr double DEG2RAD = M_PI / 180.0;

Controller::Controller() :
  rclcpp::Node("vel_ctrl"), robot({
    KMLINK("base", 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, M_PI),
    KMLINK("shoulder_base", 0.0f, -0.0452f, 0.0165f, 0.0f, 0.0f, 0.0f),
    KMREV("shoulder_yaw", -2.0f, 2.0f),
    KMLINK("shoulder_link", 0.0f, -0.0306f, 0.1025f, 0.0f, -M_PI/2, 0.0f),
    KMREV("shoulder_pitch", -M_PI/2, M_PI/2),
    KMLINK("upper_arm", 0.11257f, -0.028f, 0.0f, 0.0f, 0.0f, 0.0f),
    KMREV("elbow", -M_PI/2, M_PI/2),
    KMLINK("lower_arm", 0.0052f, -0.1349f, 0.0f, 0.0f, 0.0f, M_PI/2),
    KMREV("wrist_pitch", -M_PI/2, M_PI/2),
    KMLINK("wrist_link", -0.0601f, 0.0f, 0.0f, 0.0f, -M_PI/2, 0.0f),
    KMREV("wrist_roll", -M_PI, M_PI),
    KMLINK("gripper_center", 0.0f, 0.0f, 0.075f, 0.0f, 0.0f, 0.0f)
    }), dummy(robot)
{
    using namespace std::chrono_literals;
    this->_beginning = this->now();
    this->_last_t  = this->_beginning;
    this->_last_joint_state_callback_time = this->_beginning;

    // Make QoS for publisher
    rclcpp::QoS qos_pos_cmd_pub(rclcpp::KeepLast(1));
    qos_pos_cmd_pub.reliable();
    qos_pos_cmd_pub.durability_volatile();
    this->_cmd_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_cmds", qos_pos_cmd_pub);

    rclcpp::QoS qos_joint_state_sub(rclcpp::KeepLast(1));
    qos_joint_state_sub.best_effort();
    qos_joint_state_sub.durability_volatile();
    this->_joint_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", qos_joint_state_sub, std::bind(&Controller::_joint_state_callback, this, std::placeholders::_1));

    this->_timer = this->create_wall_timer(
      100ms, std::bind(&Controller::_timer_callback, this));

    auto ik_sol_pick_up = dummy.required_joint_angles(pose_wp_above_pick_up);
    if (ik_sol_pick_up) {
        joint_wp_above_pick_up = ik_sol_pick_up->head<5>();
        joint_wp_above_pick_up[0] -= this->angle;
        joint_wp_traj_midpoint = ik_sol_pick_up->head<5>();
        RCLCPP_INFO(this->get_logger(), "Calculated joint waypoint for above pick up position.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to calculate joint waypoint for above pick up position. Check if the pose is reachable.");
    }
    auto ik_sol_place = dummy.required_joint_angles(pose_wp_above_place);
    if (ik_sol_place) {
        joint_wp_above_place = ik_sol_place->head<5>();
        joint_wp_above_place[0] += this->angle;
        RCLCPP_INFO(this->get_logger(), "Calculated joint waypoint for above place position.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to calculate joint waypoint for above place position. Check if the pose is reachable.");
    }
}

trajectory_msgs::msg::JointTrajectory Controller::create_msg(const Eigen::Vector<double, 5> &joint_pos,
    double gripper_pos) {
    auto now = this->now();
    auto msg = trajectory_msgs::msg::JointTrajectory();
    msg.header.stamp = now;
    auto pos = km::vector_to_std(joint_pos);
    pos.emplace_back(gripper_pos);
    auto point = trajectory_msgs::msg::JointTrajectoryPoint();
    point.positions = pos;
    msg.points = {point};
    return msg;
}

void Controller::publish(const Eigen::Vector<double, 5> &joint_pos) {
    auto msg = create_msg(joint_pos, target_gripper_pos);
    this->_cmd_publisher->publish(msg);
}

void Controller::_joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {

    auto now = this->now();
    double dt = (now - _last_joint_state_callback_time).seconds();
    _last_joint_state_callback_time = now;

    auto& joint_pos = msg->position;
    Eigen::Vector<double, 5> pos_ex_gripper;
    for (size_t i = 0; i < 5; i++) {
        pos_ex_gripper[i] = joint_pos[i];
    }
    robot.set_joint_velocities((pos_ex_gripper - robot.get_joint_positions()) / dt);
    robot.set_joint_positions(pos_ex_gripper);
    gripper_vel = (joint_pos[5] - gripper_pos) / dt;
    gripper_pos = joint_pos[5];

    //  std::cout << "Received joint positions: ";
    //  for (size_t i = 0; i < joint_pos.size(); i++)  {
    //      std::cout << "Joint " << i << ": " << joint_pos[i] << " ";
    //  }
    //  std::cout << std::endl;
    //  std::cout << "Inferred joint velocities: ";
    // auto joint_vel = robot.get_joint_velocities();
    //  for (size_t i = 0; i < joint_vel.size(); i++)  {
    //      std::cout << "Joint " << i << ": " << joint_vel[i] << " ";
    //  }
    //  std::cout << "gripper velocity: " << gripper_vel << std::endl;
    if (!first_state_received) {
        RCLCPP_INFO(this->get_logger(), "First joint state received, starting trajectory execution.");
        first_state_received = true;
        current_goal = goal_type::to_home;
        _beginning = this->now();
    }
}



void Controller::_timer_callback() {
    if (!first_state_received) {
        RCLCPP_INFO(this->get_logger(), "Waiting for first joint state...");
        return;
    }
    auto now = this->now();
    double dt = (now - _last_t).seconds();
    _last_t = now;

    // settings
    const double approach_speed = 0.01f;
    const double midpoint_tol = 0.1f;

    switch (current_goal) {
        case goal_type::idle:
            break;
        case goal_type::to_home:
            target_gripper_pos = 1.0f;
            if (move_j(joint_wp_traj_midpoint, midpoint_tol)) {
                current_goal = goal_type::to_above_pick_up;
                std::cout << "Reached home position, moving to above pick up position." << std::endl;
                std::cout << "Pose: " << robot.get_end_effector_pose().transpose() << std::endl;
            }
            break;
        case goal_type::to_above_pick_up:
            target_gripper_pos = 1.0f;
            if (move_j(joint_wp_above_pick_up, 0.05f)) {
                current_goal = goal_type::pick_up;
                    std::cout << "Reached above pick up position, picking up." << std::endl;
            }
            break;
        case goal_type::pick_up:
            target_gripper_pos = 1.0f;
            if (move_linear_until_wall({0.0f, 0.0f, -approach_speed, 0.0f, 0.0f, 0.0f}, dt)) {;
                current_goal = goal_type::close_gripper;
                std::cout << "Reached pick up position, closing gripper." << std::endl;
            }
            break;
        case goal_type::close_gripper:
            if (close_gripper(0.5f, dt, 0.001f, 0.3f)) {
                current_goal = goal_type::to_midpoint;
                std::cout << "Gripper closed, moving to above place position." << std::endl;
            }
            break;
        case goal_type::to_midpoint:
            if (move_j(joint_wp_traj_midpoint, midpoint_tol)) {
                current_goal = goal_type::to_above_place;
                std::cout << "Reached trajectory midpoint, moving to above place position." << std::endl;
            }
            break;
        case goal_type::to_above_place:
            if (move_j(joint_wp_above_place, 0.05f)) {
                current_goal = goal_type::place;
                std::cout << "Reached above place position, moving down to place." << std::endl;
            }
            break;
        case goal_type::place:
            if (move_linear_until_wall({0.0f, 0.0f, -approach_speed, 0.0f, 0.0f, 0.0f}, dt)) {;
                current_goal = goal_type::open_gripper;
                std::cout << "Reached place position, opening gripper." << std::endl;
            }
            break;
        case goal_type::open_gripper:
            if (open_gripper()) {
                current_goal = goal_type::to_home;
                std::cout << "Gripper opened, task complete." << std::endl;
            }
            break;
    }
}

bool Controller::move_j(Eigen::Vector<double, 5> target_joint_pos, double tol) {
    auto diff = (target_joint_pos - robot.get_joint_positions()).eval();
    if (diff.norm() < tol) {
        RCLCPP_INFO(this->get_logger(), "move_j Target position reached");
        return true;
    }
    publish(target_joint_pos);
    return false;
}

bool Controller::move_linear_until_wall(
    Eigen::Vector<double, 6> target_ee_vel,
    double dt,
    double collision_threshold
    ) {
    if (_move_linear_until_wall_last_pos_valid) {
        auto current_ee_pos = robot.get_end_effector_position();
        Eigen::Matrix<double, 3, 1> vel = (current_ee_pos - _move_linear_until_wall_last_pos) / dt;
        _move_linear_until_wall_last_pos = current_ee_pos;
        if (vel.norm() < collision_threshold) {
            RCLCPP_INFO(this->get_logger(), "No significant end effector movement detected, likely in contact with wall. Stopping.");
            return true;
        }
        if (robot.get_end_effector_position()[2] < 0.0f) {
            RCLCPP_INFO(this->get_logger(), "End effector too close to the ground, stopping to prevent damage.");
            return true;
        }
    } else {
        _move_linear_until_wall_last_pos = robot.get_end_effector_position();
        _move_linear_until_wall_last_pos_valid = true;
    }
    auto joint_vel = robot.required_joint_velocity(target_ee_vel);
    publish(
        robot.get_joint_positions() + joint_vel * dt
        );
    return false;
}

bool Controller::open_gripper() {
    target_gripper_pos = 1.0f;
    publish(robot.get_joint_positions());
    return true;
}

bool Controller::close_gripper(double vel, double dt, double vel_threshold, double no_move_time_threshold) {
    // if (std::abs(gripper_pos - target_pos) < 0.01f) {
    //     RCLCPP_INFO(this->get_logger(), "Gripper closed to target position.");
    //     return true;
    // }
    // if (std::abs(gripper_vel) < vel_threshold) {
    //     _close_gripper_no_move_time += dt;
    //     if (_close_gripper_no_move_time > no_move_time_threshold) {
    //         RCLCPP_INFO(this->get_logger(), "Gripper velocity below threshold for 0.2 seconds, likely in contact with object. Stopping.");
    //         _close_gripper_no_move_time = 0.0f;
    //         target_gripper_pos = gripper_pos - 0.01f; // apply a small closing command to maintain force
    //         publish(robot.get_joint_positions());
    //         return true;
    //     }
    // } else {
    //     _close_gripper_no_move_time = 0.0f;
    // }
    // target_gripper_pos = std::max(target_gripper_pos - vel * dt, 0.0);
    // publish(robot.get_joint_positions());
    // return false;
    target_gripper_pos = 0.0f;
    publish(robot.get_joint_positions());
    return true;
}


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}
