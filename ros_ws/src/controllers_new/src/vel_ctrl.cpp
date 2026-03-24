#include "vel_ctrl.hpp"

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
      5ms, std::bind(&Controller::_timer_callback, this));

    // Eigen::Vector<double, 6> desired_ee_pose = {0.0, 0.0, 0.4, 0.000, -0.785, 1.570};
    // Eigen::Vector<double, 6> desired_ee_pose = {
    //     -0.174823, 0.254109, 0.102072,
    //     -2.289353, -0.000000, 0.696802
    // };
    // auto ik_solution = dummy.required_joint_angles(desired_ee_pose);
    // if (ik_solution) {
    //     RCLCPP_INFO(this->get_logger(), "IK solution found for initial pose.");
    //     auto sol = *ik_solution;
    //     for (size_t i = 0; i < sol.size(); i++) {
    //         RCLCPP_INFO(this->get_logger(), "Joint %zu: %f, %f degrees", i, sol[i], sol[i] * 180.0 / M_PI);
    //     }
    //     std::cout << "End effector pose for IK solution: \n" << dummy.get_end_effector_pose() << std::endl;
    // } else {
    //     RCLCPP_WARN(this->get_logger(), "No IK solution found for initial pose.");
    // }
}

trajectory_msgs::msg::JointTrajectory Controller::create_msg(const Eigen::Vector<double, 5> &joint_vel,
    double gripper_vel) {
    auto now = this->now();
    auto msg = trajectory_msgs::msg::JointTrajectory();
    msg.header.stamp = now;
    auto vel = km::vector_to_std(joint_vel);
    vel.emplace_back(gripper_vel);
    auto point = trajectory_msgs::msg::JointTrajectoryPoint();
    point.velocities = vel;
    msg.points = {point};
    return msg;
}

void Controller::publish(const Eigen::Vector<double, 5> &joint_vel, double gripper_vel) {
    auto msg = create_msg(joint_vel, gripper_vel);
    this->_cmd_publisher->publish(msg);
}

void Controller::_joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    auto& joint_pos = msg->position;
    auto& joint_vel = msg->velocity;
    // following is a fix to wrong state published
    // for (size_t i = 0; i < msg->position.size(); i++) {
    //   joint_pos[i] = std::fmod(joint_pos[i], 2*M_PI);
    //   if (joint_pos[i] > 0) {
    //     joint_pos[i] = M_PI - joint_pos[i];
    //   } else {
    //     joint_pos[i] = -M_PI - joint_pos[i];
    //   }
    // }
    Eigen::Vector<double, 5> pos_ex_gripper;
    Eigen::Vector<double, 5> vel_ex_gripper;
    for (size_t i = 0; i < 5; i++) {
        pos_ex_gripper[i] = joint_pos[i];
        vel_ex_gripper[i] = joint_vel[i];
    }
     robot.set_joint_positions(pos_ex_gripper);
     robot.set_joint_velocities(vel_ex_gripper);
    gripper_pos = joint_pos[5];
    gripper_vel = joint_vel[5];

     // std::cout << "Received joint positions: ";
     // for (size_t i = 0; i < joint_pos.size(); i++)  {
     //     std::cout << "Joint " << i << ": " << joint_pos[i] << " ";
     // }
     // std::cout << std::endl;
     // std::cout << "Received joint velocities: ";
     // for (size_t i = 0; i < joint_vel.size(); i++)  {
     //     std::cout << "Joint " << i << ": " << joint_vel[i] << " ";
     // }
     // std::cout << std::endl;
    if (!first_state_received) {
        RCLCPP_INFO(this->get_logger(), "First joint state received, starting trajectory execution.");
        first_state_received = true;
        _beginning = this->now();
    }
}



void Controller::_timer_callback() {
    if (!first_state_received) {
        RCLCPP_INFO(this->get_logger(), "Waiting for first joint state...");
        return;
    }
    auto now = this->now();
    double dt = (now - this->_beginning).seconds();

    switch (current_goal) {
        case goal_type::to_home: {
            Eigen::Vector<double, 5> target_pos = goal_1;
            auto diff = (target_pos - robot.get_joint_positions()).eval();
            if (diff.norm() < 0.01) {
                auto ee_pose = robot.get_end_effector_pose();
                RCLCPP_INFO(this->get_logger(), "Current end effector pose: \n%f %f %f\n%f %f %f", ee_pose[0], ee_pose[1], ee_pose[2], ee_pose[3], ee_pose[4], ee_pose[5]);
                RCLCPP_INFO(this->get_logger(), "Target position reached, stopping.");
                publish(Eigen::Vector<double, 5>::Zero(), 0.0);
                current_goal = goal_type::line;

                break;
            }
            double t = 1.0f; // duration of the trajectory
            auto speed = (diff  / t).eval();
            publish(speed, gripper_vel);
            break;
        }
        case goal_type::line: {
            // Move in a line in the end effector space
            auto desired_ee_vel = Eigen::Vector3d(0.0, -0.01, 0.01); // move to the desired position in 5 seconds
            auto joint_vel = robot.required_joint_velocity_only_position(desired_ee_vel);
            publish(joint_vel, gripper_vel);
            break;
        }
        default: {
            break;
        }
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}
