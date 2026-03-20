#include "vel_ctrl.hpp"

constexpr double DEG2RAD = M_PI / 180.0;

Controller::Controller() :
  rclcpp::Node("traj_ctrl"), robot({
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

    // robot.set_joint_positions(std::vector<double>{1.0f, 0.0f, -1.0f, 0.0f, 0.0f});
    // auto ee_pos = robot.get_end_effector_position();
    // auto ee_att = robot.get_end_effector_rotation();
    // std::cout << "End effector position: \n"<< ee_pos << std::endl;
    // std::cout << "End effector orientation: \n " << ee_att  << std::endl;
    // auto jacobian = robot.determine_jacobian();
    // std::cout << "Jacobian:\n" << jacobian << std::endl;
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

    Eigen::Vector<double, 5> target_pos = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    double t = 5.0f; // duration of the trajectory
    auto speed = ((target_pos - robot.get_joint_positions())  / t).eval();
    publish(speed, gripper_vel);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}
