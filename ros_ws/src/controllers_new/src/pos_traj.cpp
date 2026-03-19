#include "pos_traj.hpp"

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
    })
{
    using namespace std::chrono_literals;

    // Declare all parameters
    this->declare_parameter("home",
      std::vector<double>{0, 0,
                          0, 0,
                          0});
    this->home_joint_pos = this->get_parameter("home").as_double_array();
    // Declare all parameters
    this->declare_parameter("limit",
      std::vector<double>{2, M_PI/2,
                          M_PI/2, M_PI/2,
                          M_PI});
    this->joint_limits_high = this->get_parameter("limit").as_double_array();

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

void Controller::_joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  this->joint_pos = msg->position;
  // following is a fix to wrong state published
  // for (size_t i = 0; i < msg->position.size(); i++) {
  //   joint_pos[i] = std::fmod(joint_pos[i], 2*M_PI);
  //   if (joint_pos[i] > 0) {
  //     joint_pos[i] = M_PI - joint_pos[i];
  //   } else {
  //     joint_pos[i] = -M_PI - joint_pos[i];
  //   }
  // }
  this->joint_vel = msg->velocity;
   std::cout << "Received joint positions: ";
   for (size_t i = 0; i < this->joint_pos.size(); i++)  {
     std::cout << "Joint " << i << ": " << this->joint_pos[i] << " ";
   }
   std::cout << std::endl;
   std::cout << "Received joint velocities: ";
   for (size_t i = 0; i < msg->velocity.size(); i++)  {
     std::cout << "Joint " << i << ": " << msg->velocity[i] << " ";
   }
   std::cout << std::endl;
  if (!first_state_received) {
    RCLCPP_INFO(this->get_logger(), "First joint state received, starting trajectory execution.");
    first_state_received = true;
    _beginning = this->now();
    _traj_queue.emplace(std::make_shared<SmoothLinearJointPath>(
      home_joint_pos,
      home_joint_pos,
      10.0
    ));
    _traj_queue.emplace(std::make_shared<SmoothLinearJointPath>(
    home_joint_pos,
      std::vector<double>{0.7, 0.13, -0.5, -0.35, 0.0},
      10.0
    ));
    Eigen::Vector<double, 5> home_state;
    home_state << home_joint_pos[0], home_joint_pos[1], home_joint_pos[2], home_joint_pos[3], home_joint_pos[4];
    robot.set_joint_positions(home_state);
    std::cout << "Home end effector position: \n" << robot.get_end_effector_pose() << std::endl;
    Eigen::Vector<double, 5> state;
    state << 0.7, 0.13, -0.5, -0.35, 0.0;
    robot.set_joint_positions(state);
    std::cout << "Initial end effector position: \n" << robot.get_end_effector_pose() << std::endl;
    double duration = 7.0;
    int steps = 1000;
    double dt = duration / steps;
    std::vector<Eigen::Vector3d> ee_displacements;
    ee_displacements.emplace_back(0.2, 0.0, 0.0);
    ee_displacements.emplace_back(0.0, 0.0, 0.2);
    ee_displacements.emplace_back(-0.2, 0.0, 0.0);
    ee_displacements.emplace_back(0.0, 0.0, -0.2);
    for (const auto& disp : ee_displacements) {
      auto desired_ee_displacement = disp;
      std::cout << "Desired end effector displacement: \n" << desired_ee_displacement << std::endl;
      auto desired_ee_velocity = (desired_ee_displacement / duration).eval();
      for (int i = 0; i < steps; i++) {
        try {
          robot.set_joint_positions(state);
          state = robot.get_joint_positions();
        } catch (const std::out_of_range& e) {
          RCLCPP_ERROR(this->get_logger(), "Joint position out of range: %s", e.what());
          break;
        }
        auto vel = robot.required_joint_velocity_only_position(desired_ee_velocity);
        auto next_state = (state + vel * dt).eval();
        _traj_queue.emplace(std::make_shared<LinearJointPath>(
          std::vector<double>(state.data(), state.data() + state.size()),
          std::vector<double>(next_state.data(), next_state.data() + next_state.size()),
          dt
        ));
        state = next_state;
      }
    }
    robot.set_joint_positions(state);
      std::cout << "Final end effector position: \n" << robot.get_end_effector_pose() << std::endl;
  }
}

void Controller::_timer_callback()
{
  if (!first_state_received) {
    RCLCPP_INFO(this->get_logger(), "Waiting for first joint state...");
    return;
  }
  auto now = this->now();
  auto msg = trajectory_msgs::msg::JointTrajectory();
  msg.header.stamp = now;

  double dt = (now - this->_beginning).seconds();
  auto point = trajectory_msgs::msg::JointTrajectoryPoint();
  if (_traj_queue.empty()) {
    RCLCPP_INFO(this->get_logger(), "Trajectory queue is empty, doing nothing.");
    return;
  }
  while (dt > _traj_queue.front()->get_duration()) {
    RCLCPP_INFO(this->get_logger(), "Trajectory finished, moving to next trajectory.");
    _beginning += rclcpp::Duration::from_seconds(_traj_queue.front()->get_duration());
    dt = (now - this->_beginning).seconds();
    _traj_queue.pop();
    // return;
    if (_traj_queue.empty()) {
      RCLCPP_INFO(this->get_logger(), "Trajectory queue is empty after popping, doing nothing.");
      return;
    }
  }

  auto& traj = _traj_queue.front();
  // Finalize msg
  point.positions = traj->get_position(dt);
  msg.points = {point};

  // Publish
  this->_cmd_publisher->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
    
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}
