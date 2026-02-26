#include "pos_traj.hpp"

constexpr double DEG2RAD = M_PI / 180.0;

ExampleTraj::ExampleTraj() :
  rclcpp::Node("traj_ctrl")
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
    this->_pos_cmd_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_cmds", qos_pos_cmd_pub);

    rclcpp::QoS qos_joint_state_sub(rclcpp::KeepLast(1));
    qos_joint_state_sub.best_effort();
    qos_joint_state_sub.durability_volatile();
    this->_joint_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", qos_joint_state_sub, std::bind(&ExampleTraj::_joint_state_callback, this, std::placeholders::_1));

    this->_timer = this->create_wall_timer(
      10ms, std::bind(&ExampleTraj::_timer_callback, this));
}

void ExampleTraj::_joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  this->joint_pos = msg->position;
  this->joint_vel = msg->velocity;
//   std::cout << "Received joint positions: ";
//   for (size_t i = 0; i < msg->position.size(); i++)  {
//     std::cout << "Joint " << i << ": " << msg->position[i] << " ";
//   }
//   std::cout << std::endl;
//   std::cout << "Received joint velocities: ";
//   for (size_t i = 0; i < msg->velocity.size(); i++)  {
//     std::cout << "Joint " << i << ": " << msg->velocity[i] << " ";
//   }
//   std::cout << std::endl;
  if (!first_state_received) {
    RCLCPP_INFO(this->get_logger(), "First joint state received, starting trajectory execution.");
    first_state_received = true;
    _beginning = this->now();
    _traj_queue.emplace(
      joint_pos,
      home_joint_pos,
      10.0
    );
    // _traj_queue.emplace(
    //   this->home_joint_pos,
    //   std::vector<double>{0.5, 0.5, 0.5, 0.5, 0.5, 0.0},
    //   10.0
    // );
    // _traj_queue.emplace(
    //   std::vector<double>{0.5, 0.5, 0.5, 0.5, 0.5, 0.0},
    //   this->home_joint_pos,
    //   10.0
    // );
  }
}

void ExampleTraj::_timer_callback()
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
  auto& traj = _traj_queue.front();
  if (dt > traj.get_duration()) {
    RCLCPP_INFO(this->get_logger(), "Trajectory finished, moving to next trajectory.");
    _traj_queue.pop();
    _beginning = now;
    return;
  }
  // Finalize msg
  point.positions = traj.get_position(dt);
  msg.points = {point};

  // Publish
  this->_pos_cmd_publisher->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
    
  rclcpp::spin(std::make_shared<ExampleTraj>());
  rclcpp::shutdown();
  return 0;
}
