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
    this->home = this->get_parameter("home").as_double_array();
    // Declare all parameters
    this->declare_parameter("limit",
      std::vector<double>{2, M_PI/2,
                          M_PI/2, M_PI/2,
                          M_PI});
    this->limit = this->get_parameter("limit").as_double_array();

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
}

void ExampleTraj::_timer_callback()
{
  auto now = this->now();
  auto msg = trajectory_msgs::msg::JointTrajectory();
  msg.header.stamp = now;
  
  double dt = (now - this->_beginning).seconds();
  auto point = trajectory_msgs::msg::JointTrajectoryPoint();
  std::vector<double> positions;
  float period = 5.0;
  uint joint_id = static_cast<long unsigned int>(dt / period) % this->home.size();

  // Push joint position
  for(uint i = 0; i < this->home.size(); i++)
  {
    if (i == joint_id) {
      double posi = this->home.at(i)
            + 1 * sin(2.0 * M_PI / period * dt)
            * this->limit.at(i);
      positions.push_back(posi);
    }
    else {
      positions.push_back(this->home.at(i));
    }
  }
  // Push gripper positions
  // positions.push_back(0.5 * sin(2 * M_PI / 10.0 * dt) + 0.5);

  // Finalize msg
  point.positions = positions;
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
