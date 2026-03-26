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
    const double duration = 10.0f; // seconds
    _traj_queue.emplace(std::make_shared<SmoothLinearJointPath>(
      home_joint_pos,
      home_joint_pos,
      3.0f
    ));
    _traj_queue.emplace(std::make_shared<SmoothLinearJointPath>(
    home_joint_pos,
      std::vector<double>{
        0.64733988916,
        0.30526217522,
        -1.13361179642,
        -0.90658264098,
        2.37613622822,
        0.9
      },
      duration
    ));
    _traj_queue.emplace(std::make_shared<SmoothLinearJointPath>(
      std::vector<double>{
        0.64733988916,
        0.30526217522,
        -1.13361179642,
        -0.90658264098,
        2.37613622822,
        0.9
      },
      std::vector<double>{
        0.64733988916,
        0.30526217522,
        -1.13361179642,
        -0.90658264098,
        2.37613622822,
        0.3
      },
      duration
    ));
    _traj_queue.emplace(std::make_shared<SmoothLinearJointPath>(
      std::vector<double>{
        0.64733988916,
        0.30526217522,
        -1.13361179642,
        -0.90658264098,
        2.37613622822,
        0.3
      },
      std::vector<double>{
        -0.01227184624,
        0.53075734988,
        -0.07209709666,
        -1.65056331928,
        1.7027186658,
        0.3
      },
      duration
    ));
    _traj_queue.emplace(std::make_shared<SmoothLinearJointPath>(
      std::vector<double>{
        -0.01227184624,
        0.53075734988,
        -0.07209709666,
        -1.65056331928,
        1.7027186658,
        0.3
      },
      std::vector<double>{
        -0.8743690446,
        0.17947575126,
        -1.02930110338,
        -1.0906603345800001,
        0.9127185641,
        0.3
      },
      duration
    ));
    _traj_queue.emplace(std::make_shared<SmoothLinearJointPath>(
    std::vector<double>{
      -0.8743690446,
      0.17947575126,
      -1.02930110338,
      -1.0906603345800001,
      0.9127185641,
      0.3
      },
      std::vector<double>{
        -0.8743690446,
        0.17947575126,
        -1.02930110338,
        -1.0906603345800001,
        0.9127185641,
        0.9
      },
      duration
    ));
    _traj_queue.emplace(std::make_shared<SmoothLinearJointPath>(
    std::vector<double>{
      -0.8743690446,
      0.17947575126,
      -1.02930110338,
      -1.0906603345800001,
      0.9127185641,
      0.9
      },
      home_joint_pos,
      duration
    ));
    _beginning = this->now();
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
