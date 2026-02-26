#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class ExampleTraj : public rclcpp::Node
{
private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _pos_cmd_publisher;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_state_subscriber;
    void _joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Time _beginning;
    void _timer_callback();

    std::vector<double> home;
    std::vector<double> limit;
    std::vector<double> joint_pos = {0, 0, 0, 0, 0};
    std::vector<double> joint_vel = {0, 0, 0, 0, 0};
public:
    ExampleTraj();
};

    