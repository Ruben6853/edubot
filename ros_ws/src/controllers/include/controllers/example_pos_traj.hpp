#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class Controller : public rclcpp::Node
{
private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _pos_cmd_publisher;
    rclcpp::TimerBase::SharedPtr _timer;

    rclcpp::Time _beginning;
    void _timer_callback();

    std::vector<double> home_joint_pos;

public:
    Controller();
};

    