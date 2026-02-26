#include <utility>

#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class LinearJointPath {
    std::vector<double> start;
    std::vector<double> end;
    double duration;
public:
    LinearJointPath(std::vector<double> start, std::vector<double> end, double duration)
    : start(std::move(start)), end(std::move(end)), duration(duration) {}
    std::vector<double> get_position(double t) {
        if (t > duration) {
            return end;
        }
        if (t < 0) {
            return start;
        }
        std::vector<double> pos(start.size());
        for (size_t i = 0; i < start.size(); i++) {
            pos[i] = start[i] + (end[i] - start[i]) * (t / duration);
        }
        return pos;
    }
    [[nodiscard]] double get_duration() const {return duration;}
};

class ExampleTraj : public rclcpp::Node
{
private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _pos_cmd_publisher;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_state_subscriber;
    bool first_state_received = false;
    void _joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Time _beginning;
    void _timer_callback();

    std::queue<LinearJointPath> _traj_queue;
    std::vector<double> home_joint_pos = {0, 0, 0, 0, 0, 0};
    std::vector<double> joint_limits_high = {2, M_PI/2, M_PI/2, M_PI/2, M_PI, 2};
    std::vector<double> joint_limits_low = {-2, -M_PI/2, -M_PI/2, -M_PI/2, -M_PI, -0.2};
    std::vector<double> joint_pos;
    std::vector<double> joint_vel;
public:
    ExampleTraj();
};

    