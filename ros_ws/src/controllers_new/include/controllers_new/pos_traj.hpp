#include <utility>

#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class JointPath {
protected:
    double duration;
public:
    virtual ~JointPath() = default;
    explicit JointPath(double duration) : duration(duration) {}
    virtual std::vector<double> get_position(double t) = 0;
    [[nodiscard]] double get_duration() const {return duration;}
};

class LinearJointPath : public JointPath {
    std::vector<double> start;
    std::vector<double> end;
public:
    LinearJointPath(std::vector<double> start, std::vector<double> end, double duration)
    : JointPath(duration), start(std::move(start)), end(std::move(end)) {}
    std::vector<double> get_position(double t) override {
        if (t > duration) {
            std::cout << "Time " << t << " exceeds duration " << duration << ", returning end position." << std::endl;
            return end;
        }
        if (t < 0) {
            std::cout << "Time " << t << " is negative, returning start position." << std::endl;
            return start;
        }
        std::vector<double> pos(start.size());
        for (size_t i = 0; i < start.size(); i++) {
            pos[i] = start[i] + (end[i] - start[i]) * (t / duration);
        }
        return pos;
    }
};

class SmoothLinearJointPath : public LinearJointPath {
public:
    SmoothLinearJointPath(std::vector<double> start, std::vector<double> end, double duration)
    : LinearJointPath(std::move(start), std::move(end), duration) {}
    std::vector<double> get_position(double t) override {
        t = (-cos((t / duration) * M_PI)*0.5 + 0.5) * duration;
        return LinearJointPath::get_position(t);
    }
};

class Controller : public rclcpp::Node
{
private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _pos_cmd_publisher;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_state_subscriber;
    bool first_state_received = false;
    void _joint_state_callback(sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Time _beginning;
    void _timer_callback();

    std::queue<std::shared_ptr<JointPath>> _traj_queue;
    std::vector<double> home_joint_pos = {0, 0, 0, 0, 0, 0};
    std::vector<double> joint_limits_high = {2, M_PI/2, M_PI/2, M_PI/2, M_PI, 2};
    std::vector<double> joint_limits_low = {-2, -M_PI/2, -M_PI/2, -M_PI/2, -M_PI, -0.2};
    std::vector<double> joint_pos;
    std::vector<double> joint_vel;
public:
    Controller();
};

    