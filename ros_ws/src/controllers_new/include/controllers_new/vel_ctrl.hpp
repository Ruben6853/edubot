#include <utility>

#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "kinematics.hpp"

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
    [[nodiscard]] trajectory_msgs::msg::JointTrajectory create_msg(const Eigen::Vector<double, 5>& joint_vel, double gripper_vel);
    void publish(const Eigen::Vector<double, 5>& joint_vel, double gripper_vel);

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _cmd_publisher;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_state_subscriber;
    bool first_state_received = false;
    void _joint_state_callback(sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Time _beginning;
    void _timer_callback();

    Eigen::Vector<double, 5> home_joint_pos = {0, 0, 0, 0, 0};

    km::SequentialRobot robot; // represents the real robot
    km::SequentialRobot dummy; // used to run simulations
    double gripper_pos;
    double gripper_vel;
public:
    Controller();
};

    