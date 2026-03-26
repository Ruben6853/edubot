#include <utility>

#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "kinematics.hpp"


enum class goal_type {
    idle,
    to_home,
    to_above_pick_up,
    pick_up,
    close_gripper,
    to_midpoint,
    to_above_place,
    place,
    open_gripper
};

class Controller : public rclcpp::Node
{
private:
    [[nodiscard]] trajectory_msgs::msg::JointTrajectory create_msg(const Eigen::Vector<double, 5>& joint_pos, double gripper_pos);
    void publish(const Eigen::Vector<double, 5>& joint_pos);

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _cmd_publisher;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_state_subscriber;
    bool first_state_received = false;
    rclcpp::Time _last_joint_state_callback_time;
    void _joint_state_callback(sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Time _beginning;
    void _timer_callback();
    rclcpp::Time _last_t;

    km::SequentialRobot robot; // represents the real robot
    km::SequentialRobot dummy; // used to run simulations
    double gripper_pos;
    double gripper_vel;

    double target_gripper_pos = 0.0f; // to manage the gripper separate

    // Returns true if the goal is reached (within tol), false otherwise
    bool move_j(Eigen::Vector<double, 5> target_joint_pos, double tol=0.05);

    Eigen::Vector<double, 3> _move_linear_until_wall_last_pos;
    bool _move_linear_until_wall_last_pos_valid = false;
    bool move_linear_until_wall(
        Eigen::Vector<double, 6> target_ee_vel,
        double dt,
        double collision_threshold = 0.001
        );
    bool open_gripper();
    double _close_gripper_no_move_time = 0.0;
    bool close_gripper(
        double vel, double dt,
        double vel_threshold = 0.001,
        double no_move_time_threshold = 0.3
        );

    goal_type current_goal = goal_type::idle;

    // waypoints
    const double dist = 0.2f; // dist and angle are to position gripper above place point
    const double angle = 0.77f;
    Eigen::Vector<double, 5> joint_wp_home = Eigen::Vector<double, 5>::Zero();
    Eigen::Vector<double, 6> pose_wp_above_pick_up = {0.0f, dist, 0.07f, -3.1415f, -0.4f, 1.57f};
    Eigen::Vector<double, 5> joint_wp_above_pick_up; // to be calculated using ik
    Eigen::Vector<double, 6> pose_wp_above_place = {0.0f, dist, 0.15f, -3.1415f, -0.4f, 1.57f};
    Eigen::Vector<double, 5> joint_wp_above_place;
    Eigen::Vector<double, 5> joint_wp_traj_midpoint;
public:
    Controller();
};

    