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
    open_gripper,
    rise,
    to_midpoint_back,
};

class Controller : public rclcpp::Node
{
private:
    [[nodiscard]] trajectory_msgs::msg::JointTrajectory create_msg(const Eigen::Vector<double, 5>& joint_pos, double gripper_pos);
    void publish(const Eigen::Vector<double, 5>& joint_pos);
    void publish_current_pos();

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

    double _move_linear_until_wall_no_move_time = 0.0;
    Eigen::Vector<double, 3> _move_linear_until_wall_last_pos;
    bool move_linear_until_wall(
        Eigen::Vector<double, 6> target_ee_vel,
        double dt,
        double vel_threshold = 0.01,
        double no_move_time_threshold = 0.5
        );

    double _move_down_until_floor_no_move_time = 0.0;
    Eigen::Vector<double, 3> _move_down_until_floor_ik_last_pos;
    bool _move_down_until_floor_first_call = true;
    Eigen::Vector<double, 6> _move_down_until_floor_start_pose;
    Eigen::Vector<double, 5> _move_down_until_floor_state;

    bool move_down_until_floor(
        double vel,
        double dt,
        double vel_threshold = 0.01,
        double no_move_time_threshold = 0.5
    );

    bool open_gripper();
    double _close_gripper_no_move_time = 0.0;
    double _close_gripper_last_pos = 0.0;
    bool close_gripper(
        double vel, double dt,
        double vel_threshold = 0.1,
        double no_move_time_threshold = 0.5
        );

    goal_type current_goal = goal_type::idle;
    rclcpp::Time _goal_start_time;
    void change_goal(goal_type new_goal) {
        if (new_goal == goal_type::to_above_place) {
            place_cycle += 1;
        }
        current_goal = new_goal;
        _goal_start_time = this->now();
        RCLCPP_INFO(this->get_logger(), "Changing goal to: %d", static_cast<int>(new_goal));
    }

    // waypoints
    const double dist = 0.18f; // dist and angle are to position gripper above place point
    const double angle = -0.8f;
    Eigen::Vector<double, 5> joint_wp_home = Eigen::Vector<double, 5>::Zero();
    Eigen::Vector<double, 6> pose_wp_above_pick_up = {0.0f, dist, 0.06f, -3.1415f, -0.4f, 1.57f};
    Eigen::Vector<double, 5> joint_wp_above_pick_up; // to be calculated using ik
    Eigen::Vector<double, 6> pose_wp_above_place_end = {0.0f, dist, 0.13f, -3.1415f, -0.4f, 1.57f}; // after third cycle
    Eigen::Vector<double, 6> pose_wp_above_place_0 = {0.0f, dist, 0.06f, -3.1415f, -0.4f, 1.57f}; // for first cycle
    Eigen::Vector<double, 6> pose_wp_above_place_1 = {0.0f, dist, 0.08f, -3.1415f, -0.4f, 1.57f}; // for second cycle
    Eigen::Vector<double, 6> pose_wp_above_place_2 = {0.0f, dist, 0.10f, -3.1415f, -0.4f, 1.57f}; // for third cycle
    Eigen::Vector<double, 5> joint_wp_above_place_end;
    Eigen::Vector<double, 5> joint_wp_above_place_0;
    Eigen::Vector<double, 5> joint_wp_above_place_1;
    Eigen::Vector<double, 5> joint_wp_above_place_2;
    Eigen::Vector<double, 5> joint_wp_above_place; // current
    int place_cycle = -1; // to keep track of which above place waypoint to use
    Eigen::Vector<double, 5> joint_wp_traj_midpoint;
public:
    Controller();
};

    