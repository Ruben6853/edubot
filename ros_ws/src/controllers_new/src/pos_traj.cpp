#include "pos_traj.hpp"
#include <cmath>

constexpr double DEG2RAD = M_PI / 180.0;

// ── Circle trajectory parameters ─────────────────────────────────────────────
constexpr int    CIRCLE_N      = 60;     // waypoints
constexpr double CIRCLE_RADIUS = 0.06;   // [m]
constexpr double CIRCLE_PERIOD = 12.0;   // [s] one full revolution

// ── Task 2.1 poses [x, y, z] and XYZ Euler [rx, ry, rz] ─────────────────────
struct Pose21 {
    const char*       label;
    Eigen::Vector3d   pos;
    Eigen::Vector3d   euler_xyz;
};

static const Pose21 POSES_21[] = {
    {"I",  {0.2,    0.2,    0.2  }, {0.000,  1.570,  0.650}},
    {"II", {0.2,    0.1,    0.4  }, {0.000,  0.000, -1.570}},
    {"V",  {0.0,    0.0452, 0.45 }, {-0.785, 0.000,  3.141}},
};

// ─────────────────────────────────────────────────────────────────────────────

Controller::Controller() :
  rclcpp::Node("traj_ctrl"), robot({
    KMLINK("base",          0.0f,    0.0f,    0.0f,   0.0f,    0.0f,     M_PI   ),
    KMLINK("shoulder_base", 0.0f,   -0.0452f, 0.0165f,0.0f,    0.0f,     0.0f   ),
    KMREV ("shoulder_yaw", -2.0f,   2.0f                                         ),
    KMLINK("shoulder_link", 0.0f,   -0.0306f, 0.1025f,0.0f,   -M_PI/2,   0.0f   ),
    KMREV ("shoulder_pitch",-M_PI/2, M_PI/2                                      ),
    KMLINK("upper_arm",     0.11257f,-0.028f,  0.0f,   0.0f,    0.0f,     0.0f   ),
    KMREV ("elbow",        -M_PI/2,  M_PI/2                                      ),
    KMLINK("lower_arm",     0.0052f,-0.1349f,  0.0f,   0.0f,    0.0f,     M_PI/2 ),
    KMREV ("wrist_pitch",  -M_PI/2,  M_PI/2                                      ),
    KMLINK("wrist_link",   -0.0601f,  0.0f,    0.0f,   0.0f,   -M_PI/2,   0.0f   ),
    KMREV ("wrist_roll",   -M_PI,    M_PI                                         ),
    KMLINK("gripper_center", 0.0f,    0.0f,    0.075f, 0.0f,    0.0f,     0.0f   ),
  })
{
    using namespace std::chrono_literals;

    this->declare_parameter("home",
      std::vector<double>{0, 0, 0, 0, 0});
    this->home_joint_pos = this->get_parameter("home").as_double_array();

    this->declare_parameter("limit",
      std::vector<double>{2, M_PI/2, M_PI/2, M_PI/2, M_PI});
    this->joint_limits_high = this->get_parameter("limit").as_double_array();

    this->_beginning = this->now();

    rclcpp::QoS qos_pub(rclcpp::KeepLast(1));
    qos_pub.reliable();
    qos_pub.durability_volatile();
    this->_pos_cmd_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "joint_cmds", qos_pub);

    rclcpp::QoS qos_sub(rclcpp::KeepLast(1));
    qos_sub.best_effort();
    qos_sub.durability_volatile();
    this->_joint_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", qos_sub,
        std::bind(&Controller::_joint_state_callback, this, std::placeholders::_1));

    this->_timer = this->create_wall_timer(
        100ms, std::bind(&Controller::_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Controller ready, waiting for first joint state...");
}

// ── First joint-state callback: build the full trajectory queue ───────────────
void Controller::_joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    this->joint_pos = msg->position;
    this->joint_vel = msg->velocity;

    if (first_state_received) return;

    first_state_received = true;
    RCLCPP_INFO(this->get_logger(), "First joint state received — building trajectory.");

    // Convenience: home as Eigen vector
    Eigen::VectorXd q_home = Eigen::Map<const Eigen::VectorXd>(
        home_joint_pos.data(), home_joint_pos.size());

    // ── 0. Move to home ───────────────────────────────────────────────────────
    _traj_queue.emplace(std::make_shared<SmoothLinearJointPath>(
        joint_pos, home_joint_pos, 5.0));

    // ── 1. Task 2.1: visit each feasible IK pose ──────────────────────────────
    RCLCPP_INFO(this->get_logger(), "Solving IK for Task 2.1 poses...");

    std::vector<double> q_last = home_joint_pos;

    for (const auto& pose : POSES_21) {
        Eigen::Matrix3d R = km::create_xyz_rotation_matrix(pose.euler_xyz);
        Eigen::VectorXd q0 = Eigen::Map<const Eigen::VectorXd>(q_last.data(), q_last.size());

        // Try multiple starting configs for robustness
        std::optional<Eigen::VectorXd> q_sol;
        const std::vector<Eigen::VectorXd> starts = {
            q0,
            Eigen::VectorXd::Zero(5),
            (Eigen::VectorXd(5) <<  0.0,  0.6, -1.3,  0.7,  0.0).finished(),
            (Eigen::VectorXd(5) <<  0.5, -0.3,  0.8, -0.3,  0.0).finished(),
            (Eigen::VectorXd(5) <<  0.0,  0.3,  0.8,  1.3,  3.14).finished(),
        };
        for (const auto& s : starts) {
            q_sol = robot.solve_ik(pose.pos, R, s);
            if (q_sol.has_value()) break;
        }

        if (q_sol.has_value()) {
            std::vector<double> q_vec(q_sol->data(), q_sol->data() + q_sol->size());
            RCLCPP_INFO(this->get_logger(),
                "Pose %s FEASIBLE: q=[%.3f, %.3f, %.3f, %.3f, %.3f]",
                pose.label,
                (*q_sol)[0], (*q_sol)[1], (*q_sol)[2], (*q_sol)[3], (*q_sol)[4]);

            _traj_queue.emplace(std::make_shared<SmoothLinearJointPath>(q_last,  q_vec,         4.0));
            _traj_queue.emplace(std::make_shared<SmoothLinearJointPath>(q_vec,   q_vec,         2.0)); // hold
            _traj_queue.emplace(std::make_shared<SmoothLinearJointPath>(q_vec,   home_joint_pos,3.0));
            q_last = home_joint_pos;
        } else {
            RCLCPP_WARN(this->get_logger(), "Pose %s INFEASIBLE.", pose.label);
        }
    }

    // ── 2. Task 2.3: circle trajectory (position-only IK) ────────────────────
    RCLCPP_INFO(this->get_logger(), "Computing Task 2.3 circle trajectory...");

    // Derive circle center from FK at a known stable configuration
    robot.set_joint_positions((Eigen::VectorXd(5) << 0.0, 0.3, -0.6, 0.3, 0.0).finished());
    robot.forward_kinematics();
    const Eigen::Vector3d circle_center = robot.get_end_effector_position();
    RCLCPP_INFO(this->get_logger(),
        "Circle center: [%.3f, %.3f, %.3f], radius: %.3f m",
        circle_center[0], circle_center[1], circle_center[2], CIRCLE_RADIUS);

    // Solve IK for each waypoint, warm-starting from previous solution
    std::vector<std::vector<double>> circle_joints;
    Eigen::VectorXd q_circ = q_home;
    int failed_wps = 0;

    for (int i = 0; i < CIRCLE_N; i++) {
        const double angle = 2.0 * M_PI * i / CIRCLE_N;
        const Eigen::Vector3d wp = circle_center
            + CIRCLE_RADIUS * Eigen::Vector3d(std::cos(angle), std::sin(angle), 0.0);

        auto q_sol = robot.solve_ik(wp, std::nullopt, q_circ);  // position-only

        if (q_sol.has_value()) {
            q_circ = *q_sol;
            circle_joints.push_back(std::vector<double>(q_sol->data(), q_sol->data() + q_sol->size()));
        } else {
            ++failed_wps;
            circle_joints.push_back(
                circle_joints.empty() ? home_joint_pos : circle_joints.back());
        }
    }

    RCLCPP_INFO(this->get_logger(),
        "Circle IK: %d/%d waypoints solved.", CIRCLE_N - failed_wps, CIRCLE_N);

    // Move to start of circle
    _traj_queue.emplace(std::make_shared<SmoothLinearJointPath>(
        home_joint_pos, circle_joints.front(), 4.0));

    // Execute circle as sequence of short linear segments
    const double dt = CIRCLE_PERIOD / CIRCLE_N;
    for (int i = 0; i < CIRCLE_N; i++) {
        const int next = (i + 1) % CIRCLE_N;
        _traj_queue.emplace(std::make_shared<LinearJointPath>(
            circle_joints[i], circle_joints[next], dt));
    }

    // Start the clock NOW, after all IK is done, so dt is never negative
    _beginning = this->now();
    RCLCPP_INFO(this->get_logger(),
        "Trajectory queue ready: %zu segments.", _traj_queue.size());
}

// ── Timer callback: publish current trajectory point ─────────────────────────
void Controller::_timer_callback()
{
    if (!first_state_received) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Waiting for first joint state...");
        return;
    }

    auto now = this->now();
    double dt = (now - this->_beginning).seconds();

    // Guard against clock skew / late-fired timers at trajectory start
    if (dt < 0.0) { _beginning = now; dt = 0.0; }

    if (_traj_queue.empty()) {
        if (_beginning.nanoseconds() > 0) {
            // Only print once when first going empty
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
                "All trajectories complete. Ctrl+C to exit.");
        }
        return;
    }

    // Advance through finished segments
    while (!_traj_queue.empty() && dt > _traj_queue.front()->get_duration()) {
        _beginning += rclcpp::Duration::from_seconds(_traj_queue.front()->get_duration());
        dt = (now - this->_beginning).seconds();
        _traj_queue.pop();
    }
    if (_traj_queue.empty()) return;

    auto msg   = trajectory_msgs::msg::JointTrajectory();
    auto point = trajectory_msgs::msg::JointTrajectoryPoint();
    msg.header.stamp = now;
    point.positions  = _traj_queue.front()->get_position(dt);
    msg.points       = {point};
    this->_pos_cmd_publisher->publish(msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}
