//
// Created by ruben on 3/23/26.
//

#include "kinematics.hpp"

int main() {
     auto robot = km::SequentialRobot({
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
    });

    robot.set_joint_positions(std::vector<double>{
    -0.99862148778,
    0.46019423400000004,
    -1.14128170032,
    -1.0477088727400001,
    1.51557301064});

    auto pose = robot.get_end_effector_pose();
    std::cout << "End effector pose at home position: \n" << pose.transpose() << std::endl;

    auto desired_pose = Eigen::Vector<double, 6>{0.0f, 0.20f, 0.15f, -3.1415f, -0.4f, 1.57f};
    auto ik_solution = robot.required_joint_angles(
        desired_pose,
        0.15,
        1e-3,
        500,
        1000,
        0.1
    );
    if (ik_solution) {
        robot.set_joint_positions(*ik_solution);
        auto achieved_pose = robot.get_end_effector_pose();
        std::cout << "IK solution found for desired pose: \n" << desired_pose.transpose()
        << "\nJoint angles: " << ik_solution->transpose()/M_PI *180.0f
        << "\nAchieved end effector pose for IK solution: \n" << achieved_pose.transpose()<< std::endl;

        //rotate shoulder
        auto start_pos = *ik_solution;
        start_pos[0] += 0.77f; // rotate shoulder yaw by 0.5 radians
        robot.set_joint_positions(start_pos);
        auto rotated_pose = robot.get_end_effector_pose();
        std::cout << "After rotating shoulder yaw by 0.77 radians, end effector pose: \n" << rotated_pose.transpose() << std::endl;
        std::cout << "for joint angles: " << robot.get_joint_positions().transpose() << std::endl;
    }
     else {
        std::cout << "No IK solution found for desired pose: \n" << desired_pose.transpose() << std::endl;
    }
}