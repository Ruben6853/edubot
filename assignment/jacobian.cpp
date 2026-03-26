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
    std::vector<Eigen::Vector<double, 6>> ee_poses = {
        {0.2,0.2,0.2,0.000,1.570,0.650},
        {0.2,0.1,0.4,0.000,0.000, -1.570},
        {0.0,0.0,0.4,0.000, -0.785,1.570},
        {0.0,0.0,0.07,3.141,0.000,0.00},
        {0.0, 0.0452, 0.45, -0.785, 0.000, 3.141}
    };
    std::vector<std::optional<Eigen::VectorXd>> ik_solutions;
    for (const auto& pose : ee_poses) {
        auto ik_solution = robot.required_joint_angles(
            pose,
            0.15,
            1e-4,
            500,
            1000,
            0.1
            );
        if (ik_solution) {
            auto ee_pose = robot.get_end_effector_pose();
            std::cout << "IK solution found for pose: \n" << pose.transpose()
            << "\nJoint angles: " << ik_solution->transpose()/M_PI *180.0f
            << "\nEnd effector pose for IK solution: \n" << ee_pose.transpose()<< std::endl;
        }
        else {
            std::cout << "No IK solution found for pose: \n" << pose.transpose() << std::endl;
        }
        std::cout << "-----------------------------" << std::endl;
        ik_solutions.push_back(ik_solution);
    }
}