#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

/* Standard libraries */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <play_motion_msgs/PlayMotionAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/LaserScan.h>

/* Robot controller namespace */
namespace rc {
    // Aliases for using move base client
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    typedef actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction> PlayMotionClient;
    typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> FollowJointTrajectoryClient;

    // Rotation correction
    constexpr double ROTATION_CORRECTION = M_PI / 12.3;

    // Torso controller joint limits
    constexpr double MIN_TORSO_LIFT_JOINT = 0.0;
    constexpr double MAX_TORSO_LIFT_JOINT = 0.35;
    
    // Head controller joint limits
    constexpr double MIN_HEAD_1_JOINT = -1.24;
    constexpr double MAX_HEAD_1_JOINT = 1.24;
    constexpr double MIN_HEAD_2_JOINT = -0.98;
    constexpr double MAX_HEAD_2_JOINT = 0.72;

    
    // Gripper controller joint limits
    constexpr double MIN_GRIPPER_LEFT_FINGER_JOINT = 0.0;
    constexpr double MAX_GRIPPER_LEFT_FINGER_JOINT = 0.04;
    constexpr double MIN_GRIPPER_RIGHT_FINGER_JOINT = 0.0;
    constexpr double MAX_GRIPPER_RIGHT_FINGER_JOINT = 0.04;

    // Arm controller joint limits
    constexpr double MIN_ARM_1_JOINT = 0.07;
    constexpr double MAX_ARM_1_JOINT = 2.68;
    constexpr double MIN_ARM_2_JOINT = -1.50;
    constexpr double MAX_ARM_2_JOINT = 1.02;
    constexpr double MIN_ARM_3_JOINT = -3.46;
    constexpr double MAX_ARM_3_JOINT = 1.50;
    constexpr double MIN_ARM_4_JOINT = -0.32;
    constexpr double MAX_ARM_4_JOINT = 2.29;
    constexpr double MIN_ARM_5_JOINT = -2.07;
    constexpr double MAX_ARM_5_JOINT = 2.07;
    constexpr double MIN_ARM_6_JOINT = -1.39;
    constexpr double MAX_ARM_6_JOINT = 1.39;
    constexpr double MIN_ARM_7_JOINT = -2.07;
    constexpr double MAX_ARM_7_JOINT = 2.07;

    move_base_msgs::MoveBaseGoal get_move_base_goal(geometry_msgs::PoseStamped pose_stamped);
    play_motion_msgs::PlayMotionGoal get_play_motion_goal(std::string motion_name);

    control_msgs::FollowJointTrajectoryGoal get_torso_follow_joint_trajectory_goal(double torso_lift_joint);
    control_msgs::FollowJointTrajectoryGoal get_head_follow_joint_trajectory_goal(double head_1_joint, double head_2_joint);
    control_msgs::FollowJointTrajectoryGoal get_gripper_follow_joint_trajectory_goal(double gripper_left_finger_joint, double gripper_right_finger_joint);
    control_msgs::FollowJointTrajectoryGoal get_arm_follow_joint_trajectory_goal(double arm_1_joint, double arm_2_joint, double arm_3_joint, double arm_4_joint, double arm_5_joint, double arm_6_joint, double arm_7_joint);

    geometry_msgs::PoseStamped get_pose_stamped_rotation(std_msgs::Header header, float angle);
    tf2::Quaternion get_tf_quaternion_from_quaternion(geometry_msgs::Quaternion quaternion);
    geometry_msgs::PoseStamped get_pose_stamped_straight(std_msgs::Header header, float distance);

    std::vector<geometry_msgs::PoseStamped> find_table_legs(const sensor_msgs::LaserScan& msg);
    void find_dockings(const geometry_msgs::Pose table_position, geometry_msgs::PoseStamped& docking_pose_one, geometry_msgs::PoseStamped& docking_pose_two);
}

#endif // ROBOT_CONTROLLER_H