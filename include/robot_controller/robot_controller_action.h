#ifndef ROBOT_CONTROLLER_ACTION_H
#define ROBOT_CONTROLLER_ACTION_H

/* Standard libraries */
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/LaserScan.h>

/* User-defined libraries */
#include <robot_controller.h>
#include <ir2425_group_15/RobotControllerAction.h>

/* Robot controller namespace */
namespace rc {
    /* RobotController action class handling the robot controller */
    class RobotControllerAction {
        protected:
            // Node handler and action name
            ros::NodeHandle nh_;
            actionlib::SimpleActionServer<ir2425_group_15::RobotControllerAction> as_;
            std::string action_name_;

            // Goal, feedback and result messages
            ir2425_group_15::RobotControllerFeedback feedback_;
            ir2425_group_15::RobotControllerResult result_;

            // Action clients
            rc::PlayMotionClient play_motion_client_;
            rc::MoveBaseClient move_base_client_;
            rc::FollowJointTrajectoryClient torso_follow_joint_trajectory_;
            rc::FollowJointTrajectoryClient head_follow_joint_trajectory_;
            rc::FollowJointTrajectoryClient gripper_follow_joint_trajectory_;
            rc::FollowJointTrajectoryClient arm_follow_joint_trajectory_;

            // Additional attributes for corridor
            ros::Subscriber laser_subscriber_;
            ros::Publisher velocity_publisher_;
            bool is_corridor_;
            double left_distance_, right_distance_;

        public:
            // RobotController action constructor
            RobotControllerAction(std::string name);

            // Callback function describing the action for the robot controller
            void control_robot_callback(const ir2425_group_15::RobotControllerGoalConstPtr& goal);

            // Callback function to read laser scan topic and check corridor
            void corridor_check_callback(const sensor_msgs::LaserScanConstPtr& msg);

            // Motion "initial"
            void initial_navigation();
            // Motion "corridor"
            void corridor_navigation(geometry_msgs::PoseStamped waypoint);
            // Motion "final"
            void final_navigation(geometry_msgs::PoseStamped waypoint);
            
            // Motion "prepare_grasp"
            void prepare_grasp();
            // Motion "torso_control"
            void torso_control(double torso_lift_joint);
            // Motion "head_control"
            void head_control(std::vector<double> head_joints);
            // Motion "gripper_control"
            void gripper_control(std::vector<double> gripper_joints);
            // Motion "arm_control"
            void arm_control(std::vector<double> arm_joints);
            // Motion "open_gripper"
            void open_gripper();
            // Motion "close_gripper"
            void close_gripper();
            // Motion "rotate_base"
            void rotate_base(double orientation_angle);
            // Motion "move_straight"
            void move_straight(double distance);
            // Motion "home_pose"
            void home_pose();
    };
}

#endif // ROBOT_CONTROLLER_ACTION_H