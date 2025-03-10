#ifndef PICK_PLACE_ACTION_H
#define PICK_PLACE_ACTION_H

/* Standard libraries */
#include <actionlib/server/simple_action_server.h>

/* User-defined libraries */
#include <pick_place.h>
#include <robot_controller.h>
#include <ir2425_group_15/PickPlaceAction.h>
#include <ir2425_group_15/RobotControllerAction.h>

/* Pick place namespace */
namespace pp {
    /* PickPlace action class handling the pick place */
    class PickPlaceAction {
        protected:
            // Node handler and action name
            ros::NodeHandle nh_;
            actionlib::SimpleActionServer<ir2425_group_15::PickPlaceAction> as_;
            std::string action_name_;

            // Goal, feedback and result messages
            ir2425_group_15::PickPlaceFeedback feedback_;
            ir2425_group_15::PickPlaceResult result_;

            // Planning scene interface
            moveit::planning_interface::PlanningSceneInterface planning_scene_;
            
            // Move group arm interface: arm_1_joint, arm_2_joint, arm_3_joint, arm_4_joint, arm_5_joint, arm_6_joint, arm_7_joint, arm_tool_joint
            //  - Planning frame: base_footprint
            //  - End effector link: arm_tool_link
            moveit::planning_interface::MoveGroupInterface arm_move_group_;
            // Move group arm_torso interface: arm_1_joint, arm_2_joint, arm_3_joint, arm_4_joint, arm_5_joint, arm_6_joint, arm_7_joint, arm_tool_joint
            //  - Planning frame: base_footprint
            //  - End effector link: arm_tool_link
            moveit::planning_interface::MoveGroupInterface arm_torso_move_group_;
            // Move group gripper interface: gripper_left_finger_joint, gripper_right_finger_joint
            moveit::planning_interface::MoveGroupInterface gripper_move_group_;

            // Action clients
            actionlib::SimpleActionClient<ir2425_group_15::RobotControllerAction> robot_controller_client_;

            // Gazebo link service clients
            ros::ServiceClient gazebo_link_attach_client_;
            ros::ServiceClient gazebo_link_detach_client_;

        public:
            // PickPlace action constructor
            PickPlaceAction(std::string name);

            // Callback function describing the action for the pick place
            void pick_place_callback(const ir2425_group_15::PickPlaceGoalConstPtr& goal);

            // Motion "apply_table_collision_objects"
            void apply_table_collision_objects();
            // Motion "apply_apriltag_collision_objects"
            void apply_apriltag_collision_objects(std::vector<int> apriltag_ids, std::vector<geometry_msgs::PoseStamped> apriltag_pose_stamped, std::vector<std_msgs::ColorRGBA> apriltag_colors);
            // Motion "clear_collision_objects"
            void clear_collision_objects();
            // Motion "get_apriltag_collision_object_id"
            void get_apriltag_collision_object_id(int apriltag_id);

            // Motion "approach_object"
            void approach_object(std::string collision_object_id);
            // Motion "pick_object"
            void pick_object(std::string collision_object_id);
            void recovery_pick_object();
            // Motion "leave_object"
            void leave_object(std::string collision_object_id);
            void recovery_leave_object(int apriltag_id);
            // Motion "place_object"
            void place_object(int apriltag_id, geometry_msgs::PoseStamped place_location);
            void recovery_place_object(int apriltag_id);
    };
}

#endif // PICK_PLACE_ACTION_H