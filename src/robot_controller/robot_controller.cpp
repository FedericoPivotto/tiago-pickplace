/* User-defined libraries */
#include <robot_controller.h>
#include <message_builder.h>

/* Function to get the move_base goal for the given pose stamped */
move_base_msgs::MoveBaseGoal rc::get_move_base_goal(geometry_msgs::PoseStamped pose_stamped) {
    // Set the goal pose
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = pose_stamped;

    return goal;
}

/* Function to get the play_motion goal for the given motion */
play_motion_msgs::PlayMotionGoal rc::get_play_motion_goal(std::string motion_name) {
    // Set the goal motion
    play_motion_msgs::PlayMotionGoal goal;
    goal.motion_name = motion_name;
    goal.skip_planning = false;
    goal.priority = 0;

    return goal;
}

/* Function to get the torso follow_joint_trajectory goal */
control_msgs::FollowJointTrajectoryGoal rc::get_torso_follow_joint_trajectory_goal(double torso_lift_joint) {
    // Set the goal motion
    control_msgs::FollowJointTrajectoryGoal goal;

    // Start from now after 1 second
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    
    // Select torso lift joint
    int n_joints = 1, joint_index = 0;
    goal.trajectory.joint_names.push_back("torso_lift_joint");

    // Number of waypoints in the goal trajectory
    int n_waypoints = 1, waypoint_index = 0;
    goal.trajectory.points.resize(n_waypoints);

    // Waypoint position
    goal.trajectory.points[waypoint_index].positions.resize(n_joints);
    goal.trajectory.points[waypoint_index].positions[joint_index] = torso_lift_joint;
    // Waypoint step-wise velocity
    goal.trajectory.points[waypoint_index].velocities.resize(n_joints);
    goal.trajectory.points[waypoint_index].velocities[joint_index] = 0.0;
    // Waypoint to be reached 10 second after starting the trajectory
    goal.trajectory.points[waypoint_index].time_from_start = ros::Duration(5);

    return goal;
}

/* Function to get the torso follow_joint_trajectory goal */
control_msgs::FollowJointTrajectoryGoal rc::get_head_follow_joint_trajectory_goal(double head_1_joint, double head_2_joint) {
    // Set the goal motion
    control_msgs::FollowJointTrajectoryGoal goal;

    // Start from now after 1 second
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    
    // Select head joints
    int n_joints = 2, joint_index = 0;
    goal.trajectory.joint_names.push_back("head_1_joint");
    goal.trajectory.joint_names.push_back("head_2_joint");

    // Number of waypoints in the goal trajectory
    int n_waypoints = 1, waypoint_index = 0;
    goal.trajectory.points.resize(n_waypoints);

    // Waypoint position
    goal.trajectory.points[waypoint_index].positions.resize(n_joints);
    goal.trajectory.points[waypoint_index].positions[joint_index] = head_1_joint;
    goal.trajectory.points[waypoint_index].positions[joint_index + 1] = head_2_joint;
    // Waypoint step-wise velocity
    goal.trajectory.points[waypoint_index].velocities.resize(n_joints);
    goal.trajectory.points[waypoint_index].velocities[joint_index] = 0.0;
    goal.trajectory.points[waypoint_index].velocities[joint_index + 1] = 0.0;
    // Waypoint to be reached 10 second after starting the trajectory
    goal.trajectory.points[waypoint_index].time_from_start = ros::Duration(5);

    return goal;
}

/* Function to get the head follow_joint_trajectory goal */
control_msgs::FollowJointTrajectoryGoal rc::get_gripper_follow_joint_trajectory_goal(double gripper_left_finger_joint, double gripper_right_finger_joint) {
    // Set the goal motion
    control_msgs::FollowJointTrajectoryGoal goal;

    // Start from now after 1 second
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    
    // Select gripper joints
    int n_joints = 2, joint_index = 0;
    goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
    goal.trajectory.joint_names.push_back("gripper_right_finger_joint");

    // Number of waypoints in the goal trajectory
    int n_waypoints = 1, waypoint_index = 0;
    goal.trajectory.points.resize(n_waypoints);

    // Waypoint position
    goal.trajectory.points[waypoint_index].positions.resize(n_joints);
    goal.trajectory.points[waypoint_index].positions[joint_index] = gripper_left_finger_joint;
    goal.trajectory.points[waypoint_index].positions[joint_index + 1] = gripper_right_finger_joint;
    // Waypoint step-wise velocity
    goal.trajectory.points[waypoint_index].velocities.resize(n_joints);
    goal.trajectory.points[waypoint_index].velocities[joint_index] = 0.0;
    goal.trajectory.points[waypoint_index].velocities[joint_index + 1] = 0.0;
    // Waypoint to be reached 5 second after starting the trajectory
    goal.trajectory.points[waypoint_index].time_from_start = ros::Duration(3);

    return goal;
}

/* Function to get the arm follow_joint_trajectory goal */
control_msgs::FollowJointTrajectoryGoal rc::get_arm_follow_joint_trajectory_goal(double arm_1_joint, double arm_2_joint, double arm_3_joint, double arm_4_joint, double arm_5_joint, double arm_6_joint, double arm_7_joint) {
    // Set the goal motion
    control_msgs::FollowJointTrajectoryGoal goal;

    // Start from now after 1 second
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    
    // Select arm joints
    int n_joints = 7, joint_index = 0;
    goal.trajectory.joint_names.push_back("arm_1_joint");
    goal.trajectory.joint_names.push_back("arm_2_joint");
    goal.trajectory.joint_names.push_back("arm_3_joint");
    goal.trajectory.joint_names.push_back("arm_4_joint");
    goal.trajectory.joint_names.push_back("arm_5_joint");
    goal.trajectory.joint_names.push_back("arm_6_joint");
    goal.trajectory.joint_names.push_back("arm_7_joint");

    // Number of waypoints in the goal trajectory
    int n_waypoints = 1, waypoint_index = 0;
    goal.trajectory.points.resize(n_waypoints);

    // Waypoint position
    goal.trajectory.points[waypoint_index].positions.resize(n_joints);
    goal.trajectory.points[waypoint_index].positions[joint_index] = arm_1_joint;
    goal.trajectory.points[waypoint_index].positions[joint_index + 1] = arm_2_joint;
    goal.trajectory.points[waypoint_index].positions[joint_index + 2] = arm_3_joint;
    goal.trajectory.points[waypoint_index].positions[joint_index + 3] = arm_4_joint;
    goal.trajectory.points[waypoint_index].positions[joint_index + 4] = arm_5_joint;
    goal.trajectory.points[waypoint_index].positions[joint_index + 5] = arm_6_joint;
    goal.trajectory.points[waypoint_index].positions[joint_index + 6] = arm_7_joint;
    // Waypoint step-wise velocity
    goal.trajectory.points[waypoint_index].velocities.resize(n_joints);
    goal.trajectory.points[waypoint_index].velocities[joint_index] = 0.0;
    goal.trajectory.points[waypoint_index].velocities[joint_index + 1] = 0.0;
    goal.trajectory.points[waypoint_index].velocities[joint_index + 2] = 0.0;
    goal.trajectory.points[waypoint_index].velocities[joint_index + 3] = 0.0;
    goal.trajectory.points[waypoint_index].velocities[joint_index + 4] = 0.0;
    goal.trajectory.points[waypoint_index].velocities[joint_index + 5] = 0.0;
    goal.trajectory.points[waypoint_index].velocities[joint_index + 6] = 0.0;
    // Waypoint to be reached 8 second after starting the trajectory
    goal.trajectory.points[waypoint_index].time_from_start = ros::Duration(8);

    return goal;
}

/* Function to get the move_base goal for a given rotation angle in radiants */
geometry_msgs::PoseStamped rc::get_pose_stamped_rotation(std_msgs::Header header, float angle) {
    // Set the robot goal pose stamped
    tf2::Quaternion tf_rotation = rc::get_tf_quaternion_from_quaternion(mb::get_quaternion_from_rpy(0, 0, angle));
    geometry_msgs::PoseStamped pose_stamped = mb::get_pose_stamped(header, mb::get_pose(mb::get_point(), tf2::toMsg(tf_rotation)));

    return pose_stamped;
}

/* Function to get tf2::Quaternion from geometry_msgs/Quaternion */
tf2::Quaternion rc::get_tf_quaternion_from_quaternion(geometry_msgs::Quaternion quaternion) {
    return tf2::Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
}


/* Function to get the move_base goal for a given straight distance in meters */
geometry_msgs::PoseStamped rc::get_pose_stamped_straight(std_msgs::Header header, float distance) {
    // Set the robot goal pose stamped
    geometry_msgs::PoseStamped pose_stamped = mb::get_pose_stamped(header, mb::get_pose(mb::get_point(distance), mb::get_quaternion()));

    return pose_stamped;
}

/* Find table legs given scan msg */
std::vector<geometry_msgs::PoseStamped> rc::find_table_legs(const sensor_msgs::LaserScan& msg){
    // Store leg positions
    std::vector<geometry_msgs::PoseStamped> leg_positions;

    // Set parameters
    const double angle_increment = msg.angle_increment;
    const double angle_min = msg.angle_min;
    const double max_gap = 1.0;               

    // Storage of segmented regions
    std::vector<std::vector<size_t>> regions;
    std::vector<size_t> current_region;

    // Segment the ranges into regions based on large gaps
    for (size_t i = 1; i < msg.ranges.size(); ++i) {
        if (std::abs(msg.ranges[i] - msg.ranges[i - 1]) > max_gap) {
            // Add samples to region until gap between consecutive samples is too large
            if (!current_region.empty()) {
                regions.push_back(current_region);
                current_region.clear();
            }
        }
        current_region.push_back(i);
    }

    // Identify legs in each region
    for (const auto& region : regions) {
        // Check if the distances in the region increase and then decrease
        bool is_leg = false;
        // Increasing and decreasing sample counter
        int increasing_counter = 0;
        int decreasing_counter = 0;

        // Check if region is a table leg
        for (size_t j = 1; j < region.size(); ++j) {
            double dist1 = msg.ranges[region[j - 1]];
            double dist2 = msg.ranges[region[j]];

            if (dist2 > dist1) {
                increasing_counter += 1;
            }

            if (dist2 < dist1) {
                decreasing_counter += 1;
            }
        }

        // Check if enough samples resembling a leg
        if((increasing_counter >= 1 && increasing_counter <= 15) && (decreasing_counter >= 1 && decreasing_counter <= 15))
            is_leg = true;

        // Compute the leg's position (average position of the region)
        if (is_leg) {
            double avg_angle = 0.0;
            double avg_distance = 0.0;

            for (size_t index : region) {
                avg_angle += angle_min + index * angle_increment;
                avg_distance += msg.ranges[index];
            }

            avg_angle = avg_angle / region.size();
            avg_distance = avg_distance / region.size();

            double x = avg_distance * std::cos(avg_angle);
            double y = avg_distance * std::sin(avg_angle);

            // Create PoseStamped for the leg position
            geometry_msgs::PoseStamped leg_pose;
            leg_pose.header = msg.header;
            leg_pose.pose.position = mb::get_point(x, y, 0.0);
            leg_pose.pose.orientation.w = 1.0;

            // Check if legs are at most two
            if (leg_positions.size() < 2)
                leg_positions.push_back(leg_pose);
        }
    }
 
    // Return the detected leg positions
    return leg_positions;
}

/* Find docking poses for the given table leg position (w.r.t. map frame) */
void rc::find_dockings(const geometry_msgs::Pose table_position, geometry_msgs::PoseStamped& docking_pose_one, geometry_msgs::PoseStamped& docking_pose_two){
    // Define shift distance
    const double shift_distance = 0.85; // 0.9

    // Compute shifted positions
    double x_shifted_positive = table_position.position.x + shift_distance;
    double x_shifted_negative = table_position.position.x - shift_distance;

    // Frame ID for the poses
    const std::string frame_id = "map";

    // Compute yaw for both dockings
    double dx_positive = table_position.position.x - x_shifted_positive;
    double dy_positive = 0;
    double dx_negative = table_position.position.x - x_shifted_negative;
    double dy_negative = 0;

    double yaw_positive = std::atan2(dy_positive, dx_positive);
    double yaw_negative = std::atan2(dy_negative, dx_negative);

    // Compute docking quaternions
    tf2::Quaternion q_positive;
    q_positive.setRPY(0, 0, yaw_positive);

    tf2::Quaternion q_negative;
    q_negative.setRPY(0, 0, yaw_negative);

    // Compute docking poses
    docking_pose_one.header.frame_id = frame_id;
    docking_pose_one.header.stamp = ros::Time::now();
    docking_pose_one.pose.position = mb::get_point(x_shifted_positive, table_position.position.y, 0.0);
    docking_pose_one.pose.orientation = mb::get_quaternion(q_positive.x(), q_positive.y(), q_positive.z(), q_positive.w());

    docking_pose_two.header.frame_id = frame_id;
    docking_pose_two.header.stamp = ros::Time::now();
    docking_pose_two.pose.position = mb::get_point(x_shifted_negative, table_position.position.y, 0.0);
    docking_pose_two.pose.orientation = mb::get_quaternion(q_negative.x(), q_negative.y(), q_negative.z(), q_negative.w());
}