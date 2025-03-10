/* Standard libraries */
#include <ros/ros.h>

/* User-defined libraries */
#include <task_status_action.h>
#include <robot_controller_action.h>


/* TaskStatus action constructor */
ts::TaskStatusAction::TaskStatusAction(std::string name) : as_(nh_, name, boost::bind(&TaskStatusAction::task_status_callback, this, _1), false), action_name_(name), robot_navigation_client_("robot_navigation_action", true), apriltags_detection_client_("apriltags_detection_action", true), robot_controller_client_("robot_controller_action", true), pick_place_client_("pick_place_action", true), play_motion_client_("play_motion", true) {
    // Start the task status action
    as_.start();

    // Subscribe to detect table legs
    laser_subscriber_ = nh_.subscribe("scan", 1, &ts::TaskStatusAction::laser_table_callback, this);

    // Wait robot navigation, AprilTags detection and pick place servers
    robot_navigation_client_.waitForServer();
    apriltags_detection_client_.waitForServer();
    robot_controller_client_.waitForServer();
    pick_place_client_.waitForServer();
    play_motion_client_.waitForServer();
}

/* Callback function to read laser scan topic */
void ts::TaskStatusAction::laser_table_callback(const sensor_msgs::LaserScanConstPtr& msg) {
    last_msg_ = msg;
}

/* Sample message from laser topic */
sensor_msgs::LaserScan ts::TaskStatusAction::laser_sample() {
    // Sample scan msg
    sensor_msgs::LaserScan sample_msg = *last_msg_;

    // Shutdown subsriber if msg not empty
    if (last_msg_->ranges.empty() == false)
        laser_subscriber_.shutdown();

    return sample_msg;
}

/* Callback function describing the action for the robot navigation */
void ts::TaskStatusAction::task_status_callback(const ir2425_group_15::TaskStatusGoalConstPtr& goal) {
    // Pick-place task
    this->pick_place(goal);

    return;
}

/* Callback function describing the action for the pick-place */
void ts::TaskStatusAction::pick_place(const ir2425_group_15::TaskStatusGoalConstPtr& goal) {
    // Initial action state
    bool success = true;

    // Enable verbose feedback
    feedback_.verbose = true;

    // Send the AprilTags dectection goal
    apriltags_detection_client_.sendGoal(goal->apriltags_detection_goal, boost::bind(&TaskStatusAction::apriltags_detection_done_callback, this, _1, _2), NULL, boost::bind(&TaskStatusAction::apriltags_detection_status_feedback_callback, this, _1));
    success = this->wait_apriltags_detection_start() && success;

    // Send the robot navigation goal to reach tables
    robot_navigation_client_.sendGoal(goal->robot_navigation_goal, boost::bind(&TaskStatusAction::robot_navigation_done_callback, this, _1, _2), NULL, boost::bind(&TaskStatusAction::robot_navigation_status_feedback_callback, this, _1));
    success = this->wait_robot_navigation_result() && success;

    // Disable verbose feedback
    feedback_.verbose = false;
    
    // Define robot controller rotate base and move straight goals
    ir2425_group_15::RobotControllerGoal orientation_angle_goal, straight_goal;
    orientation_angle_goal.motion = "rotate_base";
    straight_goal.motion = "move_straight";

    // Send start feedback
    feedback_.task_state = "Correct orientation";
    as_.publishFeedback(feedback_);
    // Correct waypoint orientation
    orientation_angle_goal.angle = - M_PI / 2.0;
    robot_controller_client_.sendGoalAndWait(orientation_angle_goal);
    // Wait motion completion
    ros::Duration(1).sleep();

    // Send start feedback
    feedback_.task_state = "Compute docking poses through the laser scan";
    as_.publishFeedback(feedback_);
    // Estimate table legs position
    sensor_msgs::LaserScan laser_msg = ts::TaskStatusAction::laser_sample();
    std::vector<geometry_msgs::PoseStamped> table_legs = rc::find_table_legs(laser_msg);
    // Compute place table dockings
    geometry_msgs::PoseStamped table_place_pose = mb::get_pose_stamped_wrt_frame(table_legs[0], "map");
    geometry_msgs::PoseStamped docking_pose_place_left, docking_pose_place_right;
    rc::find_dockings(table_place_pose.pose, docking_pose_place_left, docking_pose_place_right);
    // Compute pick table dockings
    geometry_msgs::PoseStamped table_pick_pose = mb::get_pose_stamped_wrt_frame(table_legs[1], "map");
    geometry_msgs::PoseStamped docking_pose_pick_left, docking_pose_pick_right;
    rc::find_dockings(table_pick_pose.pose, docking_pose_pick_left, docking_pose_pick_right);

    // Extract docking position information for navigation
    double distance_offset = 0.025;
    double distance_y_docking_poses = fabs(docking_pose_place_left.pose.position.y - docking_pose_pick_left.pose.position.y) - distance_offset;
    double orientation_dockings_left = mb::get_rpy_from_quaternion(docking_pose_pick_left.pose.orientation).z;
    double orientation_dockings_right = mb::get_rpy_from_quaternion(docking_pose_pick_right.pose.orientation).z;

    // Send start feedback
    feedback_.task_state = "Reach place docking pose";
    as_.publishFeedback(feedback_);
    // Reach docking position to read table AprilTag according to offset
    geometry_msgs::PoseStamped current_pose = mb::get_frame_pose_stamped_wrt_frame("base_footprint", "map");
    double offset = 0.2;
    straight_goal.distance = fabs(docking_pose_place_left.pose.position.y - current_pose.pose.position.y) + offset;
    robot_controller_client_.sendGoalAndWait(straight_goal);
    orientation_angle_goal.angle = orientation_dockings_left;
    robot_controller_client_.sendGoalAndWait(orientation_angle_goal);

    // Send start feedback
    feedback_.task_state = "Move head down to look at TAG-10";
    as_.publishFeedback(feedback_);
    // Move head down
    ir2425_group_15::RobotControllerGoal head_control_goal;
    head_control_goal.motion = "head_control";
    head_control_goal.head_joints = std::vector<double>{0.0, -0.75};
    robot_controller_client_.sendGoalAndWait(head_control_goal);
    
    // Send start feedback
    feedback_.task_state = "Compute place locations through the camera";
    as_.publishFeedback(feedback_);
    // Compute place locations along the given straight line
    std::vector<geometry_msgs::PoseStamped> place_locations = pp::get_straight_line_place_locations(goal->pick_place_goal.m, goal->pick_place_goal.q, "tag_10", "map", 9, 0.663);
    std::vector<bool> is_occupied(place_locations.size(), false);
    
    // Send start feedback
    feedback_.task_state = "Reach pick docking pose";
    as_.publishFeedback(feedback_);
    // Reach second docking position to pick AprilTag objects
    orientation_angle_goal.angle = - M_PI / 2.0;
    robot_controller_client_.sendGoalAndWait(orientation_angle_goal);
    straight_goal.distance = distance_y_docking_poses - offset;
    robot_controller_client_.sendGoalAndWait(straight_goal);
    orientation_angle_goal.angle = orientation_dockings_left;
    robot_controller_client_.sendGoalAndWait(orientation_angle_goal);

    // Define pick-place goal
    ir2425_group_15::PickPlaceGoal pick_place_goal;
    // Define placed object vectors
    std::vector<int> placed_apriltags, recovery_placed_apriltags;

    // Perform pick-place in both sides
    for(size_t i = 0; i < 2; ++i) {
        // Send start feedback
        feedback_.task_state = "Prepare the robot for picking";
        as_.publishFeedback(feedback_);
        // Prepare robot for grasp
        ir2425_group_15::RobotControllerGoal prepare_grasp_goal;
        prepare_grasp_goal.motion = "prepare_grasp";
        robot_controller_client_.sendGoalAndWait(prepare_grasp_goal);

        // Send start feedback
        feedback_.task_state = "Start pick-place routine on current side";
        as_.publishFeedback(feedback_);
        // Send pick-place goal for each object
        std::vector<int> missed_apriltags;
        for(bool reachable_objects = true; reachable_objects;) {
            // Color string
            std::string color;
    
            // Send start feedback
            feedback_.task_state = "Apply table collision objects";
            as_.publishFeedback(feedback_);
            // Get current AprilTag available
            std::vector<std::string> apriltag_frame_ids = mb::get_frame_ids(std::regex("tag_.*"));
            if(apriltag_frame_ids.empty()) {
                // Send result feedback
                feedback_.task_state = "No reachable objects available";
                as_.publishFeedback(feedback_);
                break;
            }
            
            // Apply table collision objects
            pick_place_goal.motion = "apply_table_collision_objects";
            pick_place_client_.sendGoal(pick_place_goal, boost::bind(&TaskStatusAction::pick_place_done_callback, this, _1, _2), NULL, NULL);
            pick_place_client_.waitForResult();
            // Send subroutine result
            feedback_.task_state = feedback_.pick_place_feedback.pick_place_state;
            as_.publishFeedback(feedback_);

            // Send start feedback
            feedback_.task_state = "Apply AprilTag collision objects";
            as_.publishFeedback(feedback_);
            // Read current AprilTags detected
            ad::AprilTagsDetection pick_apriltags_feedback(
                feedback_.apriltags_detection_feedback.srv_target_apriltag_ids,
                feedback_.apriltags_detection_feedback.target_apriltag_ids, feedback_.apriltags_detection_feedback.target_apriltag_poses, feedback_.apriltags_detection_feedback.target_apriltag_colors,
                feedback_.apriltags_detection_feedback.nontarget_apriltag_ids, feedback_.apriltags_detection_feedback.nontarget_apriltag_poses, feedback_.apriltags_detection_feedback.nontarget_apriltag_colors
            );
            // Target AprilTags detected
            std::map<int, std::pair<geometry_msgs::PoseStamped, std_msgs::ColorRGBA>> pick_target_apriltags = pick_apriltags_feedback.get_target_apriltags_detected();
            // Collect AprilTag objects to avoid
            pick_place_goal.apriltag_ids.clear();
            pick_place_goal.apriltag_pose_stamped.clear();
            pick_place_goal.apriltag_colors.clear();
            for(const std::string& apriltag_frame_id : apriltag_frame_ids) {
                // Save AprilTag ID, frame and color
                int apriltag_id; std::sscanf(apriltag_frame_id.c_str(), "tag_%d", &apriltag_id);
                pick_place_goal.apriltag_ids.push_back(apriltag_id);
                geometry_msgs::PoseStamped apriltag_frame_pose_stamped = mb::get_frame_pose_stamped_wrt_frame(apriltag_frame_id, "map");
                pick_place_goal.apriltag_pose_stamped.push_back(apriltag_frame_pose_stamped);
                std_msgs::ColorRGBA apriltag_color = pick_target_apriltags[apriltag_id].second;
                pick_place_goal.apriltag_colors.push_back(apriltag_color);
            }
            // Apply AprilTag collision objects
            pick_place_goal.motion = "apply_apriltag_collision_objects";
            pick_place_client_.sendGoal(pick_place_goal, boost::bind(&TaskStatusAction::pick_place_done_callback, this, _1, _2), NULL, NULL);
            pick_place_client_.waitForResult();
            // Send subroutine result
            feedback_.task_state = feedback_.pick_place_feedback.pick_place_state;
            as_.publishFeedback(feedback_);

            // Send start feedback
            feedback_.task_state = "Approach an object";
            as_.publishFeedback(feedback_);
            // Approach object
            pick_place_goal.motion = "approach_object";
            // Check reachability
            while(reachable_objects = !apriltag_frame_ids.empty()) {
                // Select AprilTag to pick and place
                std::sscanf(apriltag_frame_ids.back().c_str(), "tag_%d", &pick_place_goal.apriltag_id);
                // Avoid placed collision object
                if(std::find(placed_apriltags.begin(), placed_apriltags.end(), pick_place_goal.apriltag_id) == placed_apriltags.end()
                    && std::find(recovery_placed_apriltags.begin(), recovery_placed_apriltags.end(), pick_place_goal.apriltag_id) == recovery_placed_apriltags.end()
                    && std::find(missed_apriltags.begin(), missed_apriltags.end(), pick_place_goal.apriltag_id) == missed_apriltags.end()) {
                    // Send approach goal
                    pick_place_client_.sendGoal(pick_place_goal, boost::bind(&TaskStatusAction::pick_place_done_callback, this, _1, _2), NULL, NULL);
                    pick_place_client_.waitForResult();
                    // Skip object when not reachable
                    if(! feedback_.pick_place_feedback.is_completed) {
                        // Send result feedback
                        color = ts::get_color_string(pick_target_apriltags[pick_place_goal.apriltag_id].second);
                        feedback_.task_state = "Object with TAG-" + std::to_string(pick_place_goal.apriltag_id) + " (" + color + ") not approachable";
                        as_.publishFeedback(feedback_);
                        apriltag_frame_ids.pop_back();
                    }
                    else {
                        // Send result feedback
                        color = ts::get_color_string(pick_target_apriltags[pick_place_goal.apriltag_id].second);
                        feedback_.task_state = "Object with TAG-" + std::to_string(pick_place_goal.apriltag_id) + " (" + color + ") approached";
                        as_.publishFeedback(feedback_);
                        break;
                    }
                } else {
                    apriltag_frame_ids.pop_back();
                }
            }
            // No objects to approach 
            if(! reachable_objects) {
                // Send result feedback
                feedback_.task_state = "No approachable objects available";
                as_.publishFeedback(feedback_);
                // Send subroutine result
                feedback_.task_state = feedback_.pick_place_feedback.pick_place_state;
                as_.publishFeedback(feedback_);
                break;
            }
            // Send subroutine result
            feedback_.task_state = feedback_.pick_place_feedback.pick_place_state;
            as_.publishFeedback(feedback_);

            // Send start feedback
            feedback_.task_state = "Pick the object";
            as_.publishFeedback(feedback_);
            // Pick object
            pick_place_goal.motion = "pick_object";
            pick_place_client_.sendGoal(pick_place_goal, boost::bind(&TaskStatusAction::pick_place_done_callback, this, _1, _2), NULL, NULL);
            pick_place_client_.waitForResult();
            // Apply recover since no pickable object
            if(! feedback_.pick_place_feedback.is_completed) {
                // Update missed AprilTag objects
                missed_apriltags.push_back(pick_place_goal.apriltag_id);
                // Send result feedback
                color = ts::get_color_string(pick_target_apriltags[pick_place_goal.apriltag_id].second);
                feedback_.task_state = "Object with TAG-" + std::to_string(pick_place_goal.apriltag_id) + " (" + color + ") not pickable";
                as_.publishFeedback(feedback_);
                // Send subroutine result
                feedback_.task_state = feedback_.pick_place_feedback.pick_place_state;
                as_.publishFeedback(feedback_);
                // Send start feedback
                feedback_.task_state = "Remove all collision objects";
                as_.publishFeedback(feedback_);
                // Clear collision objects
                pick_place_goal.motion = "clear_collision_objects";
                pick_place_client_.sendGoal(pick_place_goal, boost::bind(&TaskStatusAction::pick_place_done_callback, this, _1, _2), NULL, NULL);
                pick_place_client_.waitForResult();
                // Send subroutine result
                feedback_.task_state = feedback_.pick_place_feedback.pick_place_state;
                as_.publishFeedback(feedback_);
                continue;
            }
            // Send result feedback
            color = ts::get_color_string(pick_target_apriltags[pick_place_goal.apriltag_id].second);
            feedback_.task_state = "Object with TAG-" + std::to_string(pick_place_goal.apriltag_id) + " (" + color + ") picked";
            as_.publishFeedback(feedback_);
            // Send subroutine result
            feedback_.task_state = feedback_.pick_place_feedback.pick_place_state;
            as_.publishFeedback(feedback_);

            // Send start feedback
            feedback_.task_state = "Retreat the object";
            as_.publishFeedback(feedback_);
            // Leave object
            pick_place_goal.motion = "leave_object";
            pick_place_client_.sendGoal(pick_place_goal, boost::bind(&TaskStatusAction::pick_place_done_callback, this, _1, _2), NULL, NULL);
            pick_place_client_.waitForResult();
            // Apply recovery to since no leavable object
            if(! feedback_.pick_place_feedback.is_completed) {
                missed_apriltags.push_back(pick_place_goal.apriltag_id);
                // Send result feedback
                color = ts::get_color_string(pick_target_apriltags[pick_place_goal.apriltag_id].second);
                feedback_.task_state = "Object with TAG-" + std::to_string(pick_place_goal.apriltag_id) + " (" + color + ") not retratable";
                as_.publishFeedback(feedback_);
                // Send subroutine result
                feedback_.task_state = feedback_.pick_place_feedback.pick_place_state;
                as_.publishFeedback(feedback_);
                // Send start feedback
                feedback_.task_state = "Remove all collision objects";
                as_.publishFeedback(feedback_);
                // Clear collision objects
                pick_place_goal.motion = "clear_collision_objects";
                pick_place_client_.sendGoal(pick_place_goal, boost::bind(&TaskStatusAction::pick_place_done_callback, this, _1, _2), NULL, NULL);
                pick_place_client_.waitForResult();
                // Send subroutine result
                feedback_.task_state = feedback_.pick_place_feedback.pick_place_state;
                as_.publishFeedback(feedback_);
                continue;
            }
            // Send result feedback
            color = ts::get_color_string(pick_target_apriltags[pick_place_goal.apriltag_id].second);
            feedback_.task_state = "Object with TAG-" + std::to_string(pick_place_goal.apriltag_id) + " (" + color + ") retreated";
            as_.publishFeedback(feedback_);

            // Send start feedback
            feedback_.task_state = "Move arm to the secure pose";
            as_.publishFeedback(feedback_);
            // Move to pick secure pose
            ir2425_group_15::RobotControllerGoal secure_pose_goal;
            secure_pose_goal.motion = "arm_control";
            secure_pose_goal.arm_joints = std::vector<double>{1.50, rc::MAX_ARM_2_JOINT, 0.0, 1.17, -1.53, rc::MAX_ARM_6_JOINT, 0.0};
            robot_controller_client_.sendGoalAndWait(secure_pose_goal);

            // Send start feedback
            feedback_.task_state = "Remove all collision objects";
            as_.publishFeedback(feedback_);
            // Clear collision objects
            pick_place_goal.motion = "clear_collision_objects";
            pick_place_client_.sendGoal(pick_place_goal, boost::bind(&TaskStatusAction::pick_place_done_callback, this, _1, _2), NULL, NULL);
            pick_place_client_.waitForResult();
            // Send subroutine result
            feedback_.task_state = feedback_.pick_place_feedback.pick_place_state;
            as_.publishFeedback(feedback_);

            // Send start feedback
            feedback_.task_state = "Move to place docking pose";
            as_.publishFeedback(feedback_);
            // Go from pick position to place position
            orientation_angle_goal.angle = M_PI / 2.0;
            robot_controller_client_.sendGoalAndWait(orientation_angle_goal);
            straight_goal.distance = distance_y_docking_poses;
            robot_controller_client_.sendGoalAndWait(straight_goal);
            orientation_angle_goal.angle = i == 0 ? orientation_dockings_left : orientation_dockings_right;
            robot_controller_client_.sendGoalAndWait(orientation_angle_goal);

            // Send start feedback
            feedback_.task_state = "Apply table collision objects";
            as_.publishFeedback(feedback_);
            // Apply table collision objects
            pick_place_goal.motion = "apply_table_collision_objects";
            pick_place_client_.sendGoal(pick_place_goal, boost::bind(&TaskStatusAction::pick_place_done_callback, this, _1, _2), NULL, NULL);
            pick_place_client_.waitForResult();
            // Send subroutine result
            feedback_.task_state = feedback_.pick_place_feedback.pick_place_state;
            as_.publishFeedback(feedback_);

            // Send start feedback
            feedback_.task_state = "Apply AprilTag collision objects";
            as_.publishFeedback(feedback_);
            // Read current AprilTags detected
            ad::AprilTagsDetection place_apriltags_feedback(
                feedback_.apriltags_detection_feedback.srv_target_apriltag_ids,
                feedback_.apriltags_detection_feedback.target_apriltag_ids, feedback_.apriltags_detection_feedback.target_apriltag_poses, feedback_.apriltags_detection_feedback.target_apriltag_colors,
                feedback_.apriltags_detection_feedback.nontarget_apriltag_ids, feedback_.apriltags_detection_feedback.nontarget_apriltag_poses, feedback_.apriltags_detection_feedback.nontarget_apriltag_colors
            );
            // Target AprilTags detected
            std::map<int, std::pair<geometry_msgs::PoseStamped, std_msgs::ColorRGBA>> place_target_apriltags = place_apriltags_feedback.get_target_apriltags_detected();
            // Collect AprilTag objects to avoid
            apriltag_frame_ids = mb::get_frame_ids(std::regex("tag_.*"));
            pick_place_goal.apriltag_ids.clear();
            pick_place_goal.apriltag_pose_stamped.clear();
            pick_place_goal.apriltag_colors.clear();
            for(const std::string& apriltag_frame_id : apriltag_frame_ids) {
                // Avoid picked collision object
                int apriltag_id; std::sscanf(apriltag_frame_id.c_str(), "tag_%d", &apriltag_id);
                if(pick_place_goal.apriltag_id != apriltag_id) {
                    // Save AprilTag ID, frame and color
                    pick_place_goal.apriltag_ids.push_back(apriltag_id);
                    geometry_msgs::PoseStamped apriltag_frame_pose_stamped = mb::get_frame_pose_stamped_wrt_frame(apriltag_frame_id, "map");
                    pick_place_goal.apriltag_pose_stamped.push_back(apriltag_frame_pose_stamped);
                    std_msgs::ColorRGBA apriltag_color = place_target_apriltags[apriltag_id].second;
                    pick_place_goal.apriltag_colors.push_back(apriltag_color);
                }
            }
            // Apply AprilTag collision objects
            pick_place_goal.motion = "apply_apriltag_collision_objects";
            pick_place_client_.sendGoal(pick_place_goal, boost::bind(&TaskStatusAction::pick_place_done_callback, this, _1, _2), NULL, NULL);
            pick_place_client_.waitForResult();
            // Send subroutine result
            feedback_.task_state = feedback_.pick_place_feedback.pick_place_state;
            as_.publishFeedback(feedback_);

            // Send start feedback
            feedback_.task_state = "Place the object";
            as_.publishFeedback(feedback_);
            // Place object in straight line
            pick_place_goal.motion = "place_object";
            // Select feasible place location
            for(size_t j = 0; j < place_locations.size(); ++j) {
                size_t k = i == 0 ? j : place_locations.size() - (j + 1);
                if(is_occupied[k])
                    continue;

                pick_place_goal.place_location = place_locations[k];
                pick_place_client_.sendGoal(pick_place_goal, boost::bind(&TaskStatusAction::pick_place_done_callback, this, _1, _2), NULL, NULL);
                pick_place_client_.waitForResult();

                // Place done 
                if(feedback_.pick_place_feedback.is_completed) {
                    // Save placed object
                    placed_apriltags.push_back(pick_place_goal.apriltag_id);
                    is_occupied[k] = true;
                    // Send result feedback
                    color = ts::get_color_string(pick_target_apriltags[pick_place_goal.apriltag_id].second);
                    feedback_.task_state = "Object with TAG-" + std::to_string(pick_place_goal.apriltag_id) + " (" + color + ") placed in location " + std::to_string(k + 1);
                    as_.publishFeedback(feedback_);
                    break;
                }
            }
            // Apply recovery place when not completed
            if(! feedback_.pick_place_feedback.is_completed) {
                pick_place_goal.motion = "recovery_place_object";
                pick_place_client_.sendGoal(pick_place_goal, boost::bind(&TaskStatusAction::pick_place_done_callback, this, _1, _2), NULL, NULL);
                pick_place_client_.waitForResult();
                // Save recovery placed object
                recovery_placed_apriltags.push_back(pick_place_goal.apriltag_id);
                // Send result feedback
                color = ts::get_color_string(pick_target_apriltags[pick_place_goal.apriltag_id].second);
                feedback_.task_state = "Object with TAG-" + std::to_string(pick_place_goal.apriltag_id) + " (" + color + ") placed in recovery location";
                as_.publishFeedback(feedback_);
            }
            // Send subroutine result
            feedback_.task_state = feedback_.pick_place_feedback.pick_place_state;
            as_.publishFeedback(feedback_);
       
            // Send start feedback
            feedback_.task_state = "Move arm to the secure pose";
            as_.publishFeedback(feedback_);
            // Move to pick secure pose
            robot_controller_client_.sendGoalAndWait(secure_pose_goal);

            // Send start feedback
            feedback_.task_state = "Remove all collision objects";
            as_.publishFeedback(feedback_);
            // Clear collision objects
            pick_place_goal.motion = "clear_collision_objects";
            pick_place_client_.sendGoal(pick_place_goal, boost::bind(&TaskStatusAction::pick_place_done_callback, this, _1, _2), NULL, NULL);
            pick_place_client_.waitForResult();
            // Send subroutine result
            feedback_.task_state = feedback_.pick_place_feedback.pick_place_state;
            as_.publishFeedback(feedback_);

            // Send start feedback
            feedback_.task_state = "Move back to pick docking pose";
            as_.publishFeedback(feedback_);
            // Go from place position to pick position
            orientation_angle_goal.angle = - M_PI / 2.0;
            robot_controller_client_.sendGoalAndWait(orientation_angle_goal);
            robot_controller_client_.sendGoalAndWait(straight_goal);
            orientation_angle_goal.angle = i == 0 ? orientation_dockings_left : orientation_dockings_right;
            robot_controller_client_.sendGoalAndWait(orientation_angle_goal);
        }

        // Send start feedback
        feedback_.task_state = "Remove all collision objects";
        as_.publishFeedback(feedback_);
        // Clear collision objects
        pick_place_goal.motion = "clear_collision_objects";
        pick_place_client_.sendGoal(pick_place_goal, boost::bind(&TaskStatusAction::pick_place_done_callback, this, _1, _2), NULL, NULL);
        pick_place_client_.waitForResult();
        // Send subroutine result
        feedback_.task_state = feedback_.pick_place_feedback.pick_place_state;
        as_.publishFeedback(feedback_);

        // Send start feedback
        feedback_.task_state = "Prepare the robot in home pose";
        as_.publishFeedback(feedback_);
        // Robot to home position
        ir2425_group_15::RobotControllerGoal home_pose_goal;
        home_pose_goal.motion = "home_pose";
        robot_controller_client_.sendGoalAndWait(home_pose_goal);

        // End pick-place
        if(i == 1) {
            // Send start feedback
            feedback_.task_state = "End pick-place routine on both sides";
            as_.publishFeedback(feedback_);
            break;
        }

        // Send start feedback
        feedback_.task_state = "Change pick-place side";
        as_.publishFeedback(feedback_);
        // Navigate to the other side
        orientation_angle_goal.angle = - M_PI / 2.0;
        robot_controller_client_.sendGoalAndWait(orientation_angle_goal);
        straight_goal.distance = 0.9;
        robot_controller_client_.sendGoalAndWait(straight_goal);
        orientation_angle_goal.angle = M_PI;
        robot_controller_client_.sendGoalAndWait(orientation_angle_goal);
        straight_goal.distance = fabs(docking_pose_pick_left.pose.position.x - docking_pose_pick_right.pose.position.x);
        robot_controller_client_.sendGoalAndWait(straight_goal);
        orientation_angle_goal.angle = M_PI / 2.0;
        robot_controller_client_.sendGoalAndWait(orientation_angle_goal);
        straight_goal.distance = 0.9;
        robot_controller_client_.sendGoalAndWait(straight_goal);
        orientation_angle_goal.angle = orientation_dockings_right;
        robot_controller_client_.sendGoalAndWait(orientation_angle_goal);
    }

    // Cancel AprilTags detection goal if not reached
    if(! is_apriltags_detection_succeeded()) {
        apriltags_detection_client_.cancelGoal();
        apriltags_detection_client_.waitForResult();
    }

    // Save AprilTag IDs of placed objects in result
    std::string placed_ids = "Placed objects have ";
    for(int placed_apriltag : placed_apriltags)
        placed_ids = placed_ids + "TAG-" + std::to_string(placed_apriltag) + ", ";
    if(recovery_placed_apriltags.empty()) {
        placed_ids += "\b\b ";
    }
    else {
        for(int recovery_placed_apriltag : recovery_placed_apriltags)
            placed_ids += "TAG-" + std::to_string(recovery_placed_apriltag) + " (recovery), ";
        placed_ids += "\b\b ";
    }
    result_.task_state = placed_ids;
    // Save task result
    result_.robot_navigation_result.robot_state = "Stopped in home pose";
    result_.is_completed = placed_apriltags.size() >= goal->n_objects;

    // Publish the task status result
    success ? as_.setSucceeded(result_) : as_.setPreempted(result_);

    return;
}

/* Done callback function for robot navigation */
void ts::TaskStatusAction::robot_navigation_done_callback(const actionlib::SimpleClientGoalState& robot_navigation_state, const ir2425_group_15::RobotNavigationResultConstPtr& robot_navigation_result) {
    result_.robot_navigation_result = *robot_navigation_result;
}

/* Done callback function for AprilTags detection */
void ts::TaskStatusAction::apriltags_detection_done_callback(const actionlib::SimpleClientGoalState& apriltags_detection_state, const ir2425_group_15::AprilTagsDetectionResultConstPtr& apriltags_detection_result) {    
    result_.apriltags_detection_result = *apriltags_detection_result;
}

/* Done callback function for pick place */
void ts::TaskStatusAction::pick_place_done_callback(const actionlib::SimpleClientGoalState& pick_place_state, const ir2425_group_15::PickPlaceResultConstPtr& pick_place_result) {
    // Save pick-place result in task feedback
    feedback_.pick_place_feedback = *pick_place_result;
}

/* Feedback callback function printing the robot navigation status */
void ts::TaskStatusAction::robot_navigation_status_feedback_callback(const ir2425_group_15::RobotNavigationFeedbackConstPtr& robot_navigation_feedback) {
    feedback_.robot_navigation_feedback = *robot_navigation_feedback;
}

/* Feedback callback function printing the AprilTags detection status */
void ts::TaskStatusAction::apriltags_detection_status_feedback_callback(const ir2425_group_15::AprilTagsDetectionFeedbackConstPtr& apriltags_detection_feedback) {
    feedback_.apriltags_detection_feedback = *apriltags_detection_feedback;
}

/* Function to check if robot navigation is succeeded */
bool ts::TaskStatusAction::is_robot_navigation_succeeded() {
    return robot_navigation_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

/* Function to check if AprilTags detection is succeeded */
bool ts::TaskStatusAction::is_apriltags_detection_succeeded() {
    return apriltags_detection_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

/* Function to check if pick place is succeeded */
bool ts::TaskStatusAction::is_pick_place_succeeded() {
    return pick_place_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

/* Function waiting for the robot navigation result */
bool ts::TaskStatusAction::wait_robot_navigation_result() {
    // Initial state
    bool success = true;
    
    // Wait the end of the robot navigation
    std::string robot_state;
    while(! is_robot_navigation_succeeded()) {
        // Check if preempt is requested by the client
        if(as_.isPreemptRequested()) {
            // Cancel action clients goal
            robot_navigation_client_.cancelGoal();

            // Update the action state
            success = false;
            break;
        }

        // Send intermediate feedback
        if(feedback_.robot_navigation_feedback.robot_state != robot_state) {
            robot_state = feedback_.robot_navigation_feedback.robot_state;
            as_.publishFeedback(feedback_);
        }
    }

    // Cancel robot navigation goal if not reached
    if(! is_robot_navigation_succeeded()) {
        robot_navigation_client_.cancelGoal();
        robot_navigation_client_.waitForResult();
    }

    return success;
}

/* Function waiting for the robot navigation result */
bool ts::TaskStatusAction::wait_apriltags_detection_start() {
    // Initial state
    bool success = false;
    
    // Wait AprilTags detection start
    while(! success) {
        if(feedback_.apriltags_detection_feedback.ready)
            success = true;
    }

    return success;
}