/* Standard libraries */
#include <ros/ros.h>

/* User-defined libraries */
#include <robot_navigation_action.h>

// RobotNavigation action constructor
rn::RobotNavigationAction::RobotNavigationAction(std::string name) : as_(nh_, name, boost::bind(&RobotNavigationAction::navigate_robot_callback, this, _1), false), action_name_(name), robot_controller_client_("robot_controller_action", true), play_motion_client_("play_motion", true) {
    // Start the robot navigation action
    as_.start();

    // Wait robot controller server
    robot_controller_client_.waitForServer();
}

/* Callback function describing the action for the robot navigation */
void rn::RobotNavigationAction::navigate_robot_callback(const ir2425_group_15::RobotNavigationGoalConstPtr& goal) {
    // Action initial state
    bool success = true;

    // Execute selected motion
    if(goal->routine == "reach_waypoints")
        this->reach_waypoints(goal->waypoint_path, goal->n_waypoints);
    else {
        success = false;
        result_.robot_state = "Navigation subroutine not implemented";
    }

    // Publish the robot navigation result
    success ? as_.setSucceeded(result_) : as_.setPreempted(result_);

    return;
}

/* Done callback function for robot navigation */
void rn::RobotNavigationAction::robot_controller_done_callback(const actionlib::SimpleClientGoalState& robot_controller_state, const ir2425_group_15::RobotControllerResultConstPtr& robot_controller_result) {
    feedback_.robot_controller_result = *robot_controller_result;
}

/* Function moving the robot to through the waypoints */
void rn::RobotNavigationAction::reach_waypoints(nav_msgs::Path waypoint_path, int n_waypoints) {
    // Action initial state
    bool success = true;

    // Publish start feedback
    feedback_.robot_state = "Subroutine reach waypoints started";
    as_.publishFeedback(feedback_);

    // Corridor navigation
    ir2425_group_15::RobotControllerGoal corridor_goal;
    corridor_goal.motion = "corridor";
    corridor_goal.waypoint = waypoint_path.poses[0];
    // Update robot navigation feedback with the robot control result
    robot_controller_client_.sendGoal(corridor_goal, boost::bind(&RobotNavigationAction::robot_controller_done_callback, this, _1, _2), NULL, NULL);
    robot_controller_client_.waitForResult();
    
    // Robot navigation steps
    for(size_t i = 1; i < n_waypoints; ++i) {
        // Check if preempt is requested by the client
        if(as_.isPreemptRequested()) {
            // Set action failure
            success = false;
            break;
        }

        // Final navigation
        ir2425_group_15::RobotControllerGoal final_goal;
        final_goal.motion = "final";
        final_goal.waypoint = waypoint_path.poses[i];
        // Update robot navigation feedback with the robot control result
        robot_controller_client_.sendGoal(final_goal, boost::bind(&RobotNavigationAction::robot_controller_done_callback, this, _1, _2), NULL, NULL);
        robot_controller_client_.waitForResult();
    }

    // Prepare the robot navigation result
    result_.is_completed = success;
    result_.robot_state = success ? "Subroutine reach waypoints completed" : "Subroutine reach waypoints preempted";

    // Publish final feedback
    feedback_.robot_state = result_.robot_state;
    as_.publishFeedback(feedback_);

    return;
}