/* Standard libraries */
#include <ros/package.h>
#include <tiago_iaslab_simulation/Coeffs.h>

/* User-defined libraries */
#include <task_status_action.h>

/* Constants */
constexpr int N_APRILTAG_IDS = 9;
constexpr int MIN_OBJECTS_TO_PLACE = 3;

int main(int argc, char **argv) {
    // Node "start_task_node" with arguments
    ros::init(argc, argv, "start_task_node");

    // Send an AprilTags request to the server "get_straightline_node" through the service "straight_line_srv"
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<tiago_iaslab_simulation::Coeffs>("straight_line_srv");

    // Service for AprilTag IDs
    tiago_iaslab_simulation::Coeffs srv;
    
    // Send service request 
    srv.request.ready = true;
    if(! client.call(srv)) {
        ROS_INFO_STREAM("Failed to call");
        return -1;
    }
    
    // Save the target straight line coefficients
    float coeff_m = srv.response.coeffs[0], coeff_q = srv.response.coeffs[1];

    // Initialize task status action client
    ts::TaskStatusClient action_client("task_status_action", true); 
    action_client.waitForServer();

    // Prepare the task status goal with the target AprilTag IDs
    ir2425_group_15::TaskStatusGoal goal;
    for(size_t i = 1; i <= N_APRILTAG_IDS; ++i)
        goal.apriltags_detection_goal.srv_target_apriltag_ids.push_back(i);
    goal.apriltags_detection_goal.n_srv_target_apriltag_ids = goal.apriltags_detection_goal.srv_target_apriltag_ids.size();

    // Prepare the task status goal with the waypoint path
    nav_msgs::Path waypoint_path = rn::get_waypoint_path(ros::package::getPath("ir2425_group_15"), "config/", "waypoints", "yaml");
    ir2425_group_15::RobotNavigationGoal waypoints_goal;
    waypoints_goal.routine = "reach_waypoints";
    waypoints_goal.waypoint_path = waypoint_path;
    waypoints_goal.n_waypoints = waypoint_path.poses.size();
    goal.robot_navigation_goal = waypoints_goal;

    // Prepare the task status goal with the straight line
    goal.n_objects = MIN_OBJECTS_TO_PLACE;
    goal.pick_place_goal.m = coeff_m;
    goal.pick_place_goal.q = coeff_q;

    // Print the target AprilTag IDs and straight line coefficients
    std::string apriltag_ids_target;
    for(const auto& apriltag_id_target: goal.apriltags_detection_goal.srv_target_apriltag_ids)
        apriltag_ids_target = apriltag_ids_target + std::to_string(apriltag_id_target) + ", ";
    ROS_INFO_STREAM("[start_task]:\nTarget AprilTag IDs: " << apriltag_ids_target << "\b\b " << "\nTarget Straight Line: m = " << std::to_string(coeff_m) << ", q = " << std::to_string(coeff_q) << "\n");

    // Send the task status goal and wait for result
    action_client.sendGoal(goal, &ts::task_status_done_callback, NULL, &ts::task_status_feedback_callback);
    action_client.waitForResult();
    
    // Spin the task status callback function
    ros::spin();

    return 0;
}