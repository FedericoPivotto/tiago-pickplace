/* Standard libraries */
#include <gazebo_ros_link_attacher/Attach.h>

/* User-defined libraries */
#include <pick_place_action.h>
#include <message_builder.h>

// PickPlace action constructor
pp::PickPlaceAction::PickPlaceAction(std::string name) : as_(nh_, name, boost::bind(&pp::PickPlaceAction::pick_place_callback, this, _1), false), action_name_(name), arm_move_group_("arm"), arm_torso_move_group_("arm_torso"), gripper_move_group_("gripper"), robot_controller_client_("robot_controller_action", true) {
    // Start the pick place action
    as_.start();

    // Register Gazebo link attacher and detacher
    gazebo_link_attach_client_ = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    gazebo_link_detach_client_ = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

    // Wait robot controller server
    robot_controller_client_.waitForServer();
}

/* Callback function describing the action for the pick place */
void pp::PickPlaceAction::pick_place_callback(const ir2425_group_15::PickPlaceGoalConstPtr& goal) {
    // Action initial state
    bool success = true;

    // Execute selected motion
    if(goal->motion == "apply_table_collision_objects") {
        // Subroutine
        this->apply_table_collision_objects();
    }
    else if(goal->motion == "clear_collision_objects") {
        // Subroutine
        this->clear_collision_objects();
    }
    else if(goal->motion == "apply_apriltag_collision_objects") {
        // Subroutine
        this->apply_apriltag_collision_objects(goal->apriltag_ids, goal->apriltag_pose_stamped, goal->apriltag_colors);
    }
    else if(goal->motion == "get_apriltag_collision_object_id") {
        // Subroutine
        this->get_apriltag_collision_object_id(goal->apriltag_id);
    }
    else if(goal->motion == "approach_object") {
        // Get collision object
        this->get_apriltag_collision_object_id(goal->apriltag_id);
        // Subroutine
        this->approach_object(result_.apriltag_id);
    }
    else if(goal->motion == "pick_object") {
        // Get collision object
        this->get_apriltag_collision_object_id(goal->apriltag_id);
        // Subroutine
        this->pick_object(result_.apriltag_id);
        // Recovery
        if(! result_.is_completed)
            this->recovery_pick_object();
    }
    else if(goal->motion == "leave_object") {
        // Get collision object
        this->get_apriltag_collision_object_id(goal->apriltag_id);
        // Subroutine
        this->leave_object(result_.apriltag_id);
        // Recovery
        if(! result_.is_completed)
            this->recovery_leave_object(goal->apriltag_id);
    }
    else if(goal->motion == "place_object") {
        // Subroutine
        this->place_object(goal->apriltag_id, goal->place_location);
    }
    else if(goal->motion == "recovery_place_object") {
        // Subroutine
        this->recovery_place_object(goal->apriltag_id);
    }
    else {
        success = false;
        result_.is_completed = success;
        result_.pick_place_state = "Pick-place subroutine not implemented";
    }

    // Publish the robot control result
    success ? as_.setSucceeded(result_) : as_.setPreempted(result_);

    return;
}

/* Function allowing the end-effector to approach the given AprilTag object */
void pp::PickPlaceAction::approach_object(std::string collision_object_id) {
    // Open gripper
    ir2425_group_15::RobotControllerGoal open_gripper_goal;
    open_gripper_goal.motion = "open_gripper";
    robot_controller_client_.sendGoalAndWait(open_gripper_goal);

    // Get collision object pose stamped
    std::map<std::string, moveit_msgs::CollisionObject> collision_objects = planning_scene_.getObjects(std::vector<std::string>{collision_object_id});
    geometry_msgs::PoseStamped collision_object_pose = mb::get_pose_stamped(mb::get_header(0, ros::Time::now(), "map"), collision_objects[collision_object_id].primitive_poses[0]);

    // Set end-effector link
    arm_move_group_.setEndEffectorLink("gripper_grasping_frame_Z");
    
    // Approach pose
    geometry_msgs::PoseStamped approach_pose = collision_object_pose;
    approach_pose.pose.position.z += pp::APPROACH_DISPLACEMENT;
    approach_pose.pose.orientation = mb::get_frame_pose_stamped_wrt_frame("gripper_grasping_frame_Z", "map").pose.orientation;

    // Set target pose
    arm_move_group_.setPoseTarget(approach_pose);

    // Set planning parameters
    arm_move_group_.setPlanningTime(30);
    arm_move_group_.setNumPlanningAttempts(5);
    // Compute plan and check pose reachability
    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    bool is_reachable = arm_move_group_.plan(approach_plan) == moveit::core::MoveItErrorCode::SUCCESS;

    // Subroutine status result
    result_.is_completed = is_reachable;

    // Cancel pose not reachable
    if(! is_reachable) {
        // Subroutine result
        result_.pick_place_state = "Subroutine approach object not completed";
        return;
    }

    // Move to target pose
    arm_move_group_.move();

    // Subroutine result
    result_.pick_place_state = "Subroutine approach object completed";
}

/* Function allowing the end-effector to pick the given object */
void pp::PickPlaceAction::pick_object(std::string collision_object_id) {
    // Set pick-table as support surface
    arm_move_group_.setSupportSurfaceName("table_2");

    // Set end-effector link
    arm_move_group_.setEndEffectorLink("gripper_grasping_frame_Z");

    // Open gripper
    ir2425_group_15::RobotControllerGoal open_gripper_goal;
    open_gripper_goal.motion = "open_gripper";
    robot_controller_client_.sendGoalAndWait(open_gripper_goal);

    // Get collision object pose stamped
    std::map<std::string, moveit_msgs::CollisionObject> collision_objects = planning_scene_.getObjects(std::vector<std::string>{collision_object_id});
    geometry_msgs::PoseStamped collision_object_pose = mb::get_pose_stamped(mb::get_header(0, ros::Time::now(), "map"), collision_objects[collision_object_id].primitive_poses[0]);
    
    // Set pick pose stamped
    geometry_msgs::PoseStamped pick_pose = collision_object_pose;
    pick_pose.pose.position.z += pp::PICK_DISPLACEMENT;
    pick_pose.pose.orientation = mb::get_frame_pose_stamped_wrt_frame("gripper_grasping_frame_Z", "map").pose.orientation;

    // Set target pose
    arm_move_group_.setPoseTarget(pick_pose);

    // Set planning parameters
    arm_move_group_.setPlanningTime(30);
    arm_move_group_.setNumPlanningAttempts(5);
    arm_move_group_.setGoalOrientationTolerance(M_PI / 54);
    // Compute plan and check pose reachability
    moveit::planning_interface::MoveGroupInterface::Plan pick_plan;
    bool is_reachable = arm_move_group_.plan(pick_plan) == moveit::core::MoveItErrorCode::SUCCESS;

    // Subroutine status result
    result_.is_completed = is_reachable;

    // Cancel pose not reachable
    if(! is_reachable) {
        // Subroutine result
        result_.pick_place_state = "Subroutine pick object not completed";
        return;
    }

    // Move to target pose
    arm_move_group_.move();

    // Close gripper
    ir2425_group_15::RobotControllerGoal close_gripper_goal;
    close_gripper_goal.motion = "close_gripper";
    robot_controller_client_.sendGoalAndWait(close_gripper_goal);

    // Link and Gazebo model names
    int apriltag_id; std::sscanf(collision_object_id.c_str(), "%*[^_]_%d", &apriltag_id);
    std::string object_model = pp::get_apriltag_object_gazebo_model(apriltag_id);
    std::string object_link = object_model + "_link";
    std::string tiago_model = "tiago";
    std::string tiago_link = "arm_7_link";
    // Attach object to robot
    gazebo_ros_link_attacher::Attach gazebo_ros_link_attach = mb::get_gazebo_link_attach_request(tiago_model, tiago_link, object_model, object_link);
    // Send service request and read service response
    if((! gazebo_link_attach_client_.call(gazebo_ros_link_attach)) || (! gazebo_ros_link_attach.response.ok)) {
        // Subroutine result
        result_.is_completed = false;
        result_.pick_place_state = "Subroutine pick object not completed";
        return;
    }
    
    // Subroutine result
    result_.pick_place_state = "Subroutine pick object completed";
}

/* Function allowing to recovery from failed pick object */
void pp::PickPlaceAction::recovery_pick_object() {
    // Move to secure pose
    ir2425_group_15::RobotControllerGoal secure_pose_goal;
    secure_pose_goal.motion = "arm_control";
    secure_pose_goal.arm_joints = std::vector<double>{1.50, rc::MAX_ARM_2_JOINT, 0.0, 1.17, -1.53, rc::MAX_ARM_6_JOINT, 0.0};
    robot_controller_client_.sendGoalAndWait(secure_pose_goal);
}

/* Function allowing the end-effector to leave the given AprilTag object */
void pp::PickPlaceAction::leave_object(std::string collision_object_id) {
    // Get gripper pose stamped
    geometry_msgs::PoseStamped gripper_pose = mb::get_frame_pose_stamped_wrt_frame("gripper_grasping_frame_Z", "map");

    // Set end-effector link
    arm_move_group_.setEndEffectorLink("gripper_grasping_frame_Z");
    
    // Approach pose
    geometry_msgs::PoseStamped leave_pose = gripper_pose;
    leave_pose.pose.position.z += pp::LEAVE_DISPLACEMENT;

    // Set target pose
    arm_move_group_.setPoseTarget(leave_pose);

    // Set planning parameters
    arm_move_group_.setPlanningTime(30);
    arm_move_group_.setNumPlanningAttempts(5);
    // Compute plan and check pose reachability
    moveit::planning_interface::MoveGroupInterface::Plan leave_plan;
    bool is_reachable = arm_move_group_.plan(leave_plan) == moveit::core::MoveItErrorCode::SUCCESS;

    // Subroutine status result
    result_.is_completed = is_reachable;

    // Cancel pose not reachable
    if(! is_reachable) {
        // Subroutine result
        result_.pick_place_state = "Subroutine leave object not completed";
        return;
    }

    // Move to target pose
    arm_move_group_.move();

    // Subroutine result
    result_.pick_place_state = "Subroutine leave object completed";
}

/* Function allowing the end-effector to recover from not leaveable AprilTag object */
void pp::PickPlaceAction::recovery_leave_object(int apriltag_id) {
    // Link and Gazebo model names
    std::string object_model = pp::get_apriltag_object_gazebo_model(apriltag_id);
    std::string object_link = object_model + "_link";
    std::string tiago_model = "tiago";
    std::string arm_7_link = "arm_7_link";
    // Detach object from robot
    gazebo_ros_link_attacher::Attach gazebo_ros_link_detach = mb::get_gazebo_link_attach_request(tiago_model, arm_7_link, object_model, object_link);
    // Send service request and read service response
    if((! gazebo_link_detach_client_.call(gazebo_ros_link_detach)) || (! gazebo_ros_link_detach.response.ok))
        return;
    
    // Open gripper
    ir2425_group_15::RobotControllerGoal open_gripper_goal;
    open_gripper_goal.motion = "open_gripper";
    robot_controller_client_.sendGoalAndWait(open_gripper_goal);

    // Move to secure pose
    ir2425_group_15::RobotControllerGoal secure_pose_goal;
    secure_pose_goal.motion = "arm_control";
    secure_pose_goal.arm_joints = std::vector<double>{1.50, rc::MAX_ARM_2_JOINT, 0.0, 1.17, -1.53, rc::MAX_ARM_6_JOINT, 0.0};
    robot_controller_client_.sendGoalAndWait(secure_pose_goal);
}

/* Function allowing the end-effector to place the given object */
void pp::PickPlaceAction::place_object(int apriltag_id, geometry_msgs::PoseStamped place_location) {
    // Set place-table as support surface
    arm_move_group_.setSupportSurfaceName("table_1");

    // Set end-effector link
    arm_move_group_.setEndEffectorLink("gripper_grasping_frame_Z");

    // Set place pose stamped
    geometry_msgs::PoseStamped place_pose = place_location;
    place_pose.pose.position.z += pp::PLACE_DISPLACEMENT;
    place_pose.pose.orientation = mb::get_frame_pose_stamped_wrt_frame("gripper_grasping_frame_Z", "map").pose.orientation;

    // Set target pose
    arm_move_group_.setPoseTarget(place_pose);

    // Set planning parameters
    arm_move_group_.setPlanningTime(30);
    arm_move_group_.setNumPlanningAttempts(5);
    arm_move_group_.setGoalOrientationTolerance(M_PI / 18);
    // Compute plan and check pose reachability
    moveit::planning_interface::MoveGroupInterface::Plan place_plan;
    bool is_reachable = arm_move_group_.plan(place_plan) == moveit::core::MoveItErrorCode::SUCCESS;

    // Subroutine status result
    result_.is_completed = is_reachable;

    // Cancel pose not reachable
    if(! is_reachable) {
        // Subroutine result
        result_.pick_place_state = "Subroutine place object not completed";
        return;
    }

    // Move to target pose
    arm_move_group_.move();

    // Link and Gazebo model names
    std::string object_model = pp::get_apriltag_object_gazebo_model(apriltag_id);
    std::string object_link = object_model + "_link";
    std::string tiago_model = "tiago";
    std::string arm_7_link = "arm_7_link";
    // Detach object from robot
    gazebo_ros_link_attacher::Attach gazebo_ros_link_detach = mb::get_gazebo_link_attach_request(tiago_model, arm_7_link, object_model, object_link);
    // Send service request and read service response
    if((! gazebo_link_detach_client_.call(gazebo_ros_link_detach)) || (! gazebo_ros_link_detach.response.ok)) {
        // Subroutine result
        result_.is_completed = false;
        result_.pick_place_state = "Subroutine pick object not completed";
        return;
    }
    
    // Open gripper
    ir2425_group_15::RobotControllerGoal open_gripper_goal;
    open_gripper_goal.motion = "open_gripper";
    robot_controller_client_.sendGoalAndWait(open_gripper_goal);
    
    // Subroutine result
    result_.pick_place_state = "Subroutine place object completed";
}

/* Function allowing the end-effector to recover from not placeable AprilTag object */
void pp::PickPlaceAction::recovery_place_object(int apriltag_id) {
    // Link and Gazebo model names
    std::string object_model = pp::get_apriltag_object_gazebo_model(apriltag_id);
    std::string object_link = object_model + "_link";
    std::string tiago_model = "tiago";
    std::string arm_7_link = "arm_7_link";
    // Detach object from robot
    gazebo_ros_link_attacher::Attach gazebo_ros_link_detach = mb::get_gazebo_link_attach_request(tiago_model, arm_7_link, object_model, object_link);
    // Send service request and read service response
    if((! gazebo_link_detach_client_.call(gazebo_ros_link_detach)) || (! gazebo_ros_link_detach.response.ok))
        return;

    // Open gripper
    ir2425_group_15::RobotControllerGoal open_gripper_goal;
    open_gripper_goal.motion = "open_gripper";
    robot_controller_client_.sendGoalAndWait(open_gripper_goal);
}

/* Function to apply table collision objetcs */
void pp::PickPlaceAction::apply_table_collision_objects() {
    // Add table collision objects
    std::vector<moveit_msgs::CollisionObject> table_collision_objects = pp::get_table_collision_objects();
    planning_scene_.applyCollisionObjects(table_collision_objects);

    // Save collision object IDs
    for(moveit_msgs::CollisionObject& table_collision_object : table_collision_objects)
        result_.table_collision_object_ids.push_back(table_collision_object.id);
    
    // Subroutine status
    result_.is_completed = true;
    result_.pick_place_state = "Subroutine apply table collision objects completed";
}

/* Function to apply AprilTag collision objetcs */
void pp::PickPlaceAction::apply_apriltag_collision_objects(std::vector<int> apriltag_ids, std::vector<geometry_msgs::PoseStamped> apriltag_pose_stamped, std::vector<std_msgs::ColorRGBA> apriltag_colors) {
    // Add AprilTags collision objects
    std::vector<moveit_msgs::CollisionObject> apriltag_collision_objects;
    std::vector<moveit_msgs::ObjectColor> apriltag_object_colors;
    for(size_t i = 0; i < apriltag_ids.size(); ++i) {
        // Construct collision object 
        moveit_msgs::CollisionObject apriltag_collision_object = pp::get_apriltag_collision_object(apriltag_ids[i], apriltag_pose_stamped[i]);
        apriltag_collision_objects.push_back(apriltag_collision_object);
        // Construct object color
        moveit_msgs::ObjectColor apriltag_object_color = pp::get_apriltag_object_color(apriltag_ids[i], apriltag_collision_object.id, apriltag_colors[i]);
        apriltag_object_colors.push_back(apriltag_object_color);
    }
    planning_scene_.applyCollisionObjects(apriltag_collision_objects, apriltag_object_colors);

    // Save collision object IDs
    for(moveit_msgs::CollisionObject& apriltag_collision_object : apriltag_collision_objects)
        result_.collision_object_ids.push_back(apriltag_collision_object.id);

    // Subroutine status
    result_.is_completed = true;
    result_.pick_place_state = "Subroutine apply AprilTag collision objects completed";
}

/* Function to remove all the collision objetcs */
void pp::PickPlaceAction::clear_collision_objects() {
    // Remove all collision objects
    std::map<std::string, moveit_msgs::CollisionObject> collision_objects = planning_scene_.getObjects();
    std::vector<moveit_msgs::CollisionObject> objects_remove;
    for (auto& [id, collision_object] : collision_objects) {
        collision_object.operation = moveit_msgs::CollisionObject::REMOVE;
        objects_remove.push_back(collision_object);
    }
    planning_scene_.applyCollisionObjects(objects_remove);

    // Subroutine status
    result_.is_completed = true;
    result_.pick_place_state = "Subroutine remove all collision objects completed";
}

/* Function to get AprilTag ID from corresponding collision object */
void pp::PickPlaceAction::get_apriltag_collision_object_id(int apriltag_id) {
    // Get AprilTag collision objects
    std::map<std::string, moveit_msgs::CollisionObject> collision_objects = planning_scene_.getObjects();
    
    // Regex for hexagons, triangles and cubes
    std::regex hexagon_pattern("hexagon_.*"), cube_pattern("cube_.*"), triangle_pattern("triangle_.*");
    
    // Filter hexagon, cube, triangle collision object IDs
    std::map<int, std::string> apriltag_ids;
    for(auto const& [id, collision_object] : collision_objects) {
        // Match AprilTag ID
        int match_apriltag_id;
        if(std::regex_match(id, hexagon_pattern))
            std::sscanf(id.c_str(), "hexagon_%d", &match_apriltag_id);
        else if(std::regex_match(id, cube_pattern))
            std::sscanf(id.c_str(), "cube_%d", &match_apriltag_id);
        else if(std::regex_match(id, triangle_pattern))
            std::sscanf(id.c_str(), "triangle_%d", &match_apriltag_id);
        else
            continue;

        // Save AprilTag ID
        apriltag_ids[match_apriltag_id] = id;
    }

    // Resulting ID
    result_.apriltag_id = apriltag_ids.empty() ? "" : apriltag_ids[apriltag_id];

    // Subroutine status
    result_.is_completed = true;
    result_.pick_place_state = "Subroutine get apriltag collision object ID completed";

    return;
}