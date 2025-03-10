/* Standard libraries */
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/* User-defined libraries */
#include <robot_controller_action.h>
#include <message_builder.h>

// RobotController action constructor
rc::RobotControllerAction::RobotControllerAction(std::string name) : as_(nh_, name, boost::bind(&rc::RobotControllerAction::control_robot_callback, this, _1), false), action_name_(name), move_base_client_("move_base", true), play_motion_client_("play_motion", true), torso_follow_joint_trajectory_("/torso_controller/follow_joint_trajectory", true), head_follow_joint_trajectory_("/head_controller/follow_joint_trajectory", true), gripper_follow_joint_trajectory_("/gripper_controller/follow_joint_trajectory", true), arm_follow_joint_trajectory_("/arm_controller/follow_joint_trajectory", true), is_corridor_(false) {
    // Start the robot controller action
    as_.start();

    // Subscribe to the topic "scan"
    laser_subscriber_ = nh_.subscribe("scan", 1, &rc::RobotControllerAction::corridor_check_callback, this);
    // Publish in the topic "mobile_base_controller/cmd_vel"
    velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>("mobile_base_controller/cmd_vel", 1);

    // Wait action servers
    move_base_client_.waitForServer();
    play_motion_client_.waitForServer();
    torso_follow_joint_trajectory_.waitForServer();
    head_follow_joint_trajectory_.waitForServer();
    gripper_follow_joint_trajectory_.waitForServer();
    arm_follow_joint_trajectory_.waitForServer();
}

/* Callback function describing the action for the robot controller */
void rc::RobotControllerAction::control_robot_callback(const ir2425_group_15::RobotControllerGoalConstPtr& goal) {
    // Action initial state
    bool success = true;

    // Execute selected motion
    if(goal->motion == "initial")
        this->initial_navigation();
    else if(goal->motion == "corridor")
        this->corridor_navigation(goal->waypoint);
    else if(goal->motion == "final")
        this->final_navigation(goal->waypoint);
    else if(goal->motion == "prepare_grasp")
        this->prepare_grasp();
    else if(goal->motion == "torso_control")
        this->torso_control(goal->torso_lift_joint);
    else if(goal->motion == "head_control")
        this->head_control(goal->head_joints);
    else if(goal->motion == "gripper_control")
        this->gripper_control(goal->gripper_joints);
    else if(goal->motion == "arm_control")
        this->arm_control(goal->arm_joints);
    else if(goal->motion == "open_gripper")
        this->open_gripper();
    else if(goal->motion == "close_gripper")
        this->close_gripper();
    else if(goal->motion == "rotate_base") 
        this->rotate_base(goal->angle);
    else if(goal->motion == "move_straight") 
        this->move_straight(goal->distance);
    else if(goal->motion == "home_pose") 
        this->home_pose();
    else {
        success = false;
        result_.robot_state = "Controller subroutine not implemented";
    }

    // Publish the robot control result
    success ? as_.setSucceeded(result_) : as_.setPreempted(result_);

    return;
}

/* Callback function to read laser scan topic and check corridor */
void rc::RobotControllerAction::corridor_check_callback(const sensor_msgs::LaserScanConstPtr& msg) {
    // Setup thresholds for checking corridor
    float distance_check_threshold = 0.75;
    float corridor_side_threshold = 10;

    // Setup counter of close laser samples with respect to the robot
    int right_close_samples = 0;
    int left_close_samples = 0;

    // Number of samples to scan either on left or right side of the robot
    int side_samples_to_scan = 25;

    // Index in range list of msg representing right side and left side of the robot
    int right_side_index = (-(M_PI / 2.0) - msg->angle_min) / (msg->angle_increment);
    int left_side_index = ((M_PI / 2.0) - msg->angle_min) / (msg->angle_increment);

    // Scan current msg range list to right side
    for(int i = (-side_samples_to_scan / 2); i < (side_samples_to_scan / 2 + 1); i++) {
        if(msg->ranges[right_side_index + i] <= distance_check_threshold)
            right_close_samples += 1;
    }

    // Scan current msg range list to left side
    for(int i = (-side_samples_to_scan / 2); i < (side_samples_to_scan / 2 + 1); i++) {
        if(msg->ranges[left_side_index + i] <= distance_check_threshold)
            left_close_samples += 1;
    }

    // Check if robot is in corridor
    if((right_close_samples >= corridor_side_threshold) && (left_close_samples >= corridor_side_threshold)) {
        left_distance_ = msg->ranges[left_side_index];
        right_distance_ = msg->ranges[right_side_index];

        is_corridor_ = true;
    }
    else {
        is_corridor_ = false;
    }

    return;
}

/* Function moving the robot to the next goal in the initial room */
void rc::RobotControllerAction::initial_navigation() {
    // Perform complete rotation
    for(size_t i = 1; i <= 2; ++i) {
        // Send the play motion goal
        play_motion_msgs::PlayMotionGoal play_motion_goal = rc::get_play_motion_goal("inspect_surroundings");
        play_motion_client_.sendGoalAndWait(play_motion_goal);
        
        // Move the robot with constant velocity until in corridor
        geometry_msgs::Twist robot_vel;
        double rotation_frequency = 0.2;
        robot_vel.angular.z = M_PI * rotation_frequency;
        ros::Rate rate(10);
        ros::Time start_time = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - start_time).toSec() < 5.0) {
            velocity_publisher_.publish(robot_vel);
            rate.sleep();
        }
        
        // Publish robot velocity equal to zero
        robot_vel.angular.z = 0;
        velocity_publisher_.publish(robot_vel);
    }

    // Prepare the robot control result
    result_.waypoint = mb::get_frame_pose_stamped_wrt_frame("base_footprint", "map");
    result_.robot_state = "Subroutine initial navigation completed";

    return;
}

/* Function moving the robot to the next goal in the corridor room */
void rc::RobotControllerAction::corridor_navigation(geometry_msgs::PoseStamped waypoint) {
    // Send the move base goal
    move_base_msgs::MoveBaseGoal goal = rc::get_move_base_goal(waypoint);
    move_base_client_.sendGoalAndWait(goal);
    
    // Set the check rate of corridor
    ros::Rate rate(10);

    // Move the robot with constant velocity until in corridor
    geometry_msgs::Twist robot_vel;
    robot_vel.linear.x = 0.4;

    bool is_initial = true;
    double initial_left_distance_, initial_right_distance_;
    while(is_corridor_) {
        // Initial corridor distances
        if(is_initial) {
            initial_left_distance_ = left_distance_;
            initial_right_distance_ = right_distance_;

            is_initial = false; 
        }

        // Wall collision avoidance
        double diff = (left_distance_ - initial_left_distance_) - (right_distance_ - initial_right_distance_);
        double factor = 0;
        if(diff > 0)
            factor = 1;
        else if (diff < 0)
            factor = -1;
        double rotational_velocity = 0.1;
        robot_vel.angular.z = factor * rotational_velocity;

        // Publish velocity
        velocity_publisher_.publish(robot_vel);
        rate.sleep();
    }

    // Publish robot velocity equal to zero
    robot_vel.linear.x = 0;
    velocity_publisher_.publish(robot_vel);

    // Prepare the robot control result
    result_.waypoint = mb::get_frame_pose_stamped_wrt_frame("base_footprint", "map");
    bool success = move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    result_.robot_state = success ? "Subroutine corridor navigation completed" : "Subroutine corridor navigation preempted";

    return;
}

/* Function moving the robot to the next goal in the final room */
void rc::RobotControllerAction::final_navigation(geometry_msgs::PoseStamped waypoint) {
    // Send the move base goal with 120 secconds timeout
    move_base_msgs::MoveBaseGoal move_base_goal = rc::get_move_base_goal(waypoint);
    move_base_client_.sendGoalAndWait(move_base_goal, ros::Duration(120));
    
    // Prepare the robot control result
    result_.waypoint = waypoint;
    bool success = move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    result_.robot_state = success ? "Subroutine final navigation step completed" : "Subroutine final navigation step preempted";

    return;
}

/* Function to inspect the surroundings and prepare the grasp */
void rc::RobotControllerAction::prepare_grasp() {
    // Torso control
    this->torso_control(rc::MAX_TORSO_LIFT_JOINT);

    // Arm pre-control
    this->arm_control(std::vector<double>{rc::MIN_ARM_1_JOINT, rc::MAX_ARM_2_JOINT, 0.0, 1.17, -1.53, rc::MAX_ARM_6_JOINT, 0.0});
    // Arm control
    this->arm_control(std::vector<double>{1.50, rc::MAX_ARM_2_JOINT, 0.0, 1.17, -1.53, rc::MAX_ARM_6_JOINT, 0.0});

    // Gripper control
    this->open_gripper();

    // Prepare the robot control result
    bool success = true;
    result_.waypoint = mb::get_frame_pose_stamped_wrt_frame("base_footprint", "map");
    result_.robot_state = success ? "Subroutine prepare grasp completed" : "Subroutine prepare grasp preempted";

    return;
}

/* Function to control the robot torso */
void rc::RobotControllerAction::torso_control(double torso_lift_joint) {
    // Send the torso control goal
    control_msgs::FollowJointTrajectoryGoal torso_control_goal = rc::get_torso_follow_joint_trajectory_goal(torso_lift_joint);
    torso_follow_joint_trajectory_.sendGoalAndWait(torso_control_goal);

    // Prepare the robot control result
    bool success = torso_follow_joint_trajectory_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    result_.waypoint = mb::get_frame_pose_stamped_wrt_frame("base_footprint", "map");
    result_.robot_state = success ? "Subroutine torso control completed" : "Subroutine torso control preempted";

    return;
}

/* Function to control the robot head */
void rc::RobotControllerAction::head_control(std::vector<double> head_joints) {
    // Send the head control goal
    control_msgs::FollowJointTrajectoryGoal head_control_goal = rc::get_head_follow_joint_trajectory_goal(head_joints[0], head_joints[1]);
    head_follow_joint_trajectory_.sendGoalAndWait(head_control_goal);

    // Prepare the robot control result
    bool success = head_follow_joint_trajectory_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    result_.waypoint = mb::get_frame_pose_stamped_wrt_frame("base_footprint", "map");
    result_.robot_state = success ? "Subroutine head control completed" : "Subroutine head control preempted";

    return;
}

/* Function to control the robot gripper */
void rc::RobotControllerAction::gripper_control(std::vector<double> gripper_joints) {
    // Send the gripper control goal
    control_msgs::FollowJointTrajectoryGoal gripper_control_goal = rc::get_gripper_follow_joint_trajectory_goal(gripper_joints[0], gripper_joints[1]);
    gripper_follow_joint_trajectory_.sendGoalAndWait(gripper_control_goal);

    // Prepare the robot control result
    bool success = gripper_follow_joint_trajectory_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    result_.waypoint = mb::get_frame_pose_stamped_wrt_frame("base_footprint", "map");
    result_.robot_state = success ? "Subroutine gripper control completed" : "Subroutine gripper control preempted";

    return;
}

/* Function to control the robot arm */
void rc::RobotControllerAction::arm_control(std::vector<double> arm_joints) {
    // Send the arm control goal
    control_msgs::FollowJointTrajectoryGoal arm_control_goal = rc::get_arm_follow_joint_trajectory_goal(arm_joints[0], arm_joints[1], arm_joints[2], arm_joints[3], arm_joints[4], arm_joints[5], arm_joints[6]);
    arm_follow_joint_trajectory_.sendGoalAndWait(arm_control_goal);

    // Prepare the robot control result
    bool success = arm_follow_joint_trajectory_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    result_.waypoint = mb::get_frame_pose_stamped_wrt_frame("base_footprint", "map");
    result_.robot_state = success ? "Subroutine arm control completed" : "Subroutine arm control preempted";

    return;
}

/* Function to open the gripper */
void rc::RobotControllerAction::open_gripper() {
    // Open gripper with controller
    this->gripper_control(std::vector<double>{rc::MAX_GRIPPER_LEFT_FINGER_JOINT, rc::MAX_GRIPPER_RIGHT_FINGER_JOINT});
}

/* Function to close the gripper */
void rc::RobotControllerAction::close_gripper() {
    // Close gripper with controller
    this->gripper_control(std::vector<double>{rc::MIN_GRIPPER_LEFT_FINGER_JOINT, rc::MIN_GRIPPER_RIGHT_FINGER_JOINT});
}

/* Function to rotate the robot around z-axis, giving in input a certain angle orientation to reach */
void rc::RobotControllerAction::rotate_base(double orientation_angle) {
    double epsilon = 0.01;
    geometry_msgs::Twist cmd;
    cmd.angular.z = 0.2;

    // Get current robot position
    geometry_msgs::PoseStamped current_pose = mb::get_frame_pose_stamped_wrt_frame("base_footprint", "map");
    double first_orientation = mb::get_rpy_from_quaternion(current_pose.pose.orientation).z;

    // Handle PI orientation angle, since -PI and +PI are equivalent
    if(fabs(orientation_angle - M_PI) <= epsilon || fabs(orientation_angle + M_PI) <= epsilon) {
        if(first_orientation < 0)
            orientation_angle = - M_PI;
        else
            orientation_angle = M_PI;
    }

    double angular_difference = orientation_angle - first_orientation;
    angular_difference = atan2(sin(angular_difference), cos(angular_difference));
    if(angular_difference < 0)
        cmd.angular.z = - 0.2;
    // Get transfor buffer between frames
    std_msgs::Header header = mb::get_header(1, ros::Time::now(), "map");

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    while(! tf_buffer.canTransform("map", "base_footprint", ros::Time(0)))
        ros::Duration(0.5).sleep();

    // Publish velocity commands for the required duration
    ros::Rate rate(100);
    while (ros::ok()) {
        // Get robot pose in map
        geometry_msgs::TransformStamped transformed = tf_buffer.lookupTransform("map", "base_footprint", ros::Time(0), ros::Duration(1.0));

        // Get current quaternion
        geometry_msgs::Quaternion orientation = mb::get_quaternion(transformed.transform.rotation.x, transformed.transform.rotation.y, transformed.transform.rotation.z, transformed.transform.rotation.w);

        // Get current orientation
        double current_orientation = mb::get_rpy_from_quaternion(orientation).z;

        if(std::abs(current_orientation - orientation_angle) <= epsilon) {
            // Stop the robot
            cmd.linear.x = 0.0;
            velocity_publisher_.publish(cmd);
            break;
        }

        velocity_publisher_.publish(cmd);
        rate.sleep();
    }

    // Stop the robot after rotation
    cmd.angular.z = 0.0;
    velocity_publisher_.publish(cmd);
    
    // Prepare the robot control result
    result_.waypoint = mb::get_frame_pose_stamped_wrt_frame("base_footprint", "map");
    result_.robot_state = "Subroutine rotate base completed";

    return;
}

/* Function to move straight the robot */
void rc::RobotControllerAction::move_straight(double distance) {
    // Velocity command
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.2;
    vel_msg.angular.z = 0.0;

    // Get current robot position
    geometry_msgs::PoseStamped current_pose = mb::get_frame_pose_stamped_wrt_frame("base_footprint", "map");

    // Get transfor buffer between frames
    std_msgs::Header header = mb::get_header(1, ros::Time::now(), "map");

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    while(! tf_buffer.canTransform("map", "base_footprint", ros::Time(0)))
        ros::Duration(0.5).sleep();

    ros::Rate rate(100);
    while(ros::ok()) {
        // Get robot pose in map
        geometry_msgs::TransformStamped transformed = tf_buffer.lookupTransform("map", "base_footprint", ros::Time(0), ros::Duration(1.0));

        geometry_msgs::Point position = mb::get_point(transformed.transform.translation.x, transformed.transform.translation.y, transformed.transform.translation.z);
        geometry_msgs::Quaternion orientation = mb::get_quaternion(transformed.transform.rotation.x, transformed.transform.rotation.y, transformed.transform.rotation.z, transformed.transform.rotation.w);
        geometry_msgs::Pose pose = mb::get_pose(position, orientation);
        
        result_.waypoint =  mb::get_pose_stamped(header, pose);

        if((std::abs(result_.waypoint.pose.position.y - current_pose.pose.position.y) > distance) || (std::abs(result_.waypoint.pose.position.x - current_pose.pose.position.x) > distance)) {
            // Stop the robot
            vel_msg.linear.x = 0.0;
            velocity_publisher_.publish(vel_msg);
            break;
        }

        velocity_publisher_.publish(vel_msg);
            
        // Process ROS callbacks
        rate.sleep();
    }

    // Stop the robot
    vel_msg.linear.x = 0.0;
    velocity_publisher_.publish(vel_msg);

    // Prepare the robot control result
    result_.robot_state = "Subroutine move_straight completed";

    return;
}

/* Function allowing the robot to achieve home pose */
void rc::RobotControllerAction::home_pose() {
    // First arm control
    this->arm_control(std::vector<double>{1.50, rc::MAX_ARM_2_JOINT, 0.0, 1.17, -1.53, rc::MAX_ARM_6_JOINT, 0.0});
    // Second arm control
    this->arm_control(std::vector<double>{rc::MIN_ARM_1_JOINT, rc::MAX_ARM_2_JOINT, 0.0, 1.17, -1.53, rc::MAX_ARM_6_JOINT, 0.0});
    // Home arm control
    this->arm_control(std::vector<double>{0.2, -1.34, -0.2, 1.94, -1.57, 1.37, 0.0});

    // Prepare the robot control result
    bool success = true;
    result_.waypoint = mb::get_frame_pose_stamped_wrt_frame("base_footprint", "map");
    result_.robot_state = success ? "Subroutine home pose completed" : "Subroutine home pose preempted";

    return;
}