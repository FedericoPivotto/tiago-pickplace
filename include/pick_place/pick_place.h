#ifndef PICK_PLACE_H
#define PICK_PLACE_H

/* Standard libraries */
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

/* Pick place namespace */
namespace pp {
    // Approach displacement
    constexpr double APPROACH_DISPLACEMENT = 0.2;
    constexpr double PICK_DISPLACEMENT = 0.1;
    constexpr double LEAVE_DISPLACEMENT = 0.1;
    constexpr double PLACE_DISPLACEMENT = 0.2;

    // Table functions
    std::vector<std::tuple<uint32_t, geometry_msgs::PoseStamped, std::string, shape_msgs::SolidPrimitive>> parse_tables_yaml(std::string package, std::string path, std::string filename, std::string ext);
    std::vector<moveit_msgs::CollisionObject> get_table_collision_objects();

    // Tag functions
    std::vector<std::tuple<uint32_t, geometry_msgs::Pose, std::string, shape_msgs::SolidPrimitive, std::string>> parse_apriltags_yaml(std::string package, std::string path, std::string filename, std::string ext);
    moveit_msgs::CollisionObject get_apriltag_collision_object(uint32_t apriltag_id, geometry_msgs::PoseStamped apriltag_pose_stamped);
    moveit_msgs::ObjectColor get_apriltag_object_color(uint32_t apriltag_id, std::string apriltag_collision_object_id, std_msgs::ColorRGBA apriltag_color);
    std::string get_apriltag_object_gazebo_model(uint32_t apriltag_id);

    // Straight line place locations
    std::vector<geometry_msgs::PoseStamped> get_straight_line_place_locations(double m, double q, std::string frame, std::string wrt_frame, int slots, double edge);
}

#endif // PICK_PLACE_H