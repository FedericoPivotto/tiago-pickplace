#ifndef MESSAGE_BUILDER_H
#define MESSAGE_BUILDER_H

/* Standard libraries */
#include <ros/ros.h>
#include <regex>
#include <shape_msgs/SolidPrimitive.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <std_msgs/ColorRGBA.h>
#include <moveit_msgs/ObjectColor.h>
#include <visualization_msgs/Marker.h>

/* Message builder namespace*/
namespace mb {
    std_msgs::Header get_header(uint32_t seq, ros::Time stamp, std::string frame_id);
    geometry_msgs::Point get_point(float x = 0, float y = 0, float z = 0);
    geometry_msgs::Quaternion get_quaternion(float x = 0, float y = 0, float z = 0, float w = 1);
    geometry_msgs::Quaternion get_quaternion_from_rpy(float r = 0, float p = 0, float y = 0);
    geometry_msgs::Pose get_pose(geometry_msgs::Point position = get_point(), geometry_msgs::Quaternion orientation = get_quaternion());
    geometry_msgs::PoseStamped get_pose_stamped(std_msgs::Header header, geometry_msgs::Pose pose = get_pose());
    geometry_msgs::PoseStamped get_frame_pose_stamped_wrt_frame(std::string source_frame_id, std::string target_frame_id);
    geometry_msgs::PoseStamped get_pose_stamped_wrt_frame(geometry_msgs::PoseStamped pose_stamped, std::string target_frame_id);
    shape_msgs::SolidPrimitive get_solid_primitive(std::string shape, geometry_msgs::Point dimension);
    std::vector<std::string> get_frame_ids(std::regex pattern = std::regex(""));
    geometry_msgs::Pose subtract_poses(geometry_msgs::Pose pose_1, geometry_msgs::Pose pose_2);
    geometry_msgs::Point get_rpy_from_quaternion(geometry_msgs::Quaternion quaternion);
    gazebo_ros_link_attacher::Attach get_gazebo_link_attach_request(std::string model_name_1, std::string link_name_1, std::string model_name_2, std::string link_name_2);
    std_msgs::ColorRGBA get_color_rgba(float r, float g, float b, float a = 1);
    moveit_msgs::ObjectColor get_object_color(std::string id, std_msgs::ColorRGBA color);
    visualization_msgs::Marker get_marker(std_msgs::Header header, std::string ns, int id, int type, uint8_t action, geometry_msgs::Pose pose, geometry_msgs::Point scale, std_msgs::ColorRGBA color);
}

#endif // MESSAGE_BUILDER_H