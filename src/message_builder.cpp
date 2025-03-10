/* Standard libraries */
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/* User-defined libraries */
#include <message_builder.h>

/* Function to arbitrarily set std_msgs/Header */
std_msgs::Header mb::get_header(uint32_t seq, ros::Time stamp, std::string frame_id) {
    // Header
    std_msgs::Header header;
    
    // std_msgs/Header
    header.seq = seq;
    header.stamp = stamp;
    header.frame_id = frame_id;

    return header;
}

/* Function to arbitrarily set geometry_msgs/Point */
geometry_msgs::Point mb::get_point(float x, float y, float z) {
    // Point
    geometry_msgs::Point point;
    
    // geometry_msgs/Point
    point.x = x;
    point.y = y;
    point.z = z;

    return point;
}

/* Function to arbitrarily set geometry_msgs/Quaternion */
geometry_msgs::Quaternion mb::get_quaternion(float x, float y, float z, float w) {
    // Orientation
    geometry_msgs::Quaternion orientation;
    
    // geometry_msgs/Quaternion
    orientation.x = x;
    orientation.y = y;
    orientation.z = z;
    orientation.w = w;

    return orientation;
}

/* Function to get geometry_msgs/Quaternion from RPY angles */
geometry_msgs::Quaternion mb::get_quaternion_from_rpy(float r, float p, float y) {
    // Orientation
    tf2::Quaternion tf_orientation;
    tf_orientation.setRPY(r, p, y);
    geometry_msgs::Quaternion orientation = tf2::toMsg(tf_orientation);
    
    return orientation;
}

/* Function to arbitrarily set geometry_msgs/Pose */
geometry_msgs::Pose mb::get_pose(geometry_msgs::Point position, geometry_msgs::Quaternion orientation) {
    // Pose
    geometry_msgs::Pose pose;
    
    // geometry_msgs/Pose
    pose.position = position;
    pose.orientation = orientation;
    
    return pose;
}

/* Function to arbitrarily set geometry_msgs/PoseStamped */
geometry_msgs::PoseStamped mb::get_pose_stamped(std_msgs::Header header, geometry_msgs::Pose pose) {
    // Pose
    geometry_msgs::PoseStamped pose_stamped;
    
    // geometry_msgs/PoseStamped
    pose_stamped.header = header;
    pose_stamped.pose = pose;
    
    return pose_stamped;
}

/* Function to arbitrarily get geometry_msgs/PoseStamped of the origin of the given frame */
geometry_msgs::PoseStamped mb::get_frame_pose_stamped_wrt_frame(std::string source_frame_id, std::string target_frame_id) {
    std_msgs::Header header = mb::get_header(1, ros::Time::now(), target_frame_id);

    tf2_ros::Buffer tf_buffer(ros::Duration(10));
    tf2_ros::TransformListener tf_listener(tf_buffer);

    while(! tf_buffer.canTransform(target_frame_id, source_frame_id, ros::Time(0)))
        ros::Duration(0.5).sleep();

    geometry_msgs::TransformStamped transformed = tf_buffer.lookupTransform(target_frame_id, source_frame_id, ros::Time(0), ros::Duration(1.0));

    geometry_msgs::Point position = mb::get_point(transformed.transform.translation.x, transformed.transform.translation.y, transformed.transform.translation.z);
    geometry_msgs::Quaternion orientation = mb::get_quaternion(transformed.transform.rotation.x, transformed.transform.rotation.y, transformed.transform.rotation.z, transformed.transform.rotation.w);
    geometry_msgs::Pose pose = mb::get_pose(position, orientation);

    return mb::get_pose_stamped(header, pose);
}

/* Function to arbitrarily get geometry_msgs/PoseStamped of the pose stamped given with respect to the given frame */
geometry_msgs::PoseStamped mb::get_pose_stamped_wrt_frame(geometry_msgs::PoseStamped pose_stamped, std::string target_frame_id) {
    std::string source_frame_id = pose_stamped.header.frame_id;

    tf2_ros::Buffer tf_buffer(ros::Duration(10));
    tf2_ros::TransformListener tfListener(tf_buffer);

    while(!tf_buffer.canTransform(target_frame_id, source_frame_id, ros::Time(0)))
        ros::Duration(0.5).sleep();
    geometry_msgs::TransformStamped transformed = tf_buffer.lookupTransform(target_frame_id, source_frame_id, ros::Time(0), ros::Duration(1.0));
    
    // Transform from source frame to target frame
    geometry_msgs::PoseStamped pose_stamped_out;
    tf2::doTransform(pose_stamped, pose_stamped_out, transformed);

    return pose_stamped_out;
}

/* Function to get shape_msgs::SolidPrimitive of the given shape and dimension */
shape_msgs::SolidPrimitive mb::get_solid_primitive(std::string shape, geometry_msgs::Point dimension) {
    // Solid primitive
    shape_msgs::SolidPrimitive solid_primitive;
    
    // Shape
    if(shape == "box") {
        solid_primitive.type = solid_primitive.BOX;

        solid_primitive.dimensions.resize(3);
        solid_primitive.dimensions[solid_primitive.BOX_X] = dimension.x;
        solid_primitive.dimensions[solid_primitive.BOX_Y] = dimension.y;
        solid_primitive.dimensions[solid_primitive.BOX_Z] = dimension.z;
    }
    else if(shape == "sphere") {
        solid_primitive.type = solid_primitive.SPHERE;

        solid_primitive.dimensions.resize(1);
        solid_primitive.dimensions[solid_primitive.SPHERE_RADIUS] = dimension.x;
    }
    else if(shape == "cylinder") {
        solid_primitive.type = solid_primitive.CYLINDER;

        solid_primitive.dimensions.resize(2);
        solid_primitive.dimensions[solid_primitive.CYLINDER_HEIGHT] = dimension.x;
        solid_primitive.dimensions[solid_primitive.CYLINDER_RADIUS] = dimension.y;
    }
    else if(shape == "cone") {
        solid_primitive.type = solid_primitive.CONE;

        solid_primitive.dimensions.resize(2);
        solid_primitive.dimensions[solid_primitive.CONE_HEIGHT] = dimension.x;
        solid_primitive.dimensions[solid_primitive.CONE_RADIUS] = dimension.y;
    }
    
    return solid_primitive;
}

/* Function to get the frame id available according to a given pattern */
std::vector<std::string> mb::get_frame_ids(std::regex pattern) {
    // Initialize listener
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    while(! tf_buffer.canTransform("map", "base_footprint", ros::Time(0)))
        ros::Duration(0.5).sleep();

    // Wait and recover available frame ids
    std::vector<std::string> frame_ids;
    tf_buffer._getFrameStrings(frame_ids);

    // Return when pattern is not provided
    if(std::regex_match("", pattern))
        return frame_ids;

    // Filter frame ids matching pattern
    std::vector<std::string> matching_frame_ids;
    for (const std::string& frame_id : frame_ids) {
        if(std::regex_match(frame_id, pattern))
            matching_frame_ids.push_back(frame_id);
    }

    return matching_frame_ids;
}

/* Function to subtract second pose to first pose */
geometry_msgs::Pose mb::subtract_poses(geometry_msgs::Pose pose_1, geometry_msgs::Pose pose_2) {
    // Resulting pose
    geometry_msgs::Pose result_pose = pose_1;

    // Position subtraction
    result_pose.position.x -= pose_2.position.x;
    result_pose.position.y -= pose_2.position.y;
    result_pose.position.z -= pose_2.position.z;

    // Orientation subtraction
    geometry_msgs::Point result_rpy = mb::get_rpy_from_quaternion(result_pose.orientation);
    geometry_msgs::Point pose_2_rpy = mb::get_rpy_from_quaternion(pose_2.orientation);
    result_rpy.x += pose_2_rpy.x;
    result_rpy.y += pose_2_rpy.y;
    result_rpy.z += pose_2_rpy.z;
    result_pose.orientation = mb::get_quaternion_from_rpy(result_rpy.x, result_rpy.y, result_rpy.z);
    
    return result_pose;
}

/* Function to get RPY angles from quaternion */
geometry_msgs::Point mb::get_rpy_from_quaternion(geometry_msgs::Quaternion quaternion) {
    // Transform quaternion
    tf2::Quaternion tf_quaternion;
    tf2::fromMsg(quaternion, tf_quaternion);
    tf2::Matrix3x3 quaternion_matrix(tf_quaternion);
    
    // Compute RPY angles
    geometry_msgs::Point rpy;
    quaternion_matrix.getRPY(rpy.x, rpy.y, rpy.z);

    return rpy;
}

/* Function to get the Gazebo link attach request */
gazebo_ros_link_attacher::Attach mb::get_gazebo_link_attach_request(std::string model_name_1, std::string link_name_1, std::string model_name_2, std::string link_name_2) {
    // Gazebo link attacher request
    gazebo_ros_link_attacher::Attach gazebo_ros_link_attach;
    gazebo_ros_link_attach.request.model_name_1 = model_name_1;
    gazebo_ros_link_attach.request.link_name_1 = link_name_1;
    gazebo_ros_link_attach.request.model_name_2 = model_name_2;
    gazebo_ros_link_attach.request.link_name_2 = link_name_2;

    return gazebo_ros_link_attach;
}

/* Function to get RGBA color from the components */
std_msgs::ColorRGBA mb::get_color_rgba(float r, float g, float b, float a) {
    // Color in RGBA
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;

    return color;
}

/* Function to get object color given the RGBA color */
moveit_msgs::ObjectColor mb::get_object_color(std::string id, std_msgs::ColorRGBA color) {
    // Object color
    moveit_msgs::ObjectColor object_color;
    object_color.id = id;
    object_color.color = color;

    return object_color;
}

/* Function to get marker */
visualization_msgs::Marker mb::get_marker(std_msgs::Header header, std::string ns, int id, int type, uint8_t action, geometry_msgs::Pose pose, geometry_msgs::Point scale, std_msgs::ColorRGBA color) {
    // Marker
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = action;
    marker.pose = pose;
    marker.scale.x = scale.x;
    marker.scale.y = scale.y;
    marker.scale.z = scale.z;
    marker.color = color;

    return marker;
}