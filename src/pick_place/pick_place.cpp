/* Standard libraries */
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

/* User-defined libraries */
#include <pick_place.h>
#include <message_builder.h> 

/* Function to parse tables structures from YAML */
std::vector<std::tuple<uint32_t, geometry_msgs::PoseStamped, std::string, shape_msgs::SolidPrimitive>> pp::parse_tables_yaml(std::string package, std::string path, std::string filename, std::string ext) {
    // Tables vector
    std::vector<std::tuple<uint32_t, geometry_msgs::PoseStamped, std::string, shape_msgs::SolidPrimitive>> tables;

    // Load YAML file
    YAML::Node config_yaml = YAML::LoadFile(package + "/" + path + filename + "." + ext);

    // Tables array
    auto tables_yaml = config_yaml["tables"];

    // Table element with header, pose and dimension
    for(auto table_yaml : tables_yaml) {
        // ID
        YAML::Node table = table_yaml["table"];
        uint32_t id = table["id"].as<int>();

        // Shape
        YAML::Node type = table_yaml["type"];
        std::string shape_type = type["type"].as<std::string>();
        std::string shape = type["shape"].as<std::string>();
        // Dimension
        YAML::Node dimension = table_yaml["dimension"];
        geometry_msgs::Point point_dimension = mb::get_point(dimension["x"].as<float>(), dimension["y"].as<float>(), dimension["z"].as<float>());

        // Header
        YAML::Node header = table_yaml["header"];
        std_msgs::Header waypoint_header = mb::get_header(header["seq"].as<int>(), ros::Time::now(), header["frame_id"].as<std::string>());
        // Position
        YAML::Node point = table_yaml["point"];
        geometry_msgs::Point position = mb::get_point(point["x"].as<float>(), point["y"].as<float>(), point["z"].as<float>());        
        // Orientation
        YAML::Node quaternion = table_yaml["quaternion"];
        geometry_msgs::Quaternion orientation = mb::get_quaternion(quaternion["x"].as<float>(), quaternion["y"].as<float>(), quaternion["z"].as<float>(), quaternion["w"].as<float>());
        
        // Add pose to table vector
        geometry_msgs::Pose waypoint_pose = mb::get_pose(position, orientation);
        shape_msgs::SolidPrimitive solid_primitive = mb::get_solid_primitive(shape, point_dimension);
        tables.push_back(std::make_tuple(id, mb::get_pose_stamped(waypoint_header, waypoint_pose), shape_type, solid_primitive));
    }

    return tables;
}

/* Function to get the collision objects */
std::vector<moveit_msgs::CollisionObject> pp::get_table_collision_objects() {
    // Table collision objects
    std::vector<moveit_msgs::CollisionObject> table_collision_objects;

    // Parsed tables
    std::vector<std::tuple<uint32_t, geometry_msgs::PoseStamped, std::string, shape_msgs::SolidPrimitive>> tables = pp::parse_tables_yaml(ros::package::getPath("ir2425_group_15"), "config/", "tables", "yaml");

    // Scan tables
    for(size_t i = 0; i < tables.size(); ++i) {
        // Table information
        std::tuple<uint32_t, geometry_msgs::PoseStamped, std::string, shape_msgs::SolidPrimitive>& table = tables[i];

        // Table collision object
        moveit_msgs::CollisionObject table_collision_object;

        // Table ID
        table_collision_object.id = std::get<2>(table) + "_" + std::to_string(std::get<0>(table));
        
        // Pose stamped with respect to robot base
        table_collision_object.header = mb::get_header(i, ros::Time::now(), "map");
        table_collision_object.primitive_poses.resize(1);
        table_collision_object.primitive_poses.push_back(std::get<1>(table).pose);
        // Solid primitive
        table_collision_object.primitives.resize(1);
        table_collision_object.primitives.push_back(std::get<3>(table));

        // Empty meshes
        table_collision_object.meshes.resize(0);
        table_collision_object.mesh_poses.resize(0);
        // Empty planes
        table_collision_object.planes.resize(0);
        table_collision_object.plane_poses.resize(0);

        // Table ADD operation
        table_collision_object.operation = moveit_msgs::CollisionObject::ADD;

        // Save table collision object
        table_collision_objects.push_back(table_collision_object);
    }
    
    return table_collision_objects;
}

/* Function to parse AprilTag structures from YAML */
std::vector<std::tuple<uint32_t, geometry_msgs::Pose, std::string, shape_msgs::SolidPrimitive, std::string>> pp::parse_apriltags_yaml(std::string package, std::string path, std::string filename, std::string ext) {
    // AprilTags vector
    std::vector<std::tuple<uint32_t, geometry_msgs::Pose, std::string, shape_msgs::SolidPrimitive, std::string>> apriltags;

    // Load YAML file
    YAML::Node config_yaml = YAML::LoadFile(package + "/" + path + filename + "." + ext);

    // AprilTags array
    auto apriltags_yaml = config_yaml["apriltags"];

    // AprilTag with ID, offset and dimension
    for(auto apriltag_yaml : apriltags_yaml) {
        // ID
        YAML::Node tag = apriltag_yaml["tag"];
        uint32_t id = tag["id"].as<int>();
        
        // Shape
        YAML::Node type = apriltag_yaml["type"];
        std::string shape_type = type["type"].as<std::string>();
        std::string shape = type["shape"].as<std::string>();
        std::string model = type["model"].as<std::string>();
        // Dimension
        YAML::Node dimension = apriltag_yaml["dimension"];
        geometry_msgs::Point point_dimension = mb::get_point(dimension["x"].as<float>(), dimension["y"].as<float>(), dimension["z"].as<float>());  

        // Linear displacement
        YAML::Node point = apriltag_yaml["point"];
        geometry_msgs::Point linear_displacement = mb::get_point(point["x"].as<float>(), point["y"].as<float>(), point["z"].as<float>());        
        // Angular displacement
        YAML::Node rpy = apriltag_yaml["rpy"];
        geometry_msgs::Quaternion angular_displacement = mb::get_quaternion_from_rpy(rpy["r"].as<float>(), rpy["p"].as<float>(), rpy["y"].as<float>());
        
        // Add pose to AprilTag vector
        geometry_msgs::Pose pose = mb::get_pose(linear_displacement, angular_displacement);
        shape_msgs::SolidPrimitive solid_primitive = mb::get_solid_primitive(shape, point_dimension);
        apriltags.push_back(std::make_tuple(id, pose, shape_type, solid_primitive, model));
    }
    
    return apriltags;
}

/* Function to get the AprilTag collision object given the ID */
moveit_msgs::CollisionObject pp::get_apriltag_collision_object(uint32_t apriltag_id, geometry_msgs::PoseStamped apriltag_pose_stamped) {
    // AprilTag collision object
    moveit_msgs::CollisionObject apriltag_collision_object;

    // Parsed AprilTags
    std::vector<std::tuple<uint32_t, geometry_msgs::Pose, std::string, shape_msgs::SolidPrimitive, std::string>> apriltags = pp::parse_apriltags_yaml(ros::package::getPath("ir2425_group_15"), "config/", "apriltags", "yaml");

    // Scan AprilTags
    for(size_t i = 0; i < apriltags.size(); ++i) {
        // AprilTags information
        std::tuple<uint32_t, geometry_msgs::Pose, std::string, shape_msgs::SolidPrimitive, std::string>& apriltag = apriltags[i];

        // Check AprilTag ID
        if(std::get<0>(apriltag) == apriltag_id) {
            // AprilTag ID
            apriltag_collision_object.id = std::get<2>(apriltag) + "_" + std::to_string(apriltag_id);
            // Pose stamped with respect to robot base
            apriltag_collision_object.header = mb::get_header(apriltag_id, ros::Time::now(), "map");

            // Compute object pose stamped by subtracting
            geometry_msgs::Pose apriltag_offset_pose = std::get<1>(apriltag);
            geometry_msgs::PoseStamped object_pose_stamped = apriltag_pose_stamped;
            object_pose_stamped.pose = mb::subtract_poses(apriltag_pose_stamped.pose, apriltag_offset_pose);
            // Pose stamped with respect to robot base
            geometry_msgs::PoseStamped apriltag_pose = object_pose_stamped;
            apriltag_collision_object.primitive_poses.resize(1);
            apriltag_collision_object.primitive_poses.push_back(apriltag_pose.pose);
            // Solid primitive
            apriltag_collision_object.primitives.resize(1);
            apriltag_collision_object.primitives.push_back(std::get<3>(apriltag));

            // AprilTag ADD operation
            apriltag_collision_object.operation = moveit_msgs::CollisionObject::ADD;

            // Return table collision object
            break;
        }
    }
    
    return apriltag_collision_object;
}

/* Function to get the AprilTag object color given the ID */
moveit_msgs::ObjectColor pp::get_apriltag_object_color(uint32_t apriltag_id, std::string apriltag_collision_object_id, std_msgs::ColorRGBA apriltag_color) {
    // AprilTag object color
    moveit_msgs::ObjectColor apriltag_object_color;
    
    // Parsed AprilTags
    std::vector<std::tuple<uint32_t, geometry_msgs::Pose, std::string, shape_msgs::SolidPrimitive, std::string>> apriltags = pp::parse_apriltags_yaml(ros::package::getPath("ir2425_group_15"), "config/", "apriltags", "yaml");

    // Scan AprilTags
    for(size_t i = 0; i < apriltags.size(); ++i) {
        // AprilTags information
        std::tuple<uint32_t, geometry_msgs::Pose, std::string, shape_msgs::SolidPrimitive, std::string>& apriltag = apriltags[i];

        // Check AprilTag ID
        if(std::get<0>(apriltag) == apriltag_id) {
            // AprilTag ID and color
            apriltag_object_color.id = apriltag_collision_object_id;
            apriltag_object_color.color = apriltag_color;

            // Return object color
            break;
        }
    }

    return apriltag_object_color;
}

/* Function to get the AprilTag object Gazebo model given the ID */
std::string pp::get_apriltag_object_gazebo_model(uint32_t apriltag_id) {
    // AprilTag collision object
    std::string apriltag_object_gazebo_model;

    // Parsed AprilTags
    std::vector<std::tuple<uint32_t, geometry_msgs::Pose, std::string, shape_msgs::SolidPrimitive, std::string>> apriltags = pp::parse_apriltags_yaml(ros::package::getPath("ir2425_group_15"), "config/", "apriltags", "yaml");

    // Scan AprilTags
    for(size_t i = 0; i < apriltags.size(); ++i) {
        // AprilTags information
        std::tuple<uint32_t, geometry_msgs::Pose, std::string, shape_msgs::SolidPrimitive, std::string>& apriltag = apriltags[i];

        // Check AprilTag ID
        if(std::get<0>(apriltag) == apriltag_id) {
            // Save AprilTag object Gazebo model
            apriltag_object_gazebo_model = std::get<4>(apriltag);

            // Return table collision object
            break;
        }
    }
    
    return apriltag_object_gazebo_model;
}

/* Function to get the place locations along the given straight line */
std::vector<geometry_msgs::PoseStamped> pp::get_straight_line_place_locations(double m, double q, std::string frame, std::string wrt_frame, int slots, double edge) {
    // Lambda function for y = mx + q
    auto y = [m, q](double x) -> double { return m * x + q; };
    // Lambda function for x = (y - q) / m
    auto x = [m, q](double y) -> double { return m != 0 ? (y - q) / m : std::numeric_limits<double>::infinity(); };

    // Compute A
    double x_A, y_A;
    if(q < 0) {
        y_A = 0;
        x_A = x(y_A);

        if(x_A < 0 || x_A > edge)
            throw std::invalid_argument("Error: invalid straight line");
    }
    else if(q > edge) {
        y_A = edge;
        x_A = x(y_A);

        if(x_A < 0 || x_A > edge)
            throw std::invalid_argument("Error: invalid straight line");
    }
    else {
        x_A = 0;
        y_A = y(x_A);
    }
    std::pair<double, double> A(x_A, y_A);

    // Compute B
    double s = y(edge);
    double x_B, y_B;
    if(s < 0) {
        y_B = 0;
        x_B = x(y_B);

        if(x_B < 0 || x_B > edge)
            throw std::invalid_argument("Error: invalid straight line");
    }
    else if(s > edge) {
        y_B = edge;
        x_B = x(y_B);

        if(x_B < 0 || x_B > edge)
            throw std::invalid_argument("Error: invalid straight line");
    }
    else {
        x_B = edge;
        y_B = y(x_B);
    }
    std::pair<double, double> B(x_B, y_B);

    // Compute base, height, hypotenuse
    double b_AB = std::fabs(A.second - B.second);
    double h_AB = std::fabs(A.first - B.first);
    double i_AB = std::sqrt(std::pow(b_AB, 2) + std::pow(h_AB, 2));

    // Compute distance between objects
    double distance = i_AB / (slots - 1);

    // Compute straight line angle
    double angle = std::atan(m);

    // Compute object offsets
    double x_OFF = distance * std::cos(angle);
    double y_OFF = distance * std::sin(angle);

    // Publisher for markers
    ros::NodeHandle nh;
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    
    // Place locations
    std::vector<geometry_msgs::PoseStamped> place_locations;
    for(size_t i = 1; i <= slots; ++i) {
        // Object offsets
        double x_OBJ = x_A + x_OFF * (i - 1);
        double y_OBJ = y_A + y_OFF * (i - 1);

        // Header
        std_msgs::Header header = mb::get_header(i, ros::Time::now(), frame);
        // Pose
        geometry_msgs::Pose pose = mb::get_pose(mb::get_point(x_OBJ, y_OBJ, 0), mb::get_quaternion());

        // Pose stamped
        geometry_msgs::PoseStamped place_location_wrt_frame = mb::get_pose_stamped(header, pose);
        geometry_msgs::PoseStamped place_location = mb::get_pose_stamped_wrt_frame(place_location_wrt_frame, wrt_frame);
        place_locations.push_back(place_location);

        // Place location markers
        visualization_msgs::Marker marker = mb::get_marker(
            mb::get_header(i, ros::Time::now(), "map"), "place_locations", i, 
            visualization_msgs::Marker::CUBE, visualization_msgs::Marker::ADD, 
            place_location.pose, mb::get_point(x_OFF, y_OFF, 0.01), mb::get_color_rgba(0, 0, 0)
        );
        // Publish marker
        vis_pub.publish(marker);
    }

    return place_locations;
}