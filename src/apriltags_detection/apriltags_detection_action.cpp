/* Standard libraries */
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/* User-defined libraries */
#include <apriltags_detection_action.h>
#include <message_builder.h>

// Task action constructor
ad::AprilTagsDetectionAction::AprilTagsDetectionAction(std::string name) : as_(nh_, name, boost::bind(&ad::AprilTagsDetectionAction::detect_apriltags_callback, this, _1), false), action_name_(name), apriltags_(), is_processing_(false) {
    // Start the AprilTags detection action
    as_.start();

    // Initialize image subscriber
    image_transport::ImageTransport it(nh_);
    image_subscriber_ = it.subscribe("/xtion/rgb/image_raw", 1, &ad::AprilTagsDetectionAction::image_callback, this);
    // Subscribe to camera info to retrieve camera matrix
    camera_subscriber_ = nh_.subscribe("/xtion/rgb/camera_info", 1, &ad::AprilTagsDetectionAction::camera_info_callback, this);

    // Subscribe to the topic "tag_detections"
    tag_subscriber_ = nh_.subscribe("tag_detections", 1, &ad::AprilTagsDetectionAction::apriltags_callback, this);
}

/* Callback function to read the topic with the AprilTags detected */
void ad::AprilTagsDetectionAction::apriltags_callback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg) {
    std::string source_frame = msg->header.frame_id, target_frame = "map";

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    while(! tf_buffer.canTransform(target_frame, source_frame, ros::Time(0)))
        ros::Duration(0.5).sleep();

    // Transform available
    geometry_msgs::TransformStamped transformed = tf_buffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
    geometry_msgs::PoseStamped pose_in, pose_out;

    for(size_t i = 0; i < msg->detections.size(); ++i) {
        // Read detected AprilTag
        int apriltag_id = msg->detections.at(i).id[0];
        pose_in = mb::get_pose_stamped(
            msg->detections.at(i).pose.header,
            mb::get_pose(
                mb::get_point(
                    msg->detections.at(i).pose.pose.pose.position.x, 
                    msg->detections.at(i).pose.pose.pose.position.y, 
                    msg->detections.at(i).pose.pose.pose.position.z
                ),
                mb::get_quaternion(
                    msg->detections.at(i).pose.pose.pose.orientation.x, 
                    msg->detections.at(i).pose.pose.pose.orientation.y, 
                    msg->detections.at(i).pose.pose.pose.orientation.z, 
                    msg->detections.at(i).pose.pose.pose.orientation.w
                )
            )
        );
    
        // Transform from camera frame to map frame
        tf2::doTransform(pose_in, pose_out, transformed);

        // Processing flag
        is_processing_ = true;

        // Check image of the detected AprilTag
        if (latest_image_.empty())
            continue;
        // Check camera matrix
        if (camera_matrix_.empty())
            continue;
        // Get AprilTag roi of current image
        cv::Mat image_roi = latest_image_(this->get_roi_from_pose(pose_in, msg->detections.at(i).size[0]));
        // Check AprilTag color
        float color_threshold = 0.05;
        int is_color = this->detect_color(image_roi, color_threshold); 
        std_msgs::ColorRGBA apriltag_color;
        if(is_color == 1) // Red
            apriltag_color = mb::get_color_rgba(1, 0, 0);
        else if(is_color == 2) // Blue
            apriltag_color = mb::get_color_rgba(0, 0, 1);
        else if(is_color == 3) // Green
            apriltag_color = mb::get_color_rgba(0, 1, 0);
        else // No color
            apriltag_color = mb::get_color_rgba(0, 0, 0);

        // Update the AprilTags detected
        apriltags_.insert_apriltag(apriltag_id, pose_out, apriltag_color);

        // Processing flag
        is_processing_ = false;
    }
}

/* Callback function describing the action for the AprilTags detection */
void ad::AprilTagsDetectionAction::detect_apriltags_callback(const ir2425_group_15::AprilTagsDetectionGoalConstPtr& goal) {
    // Action initial state
    bool success = true;

    // Initialize AprilTags detection feedback
    feedback_.n_srv_target_apriltag_ids = goal->n_srv_target_apriltag_ids;
    feedback_.srv_target_apriltag_ids = goal->srv_target_apriltag_ids;

    // Initialize AprilTags detection result
    result_.n_srv_target_apriltag_ids = goal->n_srv_target_apriltag_ids;
    result_.srv_target_apriltag_ids = goal->srv_target_apriltag_ids;

    // Initialize AprilTags storage
    apriltags_.set_target_apriltag_ids(goal->srv_target_apriltag_ids);

    // Detected AprilTags reading
    while(! apriltags_.is_apriltags_detection_completed()) {
        // Check if preempt is requested by the client
        if(as_.isPreemptRequested()) {
            // Set action failure
            success = false;
            break;
        }

        // Publish the AprilTags detection feedback
        this->update_apriltags_detection_feedback();
        as_.publishFeedback(feedback_);
    }

    // Publish the AprilTags detection result
    this->update_apriltags_detection_result();
    success ? as_.setSucceeded(result_) : as_.setPreempted(result_);

    return;
}

/* Function to update the AprilTags detection feedback */
void ad::AprilTagsDetectionAction::update_apriltags_detection_feedback() {
    feedback_.ready = true;
    feedback_.n_target_apriltag_ids = apriltags_.count_target_apriltags_detected();
    feedback_.target_apriltag_ids = apriltags_.get_target_apriltag_ids_detected();
    feedback_.target_apriltag_poses = apriltags_.get_target_apriltag_poses_detected();
    feedback_.target_apriltag_colors = apriltags_.get_target_apriltag_colors_detected();

    feedback_.n_nontarget_apriltag_ids = apriltags_.count_nontarget_apriltags_detected();
    feedback_.nontarget_apriltag_ids = apriltags_.get_nontarget_apriltag_ids_detected();
    feedback_.nontarget_apriltag_poses = apriltags_.get_nontarget_apriltag_poses_detected();
    feedback_.nontarget_apriltag_colors = apriltags_.get_nontarget_apriltag_colors_detected();
}

/* Function to update the AprilTags detection result */
void ad::AprilTagsDetectionAction::update_apriltags_detection_result() {
    result_.is_completed = apriltags_.is_apriltags_detection_completed();

    result_.n_target_apriltag_ids = apriltags_.count_target_apriltags_detected();
    result_.target_apriltag_ids = apriltags_.get_target_apriltag_ids_detected();
    result_.target_apriltag_poses = apriltags_.get_target_apriltag_poses_detected();
    result_.target_apriltag_colors = apriltags_.get_target_apriltag_colors_detected();

    result_.n_nontarget_apriltag_ids = apriltags_.count_nontarget_apriltags_detected();
    result_.nontarget_apriltag_ids = apriltags_.get_nontarget_apriltag_ids_detected();
    result_.nontarget_apriltag_poses = apriltags_.get_nontarget_apriltag_poses_detected();
    result_.nontarget_apriltag_colors = apriltags_.get_nontarget_apriltag_colors_detected();
}

/* Get ROI of detected AprilTag in current image */
cv::Rect ad::AprilTagsDetectionAction::get_roi_from_pose(const geometry_msgs::PoseStamped& pose, double apriltag_size_meters) {
    // Camera intrinsic parameters
    double fx = camera_matrix_.at<double>(0,0);    
    double fy = camera_matrix_.at<double>(1,1);    
    double cx = camera_matrix_.at<double>(0,2);    
    double cy = camera_matrix_.at<double>(1,2);

    // Apriltag position w.r.t. camera frame
    double x = pose.pose.position.x;
    double y = pose.pose.position.y;    
    double z = pose.pose.position.z;

    // Projection from 3D to 2D image coordinates
    double u = (fx * x / z) + cx;
    double v = (fy * y / z) + cy;

    // Estimate AprilTag size in pixels
    double apriltag_size_pixels = (apriltag_size_meters * fx) / z;

    // Compute ROI top-left corner
    int roi_x = std::max(0, static_cast<int>(u - (apriltag_size_pixels / 2)));
    int roi_y = std::max(0, static_cast<int>(v - (apriltag_size_pixels / 2)));

    // Compute ROI width and height
    int roi_width = std::min(latest_image_.cols - roi_x, static_cast<int>(apriltag_size_pixels));
    int roi_height = std::min(latest_image_.rows - roi_y, static_cast<int>(apriltag_size_pixels));

    // Return AprilTag ROI
    return cv::Rect(roi_x, roi_y, roi_width, roi_height);
}

/* Function to detect color in ROI, given the color id */
int ad::AprilTagsDetectionAction::detect_color(const cv::Mat& image_roi, const float color_threshold){
    // Convert image to HSV color space
    cv::Mat hsv_image;
    cv::cvtColor(image_roi, hsv_image, cv::COLOR_BGR2HSV);

    // Define color thresholds
    cv::Scalar lower_low_hue_red = cv::Scalar(0, 70, 50);
    cv::Scalar upper_low_hue_red = cv::Scalar(10, 255, 255);
    cv::Scalar lower_high_hue_red = cv::Scalar(170, 70, 50);
    cv::Scalar upper_high_hue_red = cv::Scalar(180, 255, 255);

    cv::Scalar lower_blue = cv::Scalar(100, 70, 50);
    cv::Scalar upper_blue = cv::Scalar(130, 255, 255);

    cv::Scalar lower_green = cv::Scalar(35, 70, 50);
    cv::Scalar upper_green = cv::Scalar(85, 255, 255);

    // Map color IDs to their respective masks
    std::map<int, std::pair<cv::Scalar, cv::Scalar>> color_ranges = {
        {1, {lower_low_hue_red, upper_low_hue_red}},
        {2, {lower_blue, upper_blue}},
        {3, {lower_green, upper_green}},
    };

    // Define color range based on color id
    cv::Mat upper_red_mask, color_mask;

    // Check additional high hue range of red
    cv::inRange(hsv_image, lower_high_hue_red, upper_high_hue_red, upper_red_mask);

    // Dominant color id and proportion
    int dominant_color_id = -1;
    double max_color_proportion = 0.0; 

    // Iterate over colors ranges to determine the AprilTag color
    for (const auto& [color_id, ranges] : color_ranges) {
        cv::Mat color_mask;
        cv::inRange(hsv_image, ranges.first, ranges.second, color_mask);

        // Combine upper and lower masks for red color case 
        if (color_id == 1) {
            cv::bitwise_or(color_mask, upper_red_mask, color_mask);
        }

        // Compute color proportion
        double color_proportion = cv::countNonZero(color_mask) / static_cast<double>(image_roi.total());
        
        // Check for dominant color
        if (color_proportion > max_color_proportion) {
            max_color_proportion = color_proportion;
            dominant_color_id = color_id;
        }
    }

    // Return true if in ROI there is enough dominant color
    return (max_color_proportion > color_threshold) ? dominant_color_id : -1;
}

/* Callback to retrieve camera matrix */
void ad::AprilTagsDetectionAction::camera_info_callback(const sensor_msgs::CameraInfoConstPtr& msg) {
    if(! is_processing_) {
        // Extract intrinsic camera matrix
        const boost::array<double, 9>& K = msg->K;

        // Convert to Mat matrix
        camera_matrix_ = (cv::Mat_<double>(3, 3) <<
                                K[0], K[1], K[2],
                                K[3], K[4], K[5],
                                K[6], K[7], K[8]);
        
        // If camera matrix has been read, shutdown subscriber
        if (cv::countNonZero(camera_matrix_) > 0)
            camera_subscriber_.shutdown();
    }
}

/* Callback to retrieve image in which AprilTag is detected */
// WARNING: test both toCvShare(..) (shallow copy) and toCvCopy() (deep copy) if needed
void ad::AprilTagsDetectionAction::image_callback(const sensor_msgs::ImageConstPtr& msg) {
    if(! is_processing_)
        latest_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
}