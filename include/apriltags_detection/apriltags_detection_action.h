#ifndef APRILTAGS_DETECTION_ACTION_H
#define APRILTAGS_DETECTION_ACTION_H

/* Standard libraries */
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

/* User-defined libraries */
#include <apriltags_detection.h>

/* AprilTags detection namespace */
namespace ad {

    /* AprilTagsDetection action class handling the AprilTags detection */
    class AprilTagsDetectionAction {
        protected:
            // Node handler and action name
            ros::NodeHandle nh_;
            actionlib::SimpleActionServer<ir2425_group_15::AprilTagsDetectionAction> as_;
            std::string action_name_;

            // Goal, feedback and result messages
            ir2425_group_15::AprilTagsDetectionFeedback feedback_;
            ir2425_group_15::AprilTagsDetectionResult result_;

            // Additional attributes
            ros::Subscriber tag_subscriber_;
            AprilTagsDetection apriltags_;

            // Additional attributes for color
            ros::Subscriber camera_subscriber_;
            cv::Mat camera_matrix_;
            image_transport::Subscriber image_subscriber_;
            cv::Mat latest_image_;
            bool is_processing_;

        public:
            // Task action constructor
            AprilTagsDetectionAction(std::string name);

            // Callback function to read the topic with the AprilTags detected
            void apriltags_callback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg);

            // Callback function describing the action for the AprilTags detection
            void detect_apriltags_callback(const ir2425_group_15::AprilTagsDetectionGoalConstPtr& goal);

            // Extract ROI of AprilTag in current image
            cv::Rect get_roi_from_pose(const geometry_msgs::PoseStamped& pose, double apriltag_size_meters);
            // Color detection in ROI
            int detect_color(const cv::Mat& image_roi, const float color_threshold);
            // Camera info callback to retrieve intrinsic camera matrix
            void camera_info_callback(const sensor_msgs::CameraInfoConstPtr& msg);
            // Image retrieve callback
            void image_callback(const sensor_msgs::ImageConstPtr& msg);

            // Auxiliary functions
            void update_apriltags_detection_feedback();
            void update_apriltags_detection_result();
    };
}

#endif // APRILTAGS_DETECTION_ACTION_H