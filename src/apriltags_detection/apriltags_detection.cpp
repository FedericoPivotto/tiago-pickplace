/* User-defined libraries */
#include <apriltags_detection.h>
#include <message_builder.h>

/* AprilTagsDetection constructor */
ad::AprilTagsDetection::AprilTagsDetection() {
}

/* AprilTagsDetection constructor for reading purpose */
ad::AprilTagsDetection::AprilTagsDetection(const std::vector<int> target_apriltag_ids, const std::vector<int> target_apriltag_ids_detected, const geometry_msgs::PoseArray target_apriltag_poses_detected, const std::vector<std_msgs::ColorRGBA> color_target_apriltags_detected, const std::vector<int> nontarget_apriltag_ids_detected, const geometry_msgs::PoseArray nontarget_apriltag_poses_detected, const std::vector<std_msgs::ColorRGBA> color_nontarget_apriltags_detected) : target_apriltag_ids_(target_apriltag_ids) {
    // Insert target AprilTags detected
    for(size_t i = 0; i < target_apriltag_ids_detected.size(); ++i) {
        std_msgs::Header header = mb::get_header(i, ros::Time::now(), "map");
        insert_apriltag(target_apriltag_ids_detected[i], mb::get_pose_stamped(header, target_apriltag_poses_detected.poses[i]), color_target_apriltags_detected[i]);
    }

    // Insert non-target AprilTags detected
    for(size_t i = 0; i < nontarget_apriltag_ids_detected.size(); ++i) {
        std_msgs::Header header = mb::get_header(i, ros::Time::now(), "map");
        insert_apriltag(nontarget_apriltag_ids_detected[i], mb::get_pose_stamped(header, nontarget_apriltag_poses_detected.poses[i]), color_nontarget_apriltags_detected[i]);
    }
}

/* Function to set the target AprilTag IDs */
void ad::AprilTagsDetection::set_target_apriltag_ids(const std::vector<int> target_apriltag_ids) {
    target_apriltag_ids_ = target_apriltag_ids;
}

/* Function to get the number of target Apriltags to find */
int ad::AprilTagsDetection::count_target_apriltags() const {
    return target_apriltag_ids_.size();
}

/* Function to get the number of target Apriltags found */
int ad::AprilTagsDetection::count_target_apriltags_detected() const {
    return target_apriltags_detected_.size();
}

/* Function to get the number of non-target Apriltags found */
int ad::AprilTagsDetection::count_nontarget_apriltags_detected() const {
    return nontarget_apriltags_detected_.size();
}

/* Function that returns the dict for the result with target AprilTags found */
const std::map<int, std::pair<geometry_msgs::PoseStamped, std_msgs::ColorRGBA>> ad::AprilTagsDetection::get_target_apriltags_detected() const {
    return target_apriltags_detected_;
}

/* Function that returns the IDs of the dict for the result with target AprilTags found */
const std::vector<int> ad::AprilTagsDetection::get_target_apriltag_ids_detected() const {
    std::vector<int> apriltag_ids;
    for (const std::pair<int, std::pair<geometry_msgs::PoseStamped, std_msgs::ColorRGBA>>& target_apriltag : target_apriltags_detected_)
        apriltag_ids.push_back(target_apriltag.first);

    return apriltag_ids;
}

/* Function that returns the poses of the dict for the result with target AprilTags found */
const geometry_msgs::PoseArray ad::AprilTagsDetection::get_target_apriltag_poses_detected() const {
    geometry_msgs::PoseArray apriltag_poses;
    apriltag_poses.header = mb::get_header(1, ros::Time::now(), "map");
    for (const std::pair<int, std::pair<geometry_msgs::PoseStamped, std_msgs::ColorRGBA>>& target_apriltag : target_apriltags_detected_)
        apriltag_poses.poses.push_back(target_apriltag.second.first.pose);

    return apriltag_poses;
}

/* Function that returns the dict for the result with target AprilTags color found */
const std::vector<std_msgs::ColorRGBA> ad::AprilTagsDetection::get_target_apriltag_colors_detected() const {
    std::vector<std_msgs::ColorRGBA> apriltag_colors;
    for (const std::pair<int, std::pair<geometry_msgs::PoseStamped, std_msgs::ColorRGBA>>& target_apriltag : target_apriltags_detected_)
        apriltag_colors.push_back(target_apriltag.second.second);

    return apriltag_colors;
}

/* Function that returns the non-target Apriltags, useful for the feedback */
const std::map<int, std::pair<geometry_msgs::PoseStamped, std_msgs::ColorRGBA>> ad::AprilTagsDetection::get_nontarget_apriltags_detected() const {
    return nontarget_apriltags_detected_;
}

/* Function that returns the IDs of the dict for the result with non-target AprilTags found */
const std::vector<int> ad::AprilTagsDetection::get_nontarget_apriltag_ids_detected() const {
    std::vector<int> apriltag_ids;
    for (const std::pair<int, std::pair<geometry_msgs::PoseStamped, std_msgs::ColorRGBA>>& nontarget_apriltag : nontarget_apriltags_detected_)
        apriltag_ids.push_back(nontarget_apriltag.first);

    return apriltag_ids;
}

/* Function that returns the poses of the dict for the result with non-target AprilTags found */
const geometry_msgs::PoseArray ad::AprilTagsDetection::get_nontarget_apriltag_poses_detected() const {
    geometry_msgs::PoseArray apriltag_poses;
    apriltag_poses.header = mb::get_header(1, ros::Time::now(), "map");
    for (const std::pair<int, std::pair<geometry_msgs::PoseStamped, std_msgs::ColorRGBA>>& nontarget_apriltag : nontarget_apriltags_detected_)
        apriltag_poses.poses.push_back(nontarget_apriltag.second.first.pose);

    return apriltag_poses;
}

/* Function that returns the dict for the result with non-target AprilTags color found */
const std::vector<std_msgs::ColorRGBA> ad::AprilTagsDetection::get_nontarget_apriltag_colors_detected() const {
    std::vector<std_msgs::ColorRGBA> apriltag_colors;
    for (const std::pair<int, std::pair<geometry_msgs::PoseStamped, std_msgs::ColorRGBA>>& nontarget_apriltag : nontarget_apriltags_detected_)
        apriltag_colors.push_back(nontarget_apriltag.second.second);

    return apriltag_colors;
}

/* Function that returns the IDs of the target AprilTags to find */
const std::vector<int> ad::AprilTagsDetection::get_target_apriltag_ids() const {
    return target_apriltag_ids_;
}

/* Function to print the AprilTags detected */
std::ostream& ad::operator<<(std::ostream& os, const ad::AprilTagsDetection& apriltags) {
    if(apriltags.count_target_apriltags() > 0) 
        os << "Target AprilTag IDs: ";
    for (const int apriltag_id : apriltags.target_apriltag_ids_)
        os << apriltag_id << ", ";
    os << "\b\b " << std::endl;
    
    if(apriltags.count_target_apriltags_detected() > 0)
        os << "\nTarget AprilTags detected:" << std::endl;
    for (const auto& [apriltag_id, apriltag_pose_color] : apriltags.target_apriltags_detected_) {
        std::string color;
        if(apriltag_pose_color.second.r != 0)
            color = "red";
        else if(apriltag_pose_color.second.g != 0)
            color = "green";
        else if(apriltag_pose_color.second.b != 0)
            color = "blue";
        else
            color = "no color";

        os << " - ID " << apriltag_id << " (" << color << "): " << "x = " << apriltag_pose_color.first.pose.position.x  << ", y = " << apriltag_pose_color.first.pose.position.y << std::endl;
    }
 
    if(apriltags.count_nontarget_apriltags_detected() > 0)
        os << "\nNon-target AprilTags detected:" << std::endl;
    for (const auto& [apriltag_id, apriltag_pose_color] : apriltags.nontarget_apriltags_detected_) {
        std::string color;
        if(apriltag_pose_color.second.r != 0)
            color = "red";
        else if(apriltag_pose_color.second.g != 0)
            color = "green";
        else if(apriltag_pose_color.second.b != 0)
            color = "blue";
        else
            color = "no color";

        os << " - ID " << apriltag_id << " (" << color << "): " << "x = " << apriltag_pose_color.first.pose.position.x  << ", y = " << apriltag_pose_color.first.pose.position.y << std::endl;
    }
 
    return os;
}

/* Function to insert an AprilTag*/
void ad::AprilTagsDetection::insert_apriltag(int apriltag_id, geometry_msgs::PoseStamped apriltag_pose_stamped, std_msgs::ColorRGBA apriltag_color) {
    bool is_target;
    for (const int target_apriltag_id : target_apriltag_ids_) {
        if(is_target = target_apriltag_id == apriltag_id)
            break;
    }

    // Insert AprilTag pose and color
    std::pair<int, std::pair<geometry_msgs::PoseStamped, std_msgs::ColorRGBA>> apriltag(apriltag_id, std::make_pair(apriltag_pose_stamped, apriltag_color));
    is_target ? target_apriltags_detected_.insert(apriltag) : nontarget_apriltags_detected_.insert(apriltag);

    // No update if no color received
    if(apriltag_color.r == 0 && apriltag_color.g == 0 && apriltag_color.b == 0)
        return;
    
    // Update if color received and current is no color
    std_msgs::ColorRGBA current_color = is_target ? target_apriltags_detected_[apriltag_id].second : nontarget_apriltags_detected_[apriltag_id].second;
    bool current_no_color = current_color.r == 0 && current_color.g == 0 && current_color.b == 0;
    if(is_target && current_no_color)
        target_apriltags_detected_[apriltag_id].second = apriltag_color;
}

/* Function to check if AprilTags detection is completed */
bool ad::AprilTagsDetection::is_apriltags_detection_completed() const {
    return count_target_apriltags() == count_target_apriltags_detected();
}