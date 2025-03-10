/* User-defined libraries */
#include <pick_place_action.h>

int main(int argc, char **argv) {
    // Node "pick_place_node" with arguments
    ros::init(argc, argv, "pick_place_node");

    // Create the action server "pick_place_action"
    pp::PickPlaceAction pick_place_action("pick_place_action");

    // Spin the callback function
    ros::spin();

    return 0;
}