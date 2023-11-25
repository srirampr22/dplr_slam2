#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <iostream>

// Callback function for the save_map service
bool saveMapCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    ROS_INFO("save_map service has been triggered");

    // Set the response
    res.success = true;
    res.message = "Success";
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "save_map_service_server");
    ros::NodeHandle nh;

    // Create a service server for the 'save_map' service
    ros::ServiceServer service = nh.advertiseService("save_map", saveMapCallback);

    ROS_INFO("Service server for save_map has started.");

    // Spin to keep the service open
    ros::spin();

    return 0;
}
