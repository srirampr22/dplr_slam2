#include <ros/ros.h>
#include <std_srvs/Trigger.h>  // Correct service header
#include <iostream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "save_map_service_caller");
    ros::NodeHandle nh;

    // Create a service client to call the 'save_map' service
    ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("save_map");

    // Service request and response objects
    std_srvs::Trigger srv;

    char input;
    std::cout << "Press 's' to call the save_map service. Press 'q' to quit." << std::endl;

    while (ros::ok()) {
        std::cin >> input;

        // Check if the input is 's' or 'S'
        if (input == 's' || input == 'S') {
            // Call the service
            if (client.call(srv)) {
                ROS_INFO("Successfully called save_map service: %s", srv.response.message.c_str());
            } else {
                ROS_ERROR("Failed to call save_map service");
            }
        }

        // Exit condition
        if (input == 'q' || input == 'Q') {
            break;
        }
    }

    return 0;
}
