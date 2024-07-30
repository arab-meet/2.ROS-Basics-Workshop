#include <ros/ros.h>
#include <service_action_examples_package/Square.h>

// Callback function for handling service requests
bool handleSquare(service_action_examples_package::Square::Request &req,
                  service_action_examples_package::Square::Response &res)
{
    ROS_INFO("Received request: %d", req.number);
    // Compute the square of the number
    res.square = req.number * req.number;
    ROS_INFO("Returning response: %d", res.square);
    return true;
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "square_server");
    ros::NodeHandle nh;

    // Create a service named 'calculate_square' of type Square
    ros::ServiceServer service = nh.advertiseService("calculate_square", handleSquare);
    
    ROS_INFO("cpp node : Ready to calculate the square of a number.");
    
    // Keep the node running
    ros::spin();

    return 0;
}
