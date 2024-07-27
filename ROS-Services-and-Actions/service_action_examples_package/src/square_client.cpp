#include <ros/ros.h>
#include <service_action_examples_package/Square.h>

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "square_client");
    ros::NodeHandle nh;
    
    ROS_INFO("spp square_client");

    // Create a service client for 'calculate_square'
    ros::ServiceClient client = nh.serviceClient<service_action_examples_package::Square>("calculate_square");

    if (argc != 2)
    {
        ROS_INFO("Usage: square_client [number]");
        return 1;
    }

    // Create a request object and set the number
    service_action_examples_package::Square srv;
    srv.request.number = atoi(argv[1]);

    // Call the service and check the result
    if (client.call(srv))
    {
        ROS_INFO("The square of %d is %d", srv.request.number, srv.response.square);
    }
    else
    {
        ROS_ERROR("Failed tooooo call service calculate_square");
        return 1;
    }

    return 0;
}
