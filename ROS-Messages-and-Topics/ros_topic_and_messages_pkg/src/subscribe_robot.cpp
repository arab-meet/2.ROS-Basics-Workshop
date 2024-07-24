#include <ros/ros.h>
#include <ros_topic_and_messages_pkg/Position.h>  

void callback(const ros_topic_and_messages_pkg::Position::ConstPtr& data)
{
    ROS_INFO("Received: x=%f, y=%f, theta=%f", data->x, data->y, data->theta);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "position_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("robot_position", 10, callback);
    ros::spin();

    return 0;
}