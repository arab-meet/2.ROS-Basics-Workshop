
#include <ros/ros.h>
#include <ros_topic_and_messages_pkg/Position.h>  

int main(int argc, char** argv)
{
    ros::init(argc, argv, "position_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<ros_topic_and_messages_pkg::Position>("robot_position", 10);
    ros::Rate rate(1);  // 1 Hz

    while (ros::ok())
    {
        ros_topic_and_messages_pkg::Position msg;
        msg.x = rand() % 10;  // Random x position
        msg.y = rand() % 10;  // Random y position
        msg.theta = static_cast<float>(rand()) / RAND_MAX * 3.14;  // Random theta (orientation)
        ROS_INFO("Publishing: x=%f, y=%f, theta=%f", msg.x, msg.y, msg.theta);
        pub.publish(msg);
        rate.sleep();
    }

    return 0;
}
