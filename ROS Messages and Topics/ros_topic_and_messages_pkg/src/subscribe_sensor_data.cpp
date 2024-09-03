#include <iostream>
#include <ros/ros.h>
#include "ros_topic_and_messages_pkg/SensorData.h"

void sensorDataCallback(const ros_topic_and_messages_pkg::SensorData::ConstPtr& data) {
    ROS_INFO("Received sensor data: Temperature=%f, Humidity=%f", data->temperature, data->humidity);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("sensor_topic", 10, sensorDataCallback);

    ros::spin();

    return 0;
}
