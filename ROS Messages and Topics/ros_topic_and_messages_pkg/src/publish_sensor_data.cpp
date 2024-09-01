#include <iostream>
#include <ros/ros.h>
#include "ros_topic_and_messages_pkg/SensorData.h"
#include <random>

void publish_sensor_data(int argc, char** argv) {
    ros::init(argc, argv, "sensor_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<ros_topic_and_messages_pkg::SensorData>("sensor_topic", 10);
    ros::Rate rate(1);  // 1 Hz

    std::default_random_engine generator;
    std::uniform_real_distribution<double> temperature_dist(20.0, 30.0);
    std::uniform_real_distribution<double> humidity_dist(40.0, 60.0);

    while (ros::ok()) {
        ros_topic_and_messages_pkg::SensorData sensor_data;
        sensor_data.temperature = temperature_dist(generator);
        sensor_data.humidity = humidity_dist(generator);

        ROS_INFO("Publishing sensor data: Temperature=%f, Humidity=%f", sensor_data.temperature, sensor_data.humidity);
        pub.publish(sensor_data);
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    try {
        publish_sensor_data(argc, argv);
    } catch (const ros::Exception& e) {
        ROS_ERROR("ROS Exception: %s", e.what());
    }
    return 0;
}

